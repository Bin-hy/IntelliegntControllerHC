#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <duco_msg/msg/duco_robot_state.hpp>
#include <common_msgs/msg/collision_status.hpp>

#include <mutex>
#include <cmath>
#include <limits>
#include <vector>
#include <string>
#include <cstring>

using namespace std::chrono_literals;

class CollisionDetectorNode : public rclcpp::Node {
public:
    CollisionDetectorNode() : Node("collision_detector") {
        declare_parameter("point_cloud_topic",  "/camera/depth/points");
        declare_parameter("link_names", std::vector<std::string>{
            "base_link", "link_1", "link_2", "link_3",
            "link_4",    "link_5", "link_6"
        });
        declare_parameter("link_radii", std::vector<double>{
            0.15, 0.12, 0.12, 0.10, 0.10, 0.10, 0.08
        });
        declare_parameter("self_filter_margin", 0.04);
        declare_parameter("emergency_threshold", 0.03);
        declare_parameter("warning_threshold",   0.10);
        declare_parameter("caution_threshold",   0.20);
        declare_parameter("detection_hz",        10.0);
        declare_parameter("downsample_stride",   6);
        declare_parameter("auto_pause",          true);

        pc_topic_      = get_parameter("point_cloud_topic").as_string();
        link_names_    = get_parameter("link_names").as_string_array();
        link_radii_    = get_parameter("link_radii").as_double_array();
        self_margin_   = get_parameter("self_filter_margin").as_double();
        thr_emergency_ = get_parameter("emergency_threshold").as_double();
        thr_warning_   = get_parameter("warning_threshold").as_double();
        thr_caution_   = get_parameter("caution_threshold").as_double();
        stride_        = get_parameter("downsample_stride").as_int();
        auto_pause_    = get_parameter("auto_pause").as_bool();
        double hz      = get_parameter("detection_hz").as_double();

        while (link_radii_.size() < link_names_.size()) link_radii_.push_back(0.10);

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        subscribe_point_cloud(pc_topic_);

        sub_set_topic_ = create_subscription<std_msgs::msg::String>(
            "/collision_detector/set_topic", 10,
            [this](std_msgs::msg::String::SharedPtr msg) {
                if (msg->data.empty() || msg->data == pc_topic_) return;
                RCLCPP_INFO(get_logger(), "Switching: %s → %s", pc_topic_.c_str(), msg->data.c_str());
                pc_topic_ = msg->data;
                { std::lock_guard<std::mutex> lk(pc_mutex_); last_pc_.reset(); }
                subscribe_point_cloud(pc_topic_);
            });

        sub_state_ = create_subscription<duco_msg::msg::DucoRobotState>(
            "/duco_cobot/robot_state", 10,
            [this](duco_msg::msg::DucoRobotState::SharedPtr msg) {
                if (msg->collision) hw_collision_.store(true);
            });

        pub_status_ = create_publisher<common_msgs::msg::CollisionStatus>("/collision_detector/status", 10);
        client_pause_ = create_client<std_srvs::srv::SetBool>("/system/pause_task");

        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / hz));
        timer_ = create_wall_timer(period, [this]() { run_detection(); });

        RCLCPP_INFO(get_logger(), "CollisionDetector started | topic=%s", pc_topic_.c_str());
    }

private:
    void subscribe_point_cloud(const std::string& topic) {
        sub_pc_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(pc_mutex_);
                last_pc_ = msg;
            });
    }

    void run_detection() {
        auto out = common_msgs::msg::CollisionStatus();

        if (hw_collision_.exchange(false)) {
            out.status = "emergency"; out.min_distance = 0.0;
            out.nearest_link = "hardware"; out.obstacle_points = 0;
            out.message = "Hardware collision detected";
            pub_status_->publish(out); try_pause();
            RCLCPP_ERROR(get_logger(), "%s", out.message.c_str());
            return;
        }

        sensor_msgs::msg::PointCloud2::SharedPtr pc;
        { std::lock_guard<std::mutex> lk(pc_mutex_); pc = last_pc_; }

        if (!pc) {
            out.status = "sensor_unavailable"; out.message = "Waiting for point cloud";
            pub_status_->publish(out); return;
        }

        double age = (now() - rclcpp::Time(pc->header.stamp)).seconds();
        if (age > 1.0) {
            out.status = "sensor_unavailable"; out.message = "Point cloud stale";
            pub_status_->publish(out); return;
        }

        const std::string& cloud_frame = pc->header.frame_id;
        rclcpp::Time cloud_time(pc->header.stamp);

        struct Sphere { double x, y, z, r; std::string name; };
        std::vector<Sphere> spheres;
        for (size_t i = 0; i < link_names_.size(); ++i) {
            try {
                auto tf = tf_buffer_->lookupTransform(cloud_frame, link_names_[i], cloud_time, 80ms);
                spheres.push_back({tf.transform.translation.x, tf.transform.translation.y,
                                   tf.transform.translation.z, link_radii_[i], link_names_[i]});
            } catch (const tf2::TransformException&) {}
        }
        if (spheres.empty()) {
            out.status = "tf_unavailable"; out.message = "No link transforms";
            pub_status_->publish(out); return;
        }

        int off_x = -1, off_y = -1, off_z = -1;
        for (const auto& f : pc->fields) {
            if (f.name == "x") off_x = f.offset;
            else if (f.name == "y") off_y = f.offset;
            else if (f.name == "z") off_z = f.offset;
        }
        if (off_x < 0 || off_y < 0 || off_z < 0) {
            out.status = "error"; out.message = "Missing xyz fields";
            pub_status_->publish(out); return;
        }

        const uint8_t* data = pc->data.data();
        uint32_t pstep = pc->point_step;
        size_t npts = static_cast<size_t>(pc->width) * pc->height;
        double global_min = std::numeric_limits<double>::max();
        std::string nearest_link;
        int obstacle_cnt = 0;

        for (size_t idx = 0; idx < npts; idx += static_cast<size_t>(stride_)) {
            const uint8_t* pt = data + idx * pstep;
            float fx, fy, fz;
            std::memcpy(&fx, pt + off_x, sizeof(float));
            std::memcpy(&fy, pt + off_y, sizeof(float));
            std::memcpy(&fz, pt + off_z, sizeof(float));
            if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(fz)) continue;

            double px = fx, py = fy, pz = fz;
            bool is_self = false;
            for (const auto& s : spheres) {
                double dx = px-s.x, dy = py-s.y, dz = pz-s.z;
                if ((dx*dx+dy*dy+dz*dz) < (s.r+self_margin_)*(s.r+self_margin_)) { is_self = true; break; }
            }
            if (is_self) continue;

            for (const auto& s : spheres) {
                double dx = px-s.x, dy = py-s.y, dz = pz-s.z;
                double ds = std::sqrt(dx*dx+dy*dy+dz*dz) - s.r;
                if (ds < global_min) { global_min = ds; nearest_link = s.name; }
            }
            if (global_min < thr_caution_) obstacle_cnt++;
        }

        out.nearest_link = nearest_link;
        out.obstacle_points = obstacle_cnt;
        out.min_distance = (global_min == std::numeric_limits<double>::max()) ? 999.0 : global_min;

        auto fmt = [](double d) { char b[32]; std::snprintf(b,32,"%.1fcm",d*100); return std::string(b); };
        if (global_min <= thr_emergency_) {
            out.status = "emergency"; out.message = "EMERGENCY: " + fmt(global_min) + " to " + nearest_link;
            try_pause(); RCLCPP_ERROR(get_logger(), "%s", out.message.c_str());
        } else if (global_min <= thr_warning_) {
            out.status = "warning"; out.message = "WARNING: " + fmt(global_min) + " to " + nearest_link;
            RCLCPP_WARN(get_logger(), "%s", out.message.c_str());
        } else if (global_min <= thr_caution_) {
            out.status = "caution"; out.message = "CAUTION: " + fmt(global_min) + " to " + nearest_link;
        } else {
            out.status = "safe"; out.message = "Safe";
        }
        pub_status_->publish(out);
    }

    void try_pause() {
        if (!auto_pause_ || !client_pause_->service_is_ready()) return;
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = true;
        client_pause_->async_send_request(req);
    }

    std::string pc_topic_;
    std::vector<std::string> link_names_;
    std::vector<double> link_radii_;
    double self_margin_, thr_emergency_, thr_warning_, thr_caution_;
    int stride_; bool auto_pause_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    sensor_msgs::msg::PointCloud2::SharedPtr last_pc_;
    std::mutex pc_mutex_;
    std::atomic<bool> hw_collision_{false};

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_set_topic_;
    rclcpp::Subscription<duco_msg::msg::DucoRobotState>::SharedPtr sub_state_;
    rclcpp::Publisher<common_msgs::msg::CollisionStatus>::SharedPtr pub_status_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_pause_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CollisionDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
