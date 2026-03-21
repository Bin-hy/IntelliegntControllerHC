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

// ---------------------------------------------------------------------------
// CollisionDetectorNode
//
// Algorithm (per detection cycle):
//   1. Get latest PointCloud2 (already in camera optical frame).
//   2. For every robot link: look up its origin position IN the cloud frame
//      via TF (lookupTransform(cloud_frame, link_frame)). No point
//      transformation is needed – link positions come to the cloud frame.
//   3. Iterate subsampled point cloud.  For each finite point:
//      a. Self-filter: skip if inside any link sphere (radius + margin).
//      b. For remaining "external" points, compute distance to each link
//         surface (dist_to_center − link_radius).
//   4. Track global minimum surface distance and nearest link name.
//   5. Classify and publish CollisionStatus.
//   6. If emergency and auto_pause=true: call /system/pause_task.
// ---------------------------------------------------------------------------
class CollisionDetectorNode : public rclcpp::Node {
public:
    CollisionDetectorNode() : Node("collision_detector") {
        // ---- Parameters ----
        declare_parameter("point_cloud_topic",  "/camera/depth/points");
        declare_parameter("link_names", std::vector<std::string>{
            "base_link", "link_1", "link_2", "link_3",
            "link_4",    "link_5", "link_6"
        });
        // Sphere radii (m) approximating each link's physical extent.
        declare_parameter("link_radii", std::vector<double>{
            0.15, 0.12, 0.12, 0.10, 0.10, 0.10, 0.08
        });
        declare_parameter("self_filter_margin", 0.04);   // extra beyond link_radius
        declare_parameter("emergency_threshold", 0.03);  // m → trigger pause + "emergency"
        declare_parameter("warning_threshold",   0.10);  // m → "warning"
        declare_parameter("caution_threshold",   0.20);  // m → "caution"
        declare_parameter("detection_hz",        10.0);
        declare_parameter("downsample_stride",   6);     // use 1-in-N points
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

        // Pad radii array if shorter than link_names
        while (link_radii_.size() < link_names_.size()) {
            link_radii_.push_back(0.10);
        }

        // ---- TF ----
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ---- Subscriptions ----
        subscribe_point_cloud(pc_topic_);

        // Dynamic topic change: system_controller publishes the desired topic here
        sub_set_topic_ = create_subscription<std_msgs::msg::String>(
            "/collision_detector/set_topic", 10,
            [this](std_msgs::msg::String::SharedPtr msg) {
                const std::string& new_topic = msg->data;
                if (new_topic.empty() || new_topic == pc_topic_) return;
                RCLCPP_INFO(get_logger(),
                    "Switching point cloud topic: %s → %s", pc_topic_.c_str(), new_topic.c_str());
                pc_topic_ = new_topic;
                {
                    std::lock_guard<std::mutex> lk(pc_mutex_);
                    last_pc_.reset();
                }
                subscribe_point_cloud(pc_topic_);
            });

        sub_state_ = create_subscription<duco_msg::msg::DucoRobotState>(
            "/duco_cobot/robot_state", 10,
            [this](duco_msg::msg::DucoRobotState::SharedPtr msg) {
                if (msg->collision) {
                    hw_collision_.store(true);
                }
            });

        // ---- Publisher ----
        pub_status_ = create_publisher<common_msgs::msg::CollisionStatus>(
            "/collision_detector/status", 10);

        // ---- Pause service client ----
        client_pause_ = create_client<std_srvs::srv::SetBool>("/system/pause_task");

        // ---- Detection timer ----
        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / hz));
        timer_ = create_wall_timer(period, [this]() { run_detection(); });

        RCLCPP_INFO(get_logger(),
            "CollisionDetector started | topic=%s | emergency=%.3fm warning=%.3fm caution=%.3fm",
            pc_topic_.c_str(), thr_emergency_, thr_warning_, thr_caution_);
    }

private:
    // -----------------------------------------------------------------------
    void run_detection() {
        auto out = common_msgs::msg::CollisionStatus();

        // --- Check hardware collision flag (immediate, no vision needed) ---
        if (hw_collision_.exchange(false)) {
            out.status        = "emergency";
            out.min_distance  = 0.0;
            out.nearest_link  = "hardware";
            out.obstacle_points = 0;
            out.message       = "Hardware collision detected by DUCO controller";
            pub_status_->publish(out);
            try_pause();
            RCLCPP_ERROR(get_logger(), "%s", out.message.c_str());
            return;
        }

        // --- Get latest point cloud ---
        sensor_msgs::msg::PointCloud2::SharedPtr pc;
        {
            std::lock_guard<std::mutex> lk(pc_mutex_);
            pc = last_pc_;
        }

        if (!pc) {
            out.status  = "sensor_unavailable";
            out.message = "Waiting for point cloud on " + pc_topic_;
            pub_status_->publish(out);
            return;
        }

        // Reject stale clouds (> 1 s)
        double age = (now() - rclcpp::Time(pc->header.stamp)).seconds();
        if (age > 1.0) {
            out.status  = "sensor_unavailable";
            out.message = "Point cloud stale (" + std::to_string(age) + "s old)";
            pub_status_->publish(out);
            return;
        }

        const std::string& cloud_frame = pc->header.frame_id;
        rclcpp::Time       cloud_time(pc->header.stamp);

        // --- Resolve link positions in cloud frame via TF ---
        struct LinkSphere {
            double x, y, z, radius;
            std::string name;
        };
        std::vector<LinkSphere> spheres;
        spheres.reserve(link_names_.size());

        for (size_t i = 0; i < link_names_.size(); ++i) {
            try {
                // lookupTransform(target, source, time): converts source→target.
                // Translation = position of source origin expressed in target frame.
                auto tf = tf_buffer_->lookupTransform(
                    cloud_frame, link_names_[i], cloud_time, 80ms);
                spheres.push_back({
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z,
                    link_radii_[i],
                    link_names_[i]
                });
            } catch (const tf2::TransformException& ex) {
                RCLCPP_DEBUG(get_logger(), "TF %s→%s: %s",
                    cloud_frame.c_str(), link_names_[i].c_str(), ex.what());
            }
        }

        if (spheres.empty()) {
            out.status  = "tf_unavailable";
            out.message = "No robot link transforms available yet";
            pub_status_->publish(out);
            return;
        }

        // --- Parse PointCloud2 field offsets ---
        int off_x = -1, off_y = -1, off_z = -1;
        for (const auto& f : pc->fields) {
            if (f.name == "x") off_x = static_cast<int>(f.offset);
            else if (f.name == "y") off_y = static_cast<int>(f.offset);
            else if (f.name == "z") off_z = static_cast<int>(f.offset);
        }
        if (off_x < 0 || off_y < 0 || off_z < 0) {
            out.status  = "error";
            out.message = "Point cloud missing x/y/z fields";
            pub_status_->publish(out);
            return;
        }

        const uint8_t* data    = pc->data.data();
        uint32_t        pstep  = pc->point_step;
        size_t          npts   = static_cast<size_t>(pc->width) * pc->height;

        double      global_min  = std::numeric_limits<double>::max();
        std::string nearest_link;
        int         obstacle_cnt = 0;

        for (size_t idx = 0; idx < npts; idx += static_cast<size_t>(stride_)) {
            const uint8_t* pt = data + idx * pstep;
            float fx, fy, fz;
            std::memcpy(&fx, pt + off_x, sizeof(float));
            std::memcpy(&fy, pt + off_y, sizeof(float));
            std::memcpy(&fz, pt + off_z, sizeof(float));

            if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(fz)) continue;

            double px = fx, py = fy, pz = fz;

            // --- Self-filter ---
            bool is_self = false;
            for (const auto& s : spheres) {
                double dx = px - s.x, dy = py - s.y, dz = pz - s.z;
                if ((dx*dx + dy*dy + dz*dz) < (s.radius + self_margin_) * (s.radius + self_margin_)) {
                    is_self = true;
                    break;
                }
            }
            if (is_self) continue;

            // --- Surface distance to nearest link ---
            for (const auto& s : spheres) {
                double dx = px - s.x, dy = py - s.y, dz = pz - s.z;
                double dist_surface = std::sqrt(dx*dx + dy*dy + dz*dz) - s.radius;
                if (dist_surface < global_min) {
                    global_min   = dist_surface;
                    nearest_link = s.name;
                }
            }

            if (global_min < thr_caution_) {
                obstacle_cnt++;
            }
        }

        // --- Classify ---
        out.nearest_link    = nearest_link;
        out.obstacle_points = obstacle_cnt;
        out.min_distance    = (global_min == std::numeric_limits<double>::max()) ? 999.0 : global_min;

        auto fmt_dist = [](double d) {
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%.1f cm", d * 100.0);
            return std::string(buf);
        };

        if (global_min <= thr_emergency_) {
            out.status  = "emergency";
            out.message = "EMERGENCY: " + fmt_dist(global_min) + " to " + nearest_link;
            try_pause();
            RCLCPP_ERROR(get_logger(), "%s", out.message.c_str());
        } else if (global_min <= thr_warning_) {
            out.status  = "warning";
            out.message = "WARNING: " + fmt_dist(global_min) + " to " + nearest_link;
            RCLCPP_WARN(get_logger(), "%s", out.message.c_str());
        } else if (global_min <= thr_caution_) {
            out.status  = "caution";
            out.message = "CAUTION: " + fmt_dist(global_min) + " to " + nearest_link;
        } else {
            out.status  = "safe";
            out.message = "Safe";
        }

        pub_status_->publish(out);
    }

    // -----------------------------------------------------------------------
    void try_pause() {
        if (!auto_pause_) return;
        if (!client_pause_->service_is_ready()) return;
        auto req  = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = true;
        client_pause_->async_send_request(req);
        RCLCPP_ERROR(get_logger(), "Auto-pausing task due to collision risk");
    }

    // ---- Helpers ----
    void subscribe_point_cloud(const std::string& topic) {
        sub_pc_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(pc_mutex_);
                last_pc_ = msg;
            });
    }

    // ---- Parameters ----
    std::string              pc_topic_;
    std::vector<std::string> link_names_;
    std::vector<double>      link_radii_;
    double self_margin_;
    double thr_emergency_, thr_warning_, thr_caution_;
    int    stride_;
    bool   auto_pause_;

    // ---- State ----
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    sensor_msgs::msg::PointCloud2::SharedPtr last_pc_;
    std::mutex                               pc_mutex_;

    std::atomic<bool> hw_collision_{false};

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr   sub_pc_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr           sub_set_topic_;
    rclcpp::Subscription<duco_msg::msg::DucoRobotState>::SharedPtr   sub_state_;
    rclcpp::Publisher<common_msgs::msg::CollisionStatus>::SharedPtr  pub_status_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr                client_pause_;
    rclcpp::TimerBase::SharedPtr                                     timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CollisionDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
