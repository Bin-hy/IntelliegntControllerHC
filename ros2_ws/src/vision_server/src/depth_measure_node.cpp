#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <common_msgs/msg/depth_measurement.hpp>
#include "vision_server/srv/measure_depth.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;
using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// DepthMeasureNode
//
// Subscribes to a depth image (Y16 / 16-bit mm) and CameraInfo,
// measures the depth of the closest object in a configurable ROI,
// computes its 3D position in camera frame, and publishes the result.
//
// Also provides a MeasureDepth service to snapshot the current
// measurement and optionally save it to a CSV file.
// ---------------------------------------------------------------------------
class DepthMeasureNode : public rclcpp::Node {
public:
    DepthMeasureNode() : Node("depth_measure_node") {
        // --- Parameters ---
        declare_parameter("camera_ns",       "/camera");
        declare_parameter("roi_width",       200);    // pixels
        declare_parameter("roi_height",      200);    // pixels
        declare_parameter("roi_center_u",    -1);     // -1 = image center
        declare_parameter("roi_center_v",    -1);     // -1 = image center
        declare_parameter("min_depth_mm",    100.0);  // filter out < 10cm
        declare_parameter("max_depth_mm",    5000.0); // filter out > 5m
        declare_parameter("cluster_range_mm", 50.0);  // depth cluster tolerance
        declare_parameter("publish_hz",      10.0);
        declare_parameter("save_dir",        std::string(std::getenv("HOME") ? std::getenv("HOME") : ".") + "/.ros/depth_measure");

        camera_ns_      = get_parameter("camera_ns").as_string();
        roi_w_          = get_parameter("roi_width").as_int();
        roi_h_          = get_parameter("roi_height").as_int();
        roi_cu_         = get_parameter("roi_center_u").as_int();
        roi_cv_         = get_parameter("roi_center_v").as_int();
        min_depth_mm_   = get_parameter("min_depth_mm").as_double();
        max_depth_mm_   = get_parameter("max_depth_mm").as_double();
        cluster_range_  = get_parameter("cluster_range_mm").as_double();
        save_dir_       = get_parameter("save_dir").as_string();

        double hz = get_parameter("publish_hz").as_double();

        // --- Subscribe to camera ---
        subscribe_camera(camera_ns_);

        // --- Dynamic camera switch ---
        sub_set_topic_ = create_subscription<std_msgs::msg::String>(
            "/depth_measure/set_camera", 10,
            [this](std_msgs::msg::String::SharedPtr msg) {
                if (msg->data.empty() || msg->data == camera_ns_) return;
                RCLCPP_INFO(get_logger(), "Switching camera: %s → %s",
                    camera_ns_.c_str(), msg->data.c_str());
                camera_ns_ = msg->data;
                {
                    std::lock_guard<std::mutex> lk(data_mutex_);
                    last_depth_.reset();
                    has_camera_info_ = false;
                }
                subscribe_camera(camera_ns_);
            });

        // --- Publisher ---
        pub_ = create_publisher<common_msgs::msg::DepthMeasurement>(
            "/depth_measure/result", 10);

        // --- Service ---
        srv_ = create_service<vision_server::srv::MeasureDepth>(
            "/depth_measure/measure",
            std::bind(&DepthMeasureNode::handle_measure, this,
                      std::placeholders::_1, std::placeholders::_2));

        // --- Timer ---
        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / hz));
        timer_ = create_wall_timer(period, [this]() { measure_and_publish(); });

        RCLCPP_INFO(get_logger(),
            "DepthMeasureNode started | camera=%s | ROI=%dx%d | depth range=[%.0f, %.0f]mm",
            camera_ns_.c_str(), roi_w_, roi_h_, min_depth_mm_, max_depth_mm_);
    }

private:
    // -----------------------------------------------------------------------
    void subscribe_camera(const std::string& ns) {
        std::string depth_topic = ns + "/depth/image_raw";
        std::string info_topic  = ns + "/depth/camera_info";

        sub_depth_ = create_subscription<sensor_msgs::msg::Image>(
            depth_topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(data_mutex_);
                last_depth_ = msg;
            });

        sub_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            info_topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(data_mutex_);
                // K = [fx 0 cx; 0 fy cy; 0 0 1]
                fx_ = msg->k[0];
                fy_ = msg->k[4];
                cx_ = msg->k[2];
                cy_ = msg->k[5];
                has_camera_info_ = true;
            });

        RCLCPP_INFO(get_logger(), "Subscribed: %s, %s",
            depth_topic.c_str(), info_topic.c_str());
    }

    // -----------------------------------------------------------------------
    struct MeasureResult {
        bool valid = false;
        double depth_mm = 0;
        double pos_x = 0, pos_y = 0, pos_z = 0;
        int center_u = 0, center_v = 0;
        int valid_points = 0;
        std::string status;
    };

    MeasureResult do_measure() {
        MeasureResult res;

        sensor_msgs::msg::Image::SharedPtr depth_msg;
        double fx, fy, cx, cy;
        bool has_info;
        {
            std::lock_guard<std::mutex> lk(data_mutex_);
            depth_msg = last_depth_;
            fx = fx_; fy = fy_; cx = cx_; cy = cy_;
            has_info = has_camera_info_;
        }

        if (!depth_msg) {
            res.status = "no_depth";
            return res;
        }
        if (!has_info) {
            res.status = "no_camera_info";
            return res;
        }

        // Convert to cv::Mat (16-bit)
        cv::Mat depth;
        try {
            auto cv_ptr = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
            depth = cv_ptr->image;
        } catch (const std::exception& e) {
            RCLCPP_DEBUG(get_logger(), "cv_bridge error: %s", e.what());
            res.status = "no_depth";
            return res;
        }

        int img_w = depth.cols, img_h = depth.rows;
        if (img_w == 0 || img_h == 0) {
            res.status = "no_depth";
            return res;
        }

        // Determine ROI center
        int cu = (roi_cu_ >= 0) ? roi_cu_ : img_w / 2;
        int cv = (roi_cv_ >= 0) ? roi_cv_ : img_h / 2;

        // Clamp ROI to image bounds
        int x0 = std::max(0, cu - roi_w_ / 2);
        int y0 = std::max(0, cv - roi_h_ / 2);
        int x1 = std::min(img_w, x0 + roi_w_);
        int y1 = std::min(img_h, y0 + roi_h_);

        // Collect valid depths in ROI
        std::vector<std::pair<uint16_t, std::pair<int,int>>> depth_pixels; // (depth, (u, v))
        for (int y = y0; y < y1; ++y) {
            const uint16_t* row = depth.ptr<uint16_t>(y);
            for (int x = x0; x < x1; ++x) {
                uint16_t d = row[x];
                if (d > 0 &&
                    static_cast<double>(d) >= min_depth_mm_ &&
                    static_cast<double>(d) <= max_depth_mm_) {
                    depth_pixels.push_back({d, {x, y}});
                }
            }
        }

        if (depth_pixels.empty()) {
            res.status = "no_depth";
            res.center_u = cu;
            res.center_v = cv;
            return res;
        }

        // Sort by depth to find closest object cluster
        std::sort(depth_pixels.begin(), depth_pixels.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

        uint16_t min_depth = depth_pixels[0].first;
        double threshold = min_depth + cluster_range_;

        // Average depth and centroid of the closest cluster
        double sum_depth = 0;
        double sum_u = 0, sum_v = 0;
        int count = 0;

        for (const auto& [d, uv] : depth_pixels) {
            if (static_cast<double>(d) > threshold) break;
            sum_depth += d;
            sum_u += uv.first;
            sum_v += uv.second;
            count++;
        }

        double avg_depth_mm = sum_depth / count;
        double avg_u = sum_u / count;
        double avg_v = sum_v / count;

        // 3D position in camera frame
        double Z = avg_depth_mm / 1000.0;  // mm → meters
        double X = (avg_u - cx) * Z / fx;
        double Y = (avg_v - cy) * Z / fy;

        res.valid = true;
        res.depth_mm = avg_depth_mm;
        res.pos_x = X;
        res.pos_y = Y;
        res.pos_z = Z;
        res.center_u = static_cast<int>(avg_u);
        res.center_v = static_cast<int>(avg_v);
        res.valid_points = count;
        res.status = "measuring";
        return res;
    }

    // -----------------------------------------------------------------------
    void measure_and_publish() {
        auto res = do_measure();

        auto msg = common_msgs::msg::DepthMeasurement();
        msg.depth_mm     = res.depth_mm;
        msg.pos_x        = res.pos_x;
        msg.pos_y        = res.pos_y;
        msg.pos_z        = res.pos_z;
        msg.roi_center_u = res.center_u;
        msg.roi_center_v = res.center_v;
        msg.valid_points = res.valid_points;
        msg.status       = res.status;
        msg.camera_ns    = camera_ns_;
        pub_->publish(msg);
    }

    // -----------------------------------------------------------------------
    void handle_measure(
        const std::shared_ptr<vision_server::srv::MeasureDepth::Request> request,
        std::shared_ptr<vision_server::srv::MeasureDepth::Response> response)
    {
        auto res = do_measure();
        if (!res.valid) {
            response->success  = false;
            response->message  = "Measurement failed: " + res.status;
            return;
        }

        response->success  = true;
        response->depth_mm = res.depth_mm;
        response->pos_x    = res.pos_x;
        response->pos_y    = res.pos_y;
        response->pos_z    = res.pos_z;

        char buf[256];
        std::snprintf(buf, sizeof(buf),
            "Depth=%.1fmm  Pos=(%.4f, %.4f, %.4f)m  Points=%d",
            res.depth_mm, res.pos_x, res.pos_y, res.pos_z, res.valid_points);
        response->message = buf;

        // Save to CSV if file_tag is provided
        if (!request->file_tag.empty()) {
            try {
                fs::path base(save_dir_);
                fs::path tag_path(request->file_tag);
                fs::path target_dir = base;
                std::string prefix;
                if (tag_path.has_parent_path()) {
                    target_dir /= tag_path.parent_path();
                    prefix = tag_path.filename().string();
                } else {
                    prefix = tag_path.string();
                }
                if (prefix.empty()) prefix = "depth";

                if (!fs::exists(target_dir)) {
                    fs::create_directories(target_dir);
                }

                auto stamp = this->now().nanoseconds();
                fs::path csv_file = target_dir / (prefix + "_" + std::to_string(stamp) + ".csv");

                std::ofstream ofs(csv_file.string());
                if (ofs.is_open()) {
                    ofs << "depth_mm,pos_x,pos_y,pos_z,roi_u,roi_v,valid_points,camera_ns\n";
                    ofs << res.depth_mm << ","
                        << res.pos_x << ","
                        << res.pos_y << ","
                        << res.pos_z << ","
                        << res.center_u << ","
                        << res.center_v << ","
                        << res.valid_points << ","
                        << camera_ns_ << "\n";
                    ofs.close();
                    response->message += " | Saved: " + csv_file.string();
                    RCLCPP_INFO(get_logger(), "Measurement saved: %s", csv_file.string().c_str());
                }
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "Failed to save CSV: %s", e.what());
            }
        }
    }

    // --- Parameters ---
    std::string camera_ns_;
    int roi_w_, roi_h_, roi_cu_, roi_cv_;
    double min_depth_mm_, max_depth_mm_, cluster_range_;
    std::string save_dir_;

    // --- Camera intrinsics ---
    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
    bool has_camera_info_ = false;

    // --- State ---
    sensor_msgs::msg::Image::SharedPtr last_depth_;
    std::mutex data_mutex_;

    // --- ROS ---
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr       sub_depth_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr  sub_info_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr         sub_set_topic_;
    rclcpp::Publisher<common_msgs::msg::DepthMeasurement>::SharedPtr pub_;
    rclcpp::Service<vision_server::srv::MeasureDepth>::SharedPtr   srv_;
    rclcpp::TimerBase::SharedPtr                                   timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthMeasureNode>());
    rclcpp::shutdown();
    return 0;
}
