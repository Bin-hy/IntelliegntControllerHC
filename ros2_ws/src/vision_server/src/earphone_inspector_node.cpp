#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include "vision_server/srv/capture_baseline.hpp"
#include "vision_server/srv/measure_earphone.hpp"

#include <mutex>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <Eigen/Dense>

namespace fs = std::filesystem;

// ---------------------------------------------------------------------------
// EarphoneInspectorNode
//
// Measures in-ear earphone insertion angle and depth using a depth camera
// facing the ear from the front.
//
// Workflow:
//   1. call capture_baseline  → stores the "ear only" depth frame
//   2. robot places the earphone
//   3. call measure_earphone  → computes angle & depth via depth-differencing + PCA
// ---------------------------------------------------------------------------
class EarphoneInspectorNode : public rclcpp::Node {
public:
    EarphoneInspectorNode() : Node("earphone_inspector_node") {
        // --- Parameters ---
        declare_parameter("camera_ns",          "/camera");
        declare_parameter("min_diff_mm",        3.0);    // min depth change to count as earphone
        declare_parameter("max_diff_mm",        80.0);   // max depth change (filter outliers)
        declare_parameter("min_area_pixels",    50);     // min earphone region size
        declare_parameter("roi_width",          300);    // ROI around image center
        declare_parameter("roi_height",         300);
        declare_parameter("save_dir", std::string(
            std::getenv("HOME") ? std::getenv("HOME") : ".") + "/.ros/earphone_inspection");

        camera_ns_     = get_parameter("camera_ns").as_string();
        min_diff_mm_   = get_parameter("min_diff_mm").as_double();
        max_diff_mm_   = get_parameter("max_diff_mm").as_double();
        min_area_       = get_parameter("min_area_pixels").as_int();
        roi_w_         = get_parameter("roi_width").as_int();
        roi_h_         = get_parameter("roi_height").as_int();
        save_dir_      = get_parameter("save_dir").as_string();

        // --- Subscribe to camera ---
        auto depth_topic = camera_ns_ + "/depth/image_raw";
        auto color_topic = camera_ns_ + "/color/image_raw";
        auto info_topic  = camera_ns_ + "/depth/camera_info";

        sub_depth_ = create_subscription<sensor_msgs::msg::Image>(
            depth_topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(mtx_);
                last_depth_ = msg;
            });

        sub_color_ = create_subscription<sensor_msgs::msg::Image>(
            color_topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(mtx_);
                last_color_ = msg;
            });

        sub_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            info_topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(mtx_);
                fx_ = msg->k[0]; fy_ = msg->k[4];
                cx_ = msg->k[2]; cy_ = msg->k[5];
                has_info_ = true;
            });

        // --- Services ---
        srv_baseline_ = create_service<vision_server::srv::CaptureBaseline>(
            "capture_baseline",
            std::bind(&EarphoneInspectorNode::handle_baseline, this,
                      std::placeholders::_1, std::placeholders::_2));

        srv_measure_ = create_service<vision_server::srv::MeasureEarphone>(
            "measure",
            std::bind(&EarphoneInspectorNode::handle_measure, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(),
            "EarphoneInspectorNode started | camera=%s | ROI=%dx%d | diff=[%.0f,%.0f]mm",
            camera_ns_.c_str(), roi_w_, roi_h_, min_diff_mm_, max_diff_mm_);
    }

private:
    // -----------------------------------------------------------------------
    // CaptureBaseline: store the current depth as the "ear only" reference
    // -----------------------------------------------------------------------
    void handle_baseline(
        const std::shared_ptr<vision_server::srv::CaptureBaseline::Request>,
        std::shared_ptr<vision_server::srv::CaptureBaseline::Response> resp)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!last_depth_) {
            resp->success = false;
            resp->message = "No depth frame available";
            return;
        }
        try {
            auto cv_ptr = cv_bridge::toCvShare(last_depth_,
                sensor_msgs::image_encodings::TYPE_16UC1);
            baseline_depth_ = cv_ptr->image.clone();

            if (last_color_) {
                auto cv_c = cv_bridge::toCvShare(last_color_, "bgr8");
                baseline_color_ = cv_c->image.clone();
            }
        } catch (const std::exception& e) {
            resp->success = false;
            resp->message = std::string("cv_bridge error: ") + e.what();
            return;
        }

        has_baseline_ = true;
        resp->success = true;
        resp->message = "Baseline captured (" +
            std::to_string(baseline_depth_.cols) + "x" +
            std::to_string(baseline_depth_.rows) + ")";
        RCLCPP_INFO(get_logger(), "%s", resp->message.c_str());
    }

    // -----------------------------------------------------------------------
    // MeasureEarphone: compute angle and depth via depth differencing + PCA
    // -----------------------------------------------------------------------
    void handle_measure(
        const std::shared_ptr<vision_server::srv::MeasureEarphone::Request> req,
        std::shared_ptr<vision_server::srv::MeasureEarphone::Response> resp)
    {
        // Grab current frames
        cv::Mat cur_depth, cur_color;
        cv::Mat base_depth, base_color;
        double fx, fy, cx, cy;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (!has_baseline_) {
                resp->success = false;
                resp->message = "No baseline captured. Call capture_baseline first.";
                return;
            }
            if (!last_depth_) {
                resp->success = false;
                resp->message = "No depth frame available";
                return;
            }
            if (!has_info_) {
                resp->success = false;
                resp->message = "No camera intrinsics available";
                return;
            }
            try {
                auto cv_d = cv_bridge::toCvShare(last_depth_,
                    sensor_msgs::image_encodings::TYPE_16UC1);
                cur_depth = cv_d->image.clone();
                if (last_color_) {
                    auto cv_c = cv_bridge::toCvShare(last_color_, "bgr8");
                    cur_color = cv_c->image.clone();
                }
            } catch (const std::exception& e) {
                resp->success = false;
                resp->message = std::string("cv_bridge error: ") + e.what();
                return;
            }
            base_depth = baseline_depth_.clone();
            base_color = baseline_color_.clone();
            fx = fx_; fy = fy_; cx = cx_; cy = cy_;
        }

        // Check size match
        if (base_depth.size() != cur_depth.size()) {
            resp->success = false;
            resp->message = "Depth frame size mismatch with baseline";
            return;
        }

        int img_w = cur_depth.cols, img_h = cur_depth.rows;

        // ROI centered on image
        int x0 = std::max(0, img_w / 2 - roi_w_ / 2);
        int y0 = std::max(0, img_h / 2 - roi_h_ / 2);
        int x1 = std::min(img_w, x0 + roi_w_);
        int y1 = std::min(img_h, y0 + roi_h_);

        // --- Step 1: Depth differencing to extract earphone region ---
        // baseline is ear-only (further from camera), current has earphone (closer)
        // diff = baseline - current > 0 means something is now closer → earphone
        cv::Mat mask = cv::Mat::zeros(img_h, img_w, CV_8UC1);
        std::vector<Eigen::Vector3d> points_3d;

        for (int y = y0; y < y1; ++y) {
            const uint16_t* row_base = base_depth.ptr<uint16_t>(y);
            const uint16_t* row_cur  = cur_depth.ptr<uint16_t>(y);
            uint8_t* row_mask = mask.ptr<uint8_t>(y);
            for (int x = x0; x < x1; ++x) {
                uint16_t db = row_base[x];
                uint16_t dc = row_cur[x];
                // Both must be valid
                if (db == 0 || dc == 0) continue;
                double diff = static_cast<double>(db) - static_cast<double>(dc);
                if (diff >= min_diff_mm_ && diff <= max_diff_mm_) {
                    row_mask[x] = 255;
                    // Reproject to 3D using current depth
                    double Z = dc / 1000.0;  // mm → m
                    double X = (x - cx) * Z / fx;
                    double Y = (y - cy) * Z / fy;
                    points_3d.emplace_back(X, Y, Z);
                }
            }
        }

        // --- Step 2: Morphological cleanup ---
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        // Find largest contour (the earphone)
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (contours.empty()) {
            resp->success = false;
            resp->message = "No earphone region detected (no depth difference)";
            return;
        }

        // Select largest contour
        size_t largest_idx = 0;
        double largest_area = 0;
        for (size_t i = 0; i < contours.size(); ++i) {
            double a = cv::contourArea(contours[i]);
            if (a > largest_area) { largest_area = a; largest_idx = i; }
        }

        if (largest_area < min_area_) {
            resp->success = false;
            resp->message = "Earphone region too small (" +
                std::to_string(static_cast<int>(largest_area)) + " px < " +
                std::to_string(min_area_) + ")";
            return;
        }

        // Create refined mask from largest contour only
        cv::Mat earphone_mask = cv::Mat::zeros(img_h, img_w, CV_8UC1);
        cv::drawContours(earphone_mask, contours, static_cast<int>(largest_idx),
                         cv::Scalar(255), cv::FILLED);

        // Rebuild 3D points from refined mask
        points_3d.clear();
        std::vector<double> depth_diffs;
        for (int y = y0; y < y1; ++y) {
            const uint16_t* row_base = base_depth.ptr<uint16_t>(y);
            const uint16_t* row_cur  = cur_depth.ptr<uint16_t>(y);
            const uint8_t* row_mask  = earphone_mask.ptr<uint8_t>(y);
            for (int x = x0; x < x1; ++x) {
                if (row_mask[x] == 0) continue;
                uint16_t db = row_base[x];
                uint16_t dc = row_cur[x];
                if (db == 0 || dc == 0) continue;
                double diff = static_cast<double>(db) - static_cast<double>(dc);
                if (diff < min_diff_mm_ || diff > max_diff_mm_) continue;

                double Z = dc / 1000.0;
                double X = (x - cx) * Z / fx;
                double Y = (y - cy) * Z / fy;
                points_3d.emplace_back(X, Y, Z);
                depth_diffs.push_back(diff);
            }
        }

        if (static_cast<int>(points_3d.size()) < min_area_) {
            resp->success = false;
            resp->message = "Too few valid 3D points (" +
                std::to_string(points_3d.size()) + ")";
            return;
        }

        // --- Step 3: PCA to find earphone principal axis ---
        Eigen::Vector3d centroid(0, 0, 0);
        for (const auto& p : points_3d) centroid += p;
        centroid /= static_cast<double>(points_3d.size());

        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for (const auto& p : points_3d) {
            Eigen::Vector3d d = p - centroid;
            cov += d * d.transpose();
        }
        cov /= static_cast<double>(points_3d.size());

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
        // Eigenvalues in ascending order; largest = first principal component
        Eigen::Vector3d principal_axis = solver.eigenvectors().col(2).normalized();

        // Ensure the axis points roughly toward the camera (+Z direction for protrusion)
        if (principal_axis.z() < 0) principal_axis = -principal_axis;

        // --- Step 4: Compute angle relative to optical axis (Z) ---
        // Camera Z-axis ≈ ear canal direction (camera faces ear front-on)
        Eigen::Vector3d z_axis(0, 0, 1);
        double cos_angle = principal_axis.dot(z_axis);
        cos_angle = std::clamp(cos_angle, -1.0, 1.0);
        double angle_rad = std::acos(cos_angle);
        double angle_deg = angle_rad * 180.0 / M_PI;

        // --- Step 5: Compute insertion depth (median depth difference) ---
        std::sort(depth_diffs.begin(), depth_diffs.end());
        double median_diff = depth_diffs[depth_diffs.size() / 2];

        // --- Step 6: Confidence metric ---
        // Based on: number of points, eigenvalue ratio (elongation), area
        double eigen_ratio = solver.eigenvalues()(2) /
            std::max(solver.eigenvalues()(1), 1e-9);
        double area_score = std::min(1.0, largest_area / 500.0);
        double shape_score = std::min(1.0, eigen_ratio / 5.0);
        double point_score = std::min(1.0, static_cast<double>(points_3d.size()) / 200.0);
        double confidence = (area_score + shape_score + point_score) / 3.0;

        // --- Fill response ---
        resp->success    = true;
        resp->angle_deg  = angle_deg;
        resp->depth_mm   = median_diff;
        resp->confidence = confidence;

        char buf[512];
        std::snprintf(buf, sizeof(buf),
            "Angle=%.1f°  Depth=%.1fmm  Confidence=%.2f  Points=%zu  "
            "Axis=(%.3f,%.3f,%.3f)",
            angle_deg, median_diff, confidence, points_3d.size(),
            principal_axis.x(), principal_axis.y(), principal_axis.z());
        resp->message = buf;

        RCLCPP_INFO(get_logger(), "Measurement: %s", buf);

        // --- Step 7: Save debug images and result ---
        if (!req->file_tag.empty()) {
            resp->saved_path = save_results(req->file_tag,
                cur_depth, cur_color, base_depth, base_color,
                earphone_mask, contours[largest_idx],
                angle_deg, median_diff, confidence, principal_axis, centroid,
                fx, fy, cx, cy);
        }
    }

    // -----------------------------------------------------------------------
    // Save debug images and result JSON
    // -----------------------------------------------------------------------
    std::string save_results(
        const std::string& tag,
        const cv::Mat& cur_depth, const cv::Mat& cur_color,
        const cv::Mat& base_depth, const cv::Mat& base_color,
        const cv::Mat& mask, const std::vector<cv::Point>& contour,
        double angle_deg, double depth_mm, double confidence,
        const Eigen::Vector3d& axis, const Eigen::Vector3d& centroid,
        double fx, double fy, double cx, double cy)
    {
        auto stamp = this->now().nanoseconds();
        fs::path dir = fs::path(save_dir_) / tag /
            (std::to_string(stamp));
        try { fs::create_directories(dir); } catch (...) { return ""; }

        // Save depth images (normalized to 8-bit for visualization)
        auto save_depth_vis = [](const cv::Mat& d, const fs::path& path) {
            cv::Mat vis;
            cv::normalize(d, vis, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            cv::applyColorMap(vis, vis, cv::COLORMAP_JET);
            cv::imwrite(path.string(), vis);
        };

        save_depth_vis(base_depth, dir / "baseline_depth.png");
        save_depth_vis(cur_depth, dir / "measure_depth.png");

        if (!base_color.empty())
            cv::imwrite((dir / "baseline_color.png").string(), base_color);
        if (!cur_color.empty())
            cv::imwrite((dir / "measure_color.png").string(), cur_color);

        cv::imwrite((dir / "earphone_mask.png").string(), mask);

        // Debug overlay: draw contour + angle annotation on color image
        if (!cur_color.empty()) {
            cv::Mat overlay = cur_color.clone();
            // Scale contour coords if color and depth have different resolutions
            double sx = static_cast<double>(overlay.cols) / cur_depth.cols;
            double sy = static_cast<double>(overlay.rows) / cur_depth.rows;

            std::vector<cv::Point> scaled_contour;
            for (const auto& pt : contour) {
                scaled_contour.emplace_back(
                    static_cast<int>(pt.x * sx),
                    static_cast<int>(pt.y * sy));
            }
            cv::drawContours(overlay, std::vector<std::vector<cv::Point>>{scaled_contour},
                             0, cv::Scalar(0, 255, 0), 2);

            // Project centroid to image for annotation
            int cu = static_cast<int>(centroid.x() * fx / centroid.z() + cx);
            int cv_pt = static_cast<int>(centroid.y() * fy / centroid.z() + cy);
            cu = static_cast<int>(cu * sx);
            cv_pt = static_cast<int>(cv_pt * sy);

            // Draw axis direction
            double arrow_len = 60.0;
            int ax = cu + static_cast<int>(axis.x() / axis.z() * fx * 0.02 * sx * arrow_len);
            int ay = cv_pt + static_cast<int>(axis.y() / axis.z() * fy * 0.02 * sy * arrow_len);
            cv::arrowedLine(overlay, cv::Point(cu, cv_pt), cv::Point(ax, ay),
                            cv::Scalar(0, 0, 255), 2, cv::LINE_AA, 0, 0.3);

            // Text annotation
            char text[128];
            std::snprintf(text, sizeof(text), "Angle: %.1f deg", angle_deg);
            cv::putText(overlay, text, cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
            std::snprintf(text, sizeof(text), "Depth: %.1f mm", depth_mm);
            cv::putText(overlay, text, cv::Point(10, 60),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
            std::snprintf(text, sizeof(text), "Conf: %.2f", confidence);
            cv::putText(overlay, text, cv::Point(10, 90),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);

            cv::imwrite((dir / "debug_overlay.png").string(), overlay);
        }

        // Save result.json
        {
            std::ofstream ofs((dir / "result.json").string());
            if (ofs.is_open()) {
                ofs << "{\n"
                    << "  \"angle_deg\": " << angle_deg << ",\n"
                    << "  \"depth_mm\": " << depth_mm << ",\n"
                    << "  \"confidence\": " << confidence << ",\n"
                    << "  \"axis\": [" << axis.x() << ", " << axis.y() << ", " << axis.z() << "],\n"
                    << "  \"centroid_m\": [" << centroid.x() << ", " << centroid.y() << ", " << centroid.z() << "],\n"
                    << "  \"earphone_points\": " << 0 << ",\n"
                    << "  \"camera_ns\": \"" << camera_ns_ << "\",\n"
                    << "  \"timestamp_ns\": " << stamp << "\n"
                    << "}\n";
            }
        }

        RCLCPP_INFO(get_logger(), "Results saved to %s", dir.string().c_str());
        return dir.string();
    }

    // --- Parameters ---
    std::string camera_ns_;
    double min_diff_mm_, max_diff_mm_;
    int min_area_, roi_w_, roi_h_;
    std::string save_dir_;

    // --- Camera intrinsics ---
    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
    bool has_info_ = false;

    // --- Baseline ---
    cv::Mat baseline_depth_;
    cv::Mat baseline_color_;
    bool has_baseline_ = false;

    // --- Latest frames ---
    sensor_msgs::msg::Image::SharedPtr last_depth_;
    sensor_msgs::msg::Image::SharedPtr last_color_;
    std::mutex mtx_;

    // --- ROS ---
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      sub_depth_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      sub_color_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
    rclcpp::Service<vision_server::srv::CaptureBaseline>::SharedPtr srv_baseline_;
    rclcpp::Service<vision_server::srv::MeasureEarphone>::SharedPtr srv_measure_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EarphoneInspectorNode>());
    rclcpp::shutdown();
    return 0;
}
