// ---------------------------------------------------------------------------
// EarphoneInspectorNode — Robust Earphone Insertion Inspection Pipeline
//
// Redesigned for production-grade reliability:
//   - Multi-frame temporal median (reduces noise by ~2.8x)
//   - Adaptive ROI centered on detected ear surface
//   - Dual-channel detection: depth differencing + ear plane distance
//   - RGB fallback when depth channels fail
//   - Connected-component clustering (replaces morphological heuristics)
//   - Trimmed PCA with outlier rejection
//   - Optional ArUco marker for true ear-normal reference
//   - Multi-trial consistency validation
//   - Comprehensive failure handling & diagnostics
//
// Workflow:
//   1. Call capture_baseline → accumulates N frames, fits ear plane
//   2. Robot places earphone
//   3. Call measure_earphone → robust angle & depth estimation
// ---------------------------------------------------------------------------

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include "vision_server/srv/capture_baseline.hpp"
#include "vision_server/srv/measure_earphone.hpp"

#include <mutex>
#include <cmath>
#include <deque>
#include <random>
#include <numeric>
#include <functional>
#include <map>
#include <filesystem>
#include <fstream>
#include <Eigen/Dense>

// --- ArUco support (optional, compile-time detected) ---
#if __has_include(<opencv2/aruco.hpp>)
  #include <opencv2/aruco.hpp>
  #define HAS_ARUCO 1
#elif __has_include(<opencv2/objdetect/aruco_detector.hpp>)
  #include <opencv2/objdetect/aruco_detector.hpp>
  #define HAS_ARUCO 2
#else
  #define HAS_ARUCO 0
#endif

namespace fs = std::filesystem;

// ============================================================================
// Helper Structures
// ============================================================================

struct Plane {
    Eigen::Vector3d normal{0, 0, -1};
    double d = 0.4;  // ax+by+cz+d = 0

    double signed_distance(const Eigen::Vector3d& p) const {
        return normal.dot(p) + d;
    }
};

struct PCAResult {
    Eigen::Vector3d axis{0, 0, 1};
    Eigen::Vector3d centroid{0, 0, 0};
    Eigen::Vector3d eigenvalues{1, 1, 1};
    int n_points = 0;
    bool valid = false;
};

struct ArUcoResult {
    bool detected = false;
    Eigen::Vector3d ear_normal{0, 0, 1};  // ear canal direction in camera frame
    Eigen::Vector3d marker_tvec{0, 0, 0};
    double angle_correction = 0;  // angle to add for ear-frame reference
};

struct DetectionChannelStats {
    int diff_pixels = 0;
    int plane_pixels = 0;
    int hole_pixels = 0;   // baseline valid but current==0 (IR absorbed by earphone)
    int rgb_pixels = 0;
    int zero_depth_in_roi = 0;
    double avg_noise_sigma = 0;
    std::string primary_channel = "none";
};

struct SingleMeasurement {
    bool valid = false;
    double angle_deg = 0;
    double depth_mm = 0;
    double confidence = 0;
    Eigen::Vector3d axis{0, 0, 1};
    Eigen::Vector3d centroid{0, 0, 0};
    int n_points = 0;
    double area = 0;
    cv::Mat earphone_mask;
    std::vector<cv::Point> contour;
    DetectionChannelStats stats;
    std::string detail;
    // Debug masks for diagnostics
    cv::Mat debug_mask_diff;
    cv::Mat debug_mask_diff_low;  // low-threshold diff (hysteresis candidates)
    cv::Mat debug_mask_holes;
    cv::Mat debug_mask_plane;
    cv::Mat debug_candidates;
    cv::Mat debug_combined;
};

// ============================================================================
// EarphoneInspectorNode
// ============================================================================

class EarphoneInspectorNode : public rclcpp::Node {
public:
    EarphoneInspectorNode() : Node("earphone_inspector_node") {
        // --- Parameters ---
        declare_parameter("camera_ns",            "/camera");
        declare_parameter("avg_frames",            8);        // frames for temporal median
        declare_parameter("min_diff_mm",           2.0);      // min depth diff (adaptive floor)
        declare_parameter("max_diff_mm",           50.0);     // max depth diff
        declare_parameter("min_area_pixels",       30);       // min earphone cluster size
        declare_parameter("roi_width",             250);      // ROI pixels
        declare_parameter("roi_height",            250);
        declare_parameter("plane_inlier_mm",       3.0);      // RANSAC plane inlier threshold
        declare_parameter("plane_ransac_iters",    50);
        declare_parameter("plane_fg_min_mm",       2.0);      // foreground distance from plane
        declare_parameter("plane_fg_max_mm",       50.0);
        declare_parameter("pca_trim_pct",          0.10);     // trim 10% outliers before PCA
        declare_parameter("noise_sigma_scale",     2.5);      // adaptive threshold = sigma * scale
        declare_parameter("enable_aruco",          true);
        declare_parameter("aruco_marker_size_m",   0.03);
        declare_parameter("enable_multi_trial",    true);
        declare_parameter("min_confidence",        0.35);     // below this, flag unreliable
        declare_parameter("roi_margin",            15);       // inner margin to exclude ROI edge artifacts
        declare_parameter("save_dir", std::string(
            std::getenv("HOME") ? std::getenv("HOME") : ".") + "/.ros/earphone_inspection");

        camera_ns_           = get_parameter("camera_ns").as_string();
        avg_frames_          = get_parameter("avg_frames").as_int();
        min_diff_mm_         = get_parameter("min_diff_mm").as_double();
        max_diff_mm_         = get_parameter("max_diff_mm").as_double();
        min_area_            = get_parameter("min_area_pixels").as_int();
        roi_w_               = get_parameter("roi_width").as_int();
        roi_h_               = get_parameter("roi_height").as_int();
        plane_inlier_mm_     = get_parameter("plane_inlier_mm").as_double();
        plane_ransac_iters_  = get_parameter("plane_ransac_iters").as_int();
        plane_fg_min_mm_     = get_parameter("plane_fg_min_mm").as_double();
        plane_fg_max_mm_     = get_parameter("plane_fg_max_mm").as_double();
        pca_trim_pct_        = get_parameter("pca_trim_pct").as_double();
        noise_sigma_scale_   = get_parameter("noise_sigma_scale").as_double();
        enable_aruco_        = get_parameter("enable_aruco").as_bool();
        aruco_marker_size_   = get_parameter("aruco_marker_size_m").as_double();
        enable_multi_trial_  = get_parameter("enable_multi_trial").as_bool();
        min_confidence_      = get_parameter("min_confidence").as_double();
        roi_margin_          = get_parameter("roi_margin").as_int();
        save_dir_            = get_parameter("save_dir").as_string();

        // --- Subscribe to camera ---
        auto depth_topic = camera_ns_ + "/depth/image_raw";
        auto color_topic = camera_ns_ + "/color/image_raw";
        auto info_topic  = camera_ns_ + "/depth/camera_info";

        sub_depth_ = create_subscription<sensor_msgs::msg::Image>(
            depth_topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(mtx_);
                try {
                    auto cv_ptr = cv_bridge::toCvShare(msg,
                        sensor_msgs::image_encodings::TYPE_16UC1);
                    depth_buffer_.push_back(cv_ptr->image.clone());
                    if (depth_buffer_.size() > kBufferSize) {
                        depth_buffer_.pop_front();
                    }
                } catch (...) {}
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
            "EarphoneInspectorNode started | camera=%s | ROI=%dx%d | "
            "diff=[%.1f,%.1f]mm | avg_frames=%d | aruco=%s",
            camera_ns_.c_str(), roi_w_, roi_h_,
            min_diff_mm_, max_diff_mm_, avg_frames_,
            enable_aruco_ ? "on" : "off");
    }

private:
    static constexpr size_t kBufferSize = 24;  // keep up to 24 frames (~800ms at 30fps)

    // ========================================================================
    // Temporal Median Computation
    // ========================================================================

    /// Compute per-pixel median from a sequence of 16UC1 depth frames.
    /// Returns CV_32FC1 (units: mm). Zero = invalid (>50% of frames had zero).
    cv::Mat compute_median_depth(const std::deque<cv::Mat>& buffer, int max_frames) {
        int n = std::min(static_cast<int>(buffer.size()), max_frames);
        if (n == 0) return cv::Mat();

        // Use the most recent frames
        int start_idx = static_cast<int>(buffer.size()) - n;

        int rows = buffer[0].rows, cols = buffer[0].cols;
        cv::Mat result(rows, cols, CV_32FC1, cv::Scalar(0));

        std::vector<uint16_t> vals;
        vals.reserve(n);

        for (int y = 0; y < rows; ++y) {
            float* out_row = result.ptr<float>(y);
            for (int x = 0; x < cols; ++x) {
                vals.clear();
                for (int i = start_idx; i < static_cast<int>(buffer.size()); ++i) {
                    uint16_t v = buffer[i].at<uint16_t>(y, x);
                    if (v > 0) vals.push_back(v);
                }
                // Require >50% valid readings
                if (static_cast<int>(vals.size()) > n / 2) {
                    size_t mid = vals.size() / 2;
                    std::nth_element(vals.begin(), vals.begin() + mid, vals.end());
                    out_row[x] = static_cast<float>(vals[mid]);
                }
                // else: stays 0 (invalid)
            }
        }
        return result;
    }

    /// Compute per-pixel noise std-dev from buffer relative to median.
    /// Returns CV_32FC1 (units: mm).
    cv::Mat compute_noise_map(const std::deque<cv::Mat>& buffer,
                              const cv::Mat& median_f32, int max_frames) {
        int n = std::min(static_cast<int>(buffer.size()), max_frames);
        int start_idx = static_cast<int>(buffer.size()) - n;
        int rows = median_f32.rows, cols = median_f32.cols;
        cv::Mat noise(rows, cols, CV_32FC1, cv::Scalar(0));

        for (int y = 0; y < rows; ++y) {
            const float* med_row = median_f32.ptr<float>(y);
            float* noise_row = noise.ptr<float>(y);
            for (int x = 0; x < cols; ++x) {
                float med = med_row[x];
                if (med < 1.0f) continue;

                double sum_sq = 0;
                int count = 0;
                for (int i = start_idx; i < static_cast<int>(buffer.size()); ++i) {
                    uint16_t v = buffer[i].at<uint16_t>(y, x);
                    if (v > 0) {
                        double d = static_cast<double>(v) - static_cast<double>(med);
                        sum_sq += d * d;
                        count++;
                    }
                }
                if (count > 1) {
                    noise_row[x] = static_cast<float>(std::sqrt(sum_sq / count));
                }
            }
        }
        return noise;
    }

    // ========================================================================
    // Adaptive ROI — find ear surface centroid in baseline
    // ========================================================================

    cv::Rect find_ear_roi(const cv::Mat& baseline_f32) {
        int img_w = baseline_f32.cols, img_h = baseline_f32.rows;

        // Search in center 60% of image
        int sx0 = img_w * 2 / 10, sy0 = img_h * 2 / 10;
        int sx1 = img_w * 8 / 10, sy1 = img_h * 8 / 10;

        // Collect valid depth points (stride=2 for speed)
        std::vector<std::pair<float, cv::Point>> pts;
        for (int y = sy0; y < sy1; y += 2) {
            const float* row = baseline_f32.ptr<float>(y);
            for (int x = sx0; x < sx1; x += 2) {
                float d = row[x];
                if (d > 100.0f && d < 800.0f) {
                    pts.push_back({d, cv::Point(x, y)});
                }
            }
        }

        if (pts.empty()) {
            RCLCPP_WARN(get_logger(), "No valid depth in search area, using image center ROI");
            int x0 = std::max(0, img_w / 2 - roi_w_ / 2);
            int y0 = std::max(0, img_h / 2 - roi_h_ / 2);
            return cv::Rect(x0, y0,
                std::min(roi_w_, img_w - x0), std::min(roi_h_, img_h - y0));
        }

        // Sort by depth — closest points are the ear surface
        std::sort(pts.begin(), pts.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });

        // Take closest 30% as "ear surface"
        int n_ear = std::max(10, static_cast<int>(pts.size() * 0.3));
        double sum_u = 0, sum_v = 0;
        for (int i = 0; i < n_ear; ++i) {
            sum_u += pts[i].second.x;
            sum_v += pts[i].second.y;
        }
        int ear_cu = static_cast<int>(sum_u / n_ear);
        int ear_cv = static_cast<int>(sum_v / n_ear);

        int x0 = std::max(0, ear_cu - roi_w_ / 2);
        int y0 = std::max(0, ear_cv - roi_h_ / 2);
        int x1 = std::min(img_w, x0 + roi_w_);
        int y1 = std::min(img_h, y0 + roi_h_);

        RCLCPP_INFO(get_logger(), "Ear ROI: center=(%d,%d) rect=[%d,%d,%d,%d]",
            ear_cu, ear_cv, x0, y0, x1 - x0, y1 - y0);

        return cv::Rect(x0, y0, x1 - x0, y1 - y0);
    }

    // ========================================================================
    // RANSAC Plane Fitting — fit plane to ear surface
    // ========================================================================

    Plane fit_ear_plane(const cv::Mat& baseline_f32, const cv::Rect& roi,
                        double fx, double fy, double cx, double cy) {
        // Collect 3D points from baseline within ROI (stride=3 for speed)
        std::vector<Eigen::Vector3d> pts;
        for (int y = roi.y; y < roi.y + roi.height; y += 3) {
            const float* row = baseline_f32.ptr<float>(y);
            for (int x = roi.x; x < roi.x + roi.width; x += 3) {
                float d = row[x];
                if (d > 100.0f && d < 800.0f) {
                    double Z = d / 1000.0;
                    double X = (x - cx) * Z / fx;
                    double Y = (y - cy) * Z / fy;
                    pts.emplace_back(X, Y, Z);
                }
            }
        }

        Plane plane;
        if (pts.size() < 20) {
            RCLCPP_WARN(get_logger(),
                "Plane fit: only %zu points, using default plane", pts.size());
            plane.normal = Eigen::Vector3d(0, 0, -1);
            plane.d = 0.4;  // default 400mm
            return plane;
        }

        // RANSAC
        std::mt19937 rng(42);
        int best_inliers = 0;
        double inlier_thresh = plane_inlier_mm_ / 1000.0;  // convert to meters

        for (int iter = 0; iter < plane_ransac_iters_; ++iter) {
            size_t i0 = rng() % pts.size();
            size_t i1 = rng() % pts.size();
            size_t i2 = rng() % pts.size();
            if (i0 == i1 || i1 == i2 || i0 == i2) continue;

            Eigen::Vector3d v1 = pts[i1] - pts[i0];
            Eigen::Vector3d v2 = pts[i2] - pts[i0];
            Eigen::Vector3d n = v1.cross(v2);
            if (n.norm() < 1e-9) continue;
            n.normalize();

            double dd = -n.dot(pts[i0]);
            int inliers = 0;
            for (const auto& p : pts) {
                if (std::abs(n.dot(p) + dd) < inlier_thresh) inliers++;
            }
            if (inliers > best_inliers) {
                best_inliers = inliers;
                plane.normal = n;
                plane.d = dd;
            }
        }

        // Refine: refit using all inliers via SVD
        {
            std::vector<Eigen::Vector3d> inlier_pts;
            for (const auto& p : pts) {
                if (std::abs(plane.normal.dot(p) + plane.d) < inlier_thresh) {
                    inlier_pts.push_back(p);
                }
            }
            if (inlier_pts.size() >= 3) {
                Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
                for (const auto& p : inlier_pts) centroid += p;
                centroid /= static_cast<double>(inlier_pts.size());

                Eigen::MatrixXd A(inlier_pts.size(), 3);
                for (size_t i = 0; i < inlier_pts.size(); ++i) {
                    A.row(i) = (inlier_pts[i] - centroid).transpose();
                }
                Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
                plane.normal = svd.matrixV().col(2).normalized();
                plane.d = -plane.normal.dot(centroid);
            }
        }

        // Ensure normal points away from camera (toward ear → negative Z direction)
        // Ear is further from camera, earphone is closer
        // Convention: signed_distance < 0 means point is in front of plane (closer to camera)
        if (plane.normal.z() > 0) {
            plane.normal = -plane.normal;
            plane.d = -plane.d;
        }

        RCLCPP_INFO(get_logger(),
            "Ear plane: normal=(%.3f, %.3f, %.3f) d=%.4f inliers=%d/%zu",
            plane.normal.x(), plane.normal.y(), plane.normal.z(),
            plane.d, best_inliers, pts.size());

        return plane;
    }

    // ========================================================================
    // CaptureBaseline Service
    // ========================================================================

    void handle_baseline(
        const std::shared_ptr<vision_server::srv::CaptureBaseline::Request> req,
        std::shared_ptr<vision_server::srv::CaptureBaseline::Response> resp)
    {
        // --- Load from file if load_path is provided ---
        if (!req->load_path.empty()) {
            load_baseline_from_dir(req->load_path, resp);
            return;
        }

        // --- Live capture from camera ---
        std::deque<cv::Mat> buffer_copy;
        cv::Mat color;
        double fx, fy, cx, cy;
        bool has_info;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            buffer_copy = depth_buffer_;  // copy buffer
            if (last_color_) {
                try {
                    auto cv_c = cv_bridge::toCvShare(last_color_, "bgr8");
                    color = cv_c->image.clone();
                } catch (...) {}
            }
            fx = fx_; fy = fy_; cx = cx_; cy = cy_;
            has_info = has_info_;
        }

        int n_frames = static_cast<int>(buffer_copy.size());
        if (n_frames < 3) {
            resp->success = false;
            resp->message = "Insufficient depth frames (need >= 3, have "
                            + std::to_string(n_frames) + "). Wait for camera to stabilize.";
            return;
        }
        if (!has_info) {
            resp->success = false;
            resp->message = "No camera intrinsics available";
            return;
        }

        int n_use = std::min(n_frames, avg_frames_);

        // Compute temporal median
        baseline_avg_ = compute_median_depth(buffer_copy, n_use);
        if (baseline_avg_.empty()) {
            resp->success = false;
            resp->message = "Failed to compute baseline median";
            return;
        }

        // Compute noise map
        noise_map_ = compute_noise_map(buffer_copy, baseline_avg_, n_use);

        // Compute average noise
        double noise_sum = 0;
        int noise_count = 0;
        for (int y = 0; y < noise_map_.rows; ++y) {
            const float* row = noise_map_.ptr<float>(y);
            for (int x = 0; x < noise_map_.cols; ++x) {
                if (row[x] > 0) {
                    noise_sum += row[x];
                    noise_count++;
                }
            }
        }
        double avg_noise = noise_count > 0 ? noise_sum / noise_count : 3.0;

        // Find adaptive ROI
        ear_roi_ = find_ear_roi(baseline_avg_);

        // Fit ear surface plane
        ear_plane_ = fit_ear_plane(baseline_avg_, ear_roi_, fx, fy, cx, cy);

        // Store baseline color
        baseline_color_ = color;
        has_baseline_ = true;

        // --- Save baseline data for future reuse ---
        std::string saved_path = save_baseline_data(avg_noise);
        resp->saved_baseline_path = saved_path;

        char buf[512];
        std::snprintf(buf, sizeof(buf),
            "Baseline captured: %dx%d | %d frames averaged | "
            "ROI=(%d,%d,%dx%d) | avg_noise=%.1fmm | saved=%s",
            baseline_avg_.cols, baseline_avg_.rows, n_use,
            ear_roi_.x, ear_roi_.y, ear_roi_.width, ear_roi_.height,
            avg_noise, saved_path.c_str());
        resp->success = true;
        resp->message = buf;
        RCLCPP_INFO(get_logger(), "%s", buf);
    }

    /// Save baseline data (depth median, noise map, plane, ROI, color) to disk
    std::string save_baseline_data(double avg_noise) {
        // Use a fixed "latest" directory + timestamped directory
        fs::path base_dir = fs::path(save_dir_) / "_baselines";
        auto stamp_str = std::to_string(this->now().nanoseconds());
        fs::path dir = base_dir / stamp_str;
        fs::path latest_link = base_dir / "latest";

        try {
            fs::create_directories(dir);

            // Save depth median as 32-bit float EXR (lossless)
            cv::imwrite((dir / "baseline_avg.exr").string(), baseline_avg_);

            // Save noise map
            cv::imwrite((dir / "noise_map.exr").string(), noise_map_);

            // Save color
            if (!baseline_color_.empty()) {
                cv::imwrite((dir / "baseline_color.png").string(), baseline_color_);
            }

            // Save metadata (plane, ROI, intrinsics) as YAML
            cv::FileStorage meta((dir / "meta.yaml").string(),
                                  cv::FileStorage::WRITE);
            meta << "roi_x" << ear_roi_.x;
            meta << "roi_y" << ear_roi_.y;
            meta << "roi_w" << ear_roi_.width;
            meta << "roi_h" << ear_roi_.height;
            meta << "plane_nx" << ear_plane_.normal.x();
            meta << "plane_ny" << ear_plane_.normal.y();
            meta << "plane_nz" << ear_plane_.normal.z();
            meta << "plane_d" << ear_plane_.d;
            meta << "avg_noise" << avg_noise;
            meta << "fx" << fx_;
            meta << "fy" << fy_;
            meta << "cx" << cx_;
            meta << "cy" << cy_;
            meta.release();

            // Update "latest" symlink
            fs::remove(latest_link);
            fs::create_directory_symlink(dir, latest_link);

            RCLCPP_INFO(get_logger(), "Baseline saved to %s", dir.string().c_str());
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Failed to save baseline: %s", e.what());
            return "";
        }
        return dir.string();
    }

    /// Load baseline data from a previously saved directory
    void load_baseline_from_dir(
        const std::string& dir_path,
        std::shared_ptr<vision_server::srv::CaptureBaseline::Response> resp)
    {
        fs::path dir(dir_path);

        // Resolve "latest" symlink
        if (dir.filename() == "latest" || dir_path == "latest") {
            fs::path latest = fs::path(save_dir_) / "_baselines" / "latest";
            if (fs::exists(latest)) {
                dir = fs::canonical(latest);
            } else {
                resp->success = false;
                resp->message = "No saved baseline found. Capture one first.";
                return;
            }
        }

        if (!fs::exists(dir)) {
            resp->success = false;
            resp->message = "Baseline directory does not exist: " + dir.string();
            return;
        }

        try {
            // Load depth median
            fs::path avg_path = dir / "baseline_avg.exr";
            if (!fs::exists(avg_path)) {
                resp->success = false;
                resp->message = "Missing baseline_avg.exr in " + dir.string();
                return;
            }
            baseline_avg_ = cv::imread(avg_path.string(),
                                        cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
            if (baseline_avg_.empty()) {
                resp->success = false;
                resp->message = "Failed to read baseline_avg.exr";
                return;
            }
            // Ensure CV_32FC1
            if (baseline_avg_.type() != CV_32FC1) {
                baseline_avg_.convertTo(baseline_avg_, CV_32FC1);
            }

            // Load noise map
            fs::path noise_path = dir / "noise_map.exr";
            if (fs::exists(noise_path)) {
                noise_map_ = cv::imread(noise_path.string(),
                                         cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
                if (noise_map_.type() != CV_32FC1) {
                    noise_map_.convertTo(noise_map_, CV_32FC1);
                }
            } else {
                // Create default noise map (uniform 3mm)
                noise_map_ = cv::Mat(baseline_avg_.size(), CV_32FC1, cv::Scalar(3.0f));
            }

            // Load color
            fs::path color_path = dir / "baseline_color.png";
            if (fs::exists(color_path)) {
                baseline_color_ = cv::imread(color_path.string(), cv::IMREAD_COLOR);
            }

            // Load metadata
            fs::path meta_path = dir / "meta.yaml";
            if (fs::exists(meta_path)) {
                cv::FileStorage meta(meta_path.string(), cv::FileStorage::READ);
                int rx, ry, rw, rh;
                meta["roi_x"] >> rx;
                meta["roi_y"] >> ry;
                meta["roi_w"] >> rw;
                meta["roi_h"] >> rh;
                ear_roi_ = cv::Rect(rx, ry, rw, rh);

                double nx, ny, nz, pd;
                meta["plane_nx"] >> nx;
                meta["plane_ny"] >> ny;
                meta["plane_nz"] >> nz;
                meta["plane_d"] >> pd;
                ear_plane_.normal = Eigen::Vector3d(nx, ny, nz);
                ear_plane_.d = pd;

                // Load camera intrinsics if available (for offline testing)
                double lfx = 0, lfy = 0, lcx = 0, lcy = 0;
                meta["fx"] >> lfx;
                meta["fy"] >> lfy;
                meta["cx"] >> lcx;
                meta["cy"] >> lcy;
                if (lfx > 0 && lfy > 0) {
                    std::lock_guard<std::mutex> lk(mtx_);
                    if (!has_info_) {
                        fx_ = lfx; fy_ = lfy; cx_ = lcx; cy_ = lcy;
                        has_info_ = true;
                        RCLCPP_INFO(get_logger(),
                            "Loaded camera intrinsics from baseline: "
                            "fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                            fx_, fy_, cx_, cy_);
                    }
                }

                double avg_noise = 3.0;
                meta["avg_noise"] >> avg_noise;
                meta.release();

                has_baseline_ = true;
                resp->saved_baseline_path = dir.string();

                char buf[512];
                std::snprintf(buf, sizeof(buf),
                    "Baseline loaded: %dx%d | ROI=(%d,%d,%dx%d) | "
                    "avg_noise=%.1fmm | from=%s",
                    baseline_avg_.cols, baseline_avg_.rows,
                    ear_roi_.x, ear_roi_.y, ear_roi_.width, ear_roi_.height,
                    avg_noise, dir.string().c_str());
                resp->success = true;
                resp->message = buf;
                RCLCPP_INFO(get_logger(), "%s", buf);
            } else {
                // No metadata — recompute ROI and plane
                std::lock_guard<std::mutex> lk(mtx_);
                ear_roi_ = find_ear_roi(baseline_avg_);
                ear_plane_ = fit_ear_plane(baseline_avg_, ear_roi_,
                                            fx_, fy_, cx_, cy_);
                has_baseline_ = true;
                resp->saved_baseline_path = dir.string();
                resp->success = true;
                resp->message = "Baseline loaded (recomputed ROI/plane): "
                                + dir.string();
            }
        } catch (const std::exception& e) {
            resp->success = false;
            resp->message = std::string("Failed to load baseline: ") + e.what();
        }
    }

    // ========================================================================
    // Detection Channels
    // ========================================================================

    /// Channel A: Noise-adaptive depth differencing
    cv::Mat detect_by_depth_diff(const cv::Mat& current_avg, const cv::Rect& roi,
                                  DetectionChannelStats& stats) {
        int img_h = current_avg.rows, img_w = current_avg.cols;
        cv::Mat mask = cv::Mat::zeros(img_h, img_w, CV_8UC1);

        for (int y = roi.y; y < roi.y + roi.height; ++y) {
            const float* row_base = baseline_avg_.ptr<float>(y);
            const float* row_cur  = current_avg.ptr<float>(y);
            const float* row_noise = noise_map_.ptr<float>(y);
            uint8_t* row_mask = mask.ptr<uint8_t>(y);

            for (int x = roi.x; x < roi.x + roi.width; ++x) {
                float db = row_base[x];
                float dc = row_cur[x];

                if (db < 1.0f || dc < 1.0f) {
                    if (dc < 1.0f) stats.zero_depth_in_roi++;
                    continue;
                }

                double diff = static_cast<double>(db) - static_cast<double>(dc);

                // Adaptive threshold: max(min_diff, sigma * scale)
                float local_sigma = row_noise[x];
                float adaptive_min = std::max(
                    static_cast<float>(min_diff_mm_),
                    local_sigma * static_cast<float>(noise_sigma_scale_));

                if (diff >= adaptive_min && diff <= max_diff_mm_) {
                    row_mask[x] = 255;
                    stats.diff_pixels++;
                }
            }
        }
        return mask;
    }

    /// Channel B: Plane distance foreground extraction
    /// Uses inner margin to exclude ROI edge artifacts (depth discontinuities)
    cv::Mat detect_by_plane_distance(const cv::Mat& current_avg, const cv::Rect& roi,
                                      double fx, double fy, double cx, double cy,
                                      DetectionChannelStats& stats) {
        int img_h = current_avg.rows, img_w = current_avg.cols;
        cv::Mat mask = cv::Mat::zeros(img_h, img_w, CV_8UC1);

        double fg_min_m = plane_fg_min_mm_ / 1000.0;
        double fg_max_m = plane_fg_max_mm_ / 1000.0;

        // Apply inner margin to avoid edge artifacts
        int m = roi_margin_;
        int ry0 = roi.y + m, ry1 = roi.y + roi.height - m;
        int rx0 = roi.x + m, rx1 = roi.x + roi.width - m;

        for (int y = ry0; y < ry1; ++y) {
            const float* row_cur = current_avg.ptr<float>(y);
            uint8_t* row_mask = mask.ptr<uint8_t>(y);

            for (int x = rx0; x < rx1; ++x) {
                float dc = row_cur[x];
                if (dc < 100.0f || dc > 800.0f) continue;

                double Z = dc / 1000.0;
                double X = (x - cx) * Z / fx;
                double Y = (y - cy) * Z / fy;
                Eigen::Vector3d pt(X, Y, Z);

                // Signed distance: negative = in front of plane (closer to camera)
                double dist = ear_plane_.signed_distance(pt);
                if (dist < -fg_min_m && dist > -fg_max_m) {
                    row_mask[x] = 255;
                    stats.plane_pixels++;
                }
            }
        }
        return mask;
    }

    /// Channel D: Depth hole detection (baseline valid → current zero)
    /// Detects where earphone absorbed IR light, creating new zero-depth regions.
    /// This is the most reliable signal for dark/black earphones.
    cv::Mat detect_depth_holes(const cv::Mat& current_avg, const cv::Rect& roi,
                                DetectionChannelStats& stats) {
        int img_h = current_avg.rows, img_w = current_avg.cols;
        cv::Mat mask = cv::Mat::zeros(img_h, img_w, CV_8UC1);

        int m = roi_margin_;
        int ry0 = roi.y + m, ry1 = roi.y + roi.height - m;
        int rx0 = roi.x + m, rx1 = roi.x + roi.width - m;

        for (int y = ry0; y < ry1; ++y) {
            const float* row_base = baseline_avg_.ptr<float>(y);
            const float* row_cur  = current_avg.ptr<float>(y);
            uint8_t* row_mask = mask.ptr<uint8_t>(y);

            for (int x = rx0; x < rx1; ++x) {
                float db = row_base[x];
                float dc = row_cur[x];

                // Baseline had valid depth, but current is zero → something is blocking IR
                if (db > 100.0f && dc < 1.0f) {
                    row_mask[x] = 255;
                    stats.hole_pixels++;
                }
            }
        }

        // Morphological closing to connect nearby depth holes
        // Use small kernel only — large kernels connect earphone holes with
        // ear canal noise holes. The merge step will handle spatial association.
        if (stats.hole_pixels > 5) {
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        }
        return mask;
    }

    /// Channel C (fallback): RGB difference detection
    cv::Mat detect_by_rgb_diff(const cv::Mat& cur_color, const cv::Rect& roi,
                                DetectionChannelStats& stats) {
        if (cur_color.empty() || baseline_color_.empty()) return cv::Mat();
        if (cur_color.size() != baseline_color_.size()) return cv::Mat();

        cv::Mat gray_base, gray_cur, diff_gray;
        cv::cvtColor(baseline_color_, gray_base, cv::COLOR_BGR2GRAY);
        cv::cvtColor(cur_color, gray_cur, cv::COLOR_BGR2GRAY);

        // Scale ROI if color and depth have different resolutions
        double sx = static_cast<double>(cur_color.cols) / baseline_avg_.cols;
        double sy = static_cast<double>(cur_color.rows) / baseline_avg_.rows;
        cv::Rect color_roi(
            static_cast<int>(roi.x * sx), static_cast<int>(roi.y * sy),
            static_cast<int>(roi.width * sx), static_cast<int>(roi.height * sy));
        color_roi &= cv::Rect(0, 0, cur_color.cols, cur_color.rows);

        cv::absdiff(gray_cur, gray_base, diff_gray);
        cv::GaussianBlur(diff_gray, diff_gray, cv::Size(5, 5), 0);

        cv::Mat mask = cv::Mat::zeros(diff_gray.size(), CV_8UC1);
        cv::threshold(diff_gray(color_roi), mask(color_roi), 25, 255, cv::THRESH_BINARY);

        // Small morphological cleanup
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

        stats.rgb_pixels = cv::countNonZero(mask);

        // If depth and color resolutions differ, resize mask to depth resolution
        if (mask.size() != baseline_avg_.size()) {
            cv::Mat resized;
            cv::resize(mask, resized, baseline_avg_.size(), 0, 0, cv::INTER_NEAREST);
            return resized;
        }
        return mask;
    }

    // ========================================================================
    // Connected Component Clustering
    // ========================================================================

    /// Select the best earphone cluster from a binary mask.
    /// Returns refined mask (merged nearby clusters) and fills contour.
    ///
    /// Key insight: The earphone often appears as two separate blobs (tip absorbs
    /// IR → depth hole, stem has depth diff) with a gap between them. We must:
    ///   1. Use large morphological closing to bridge the gap
    ///   2. Merge nearby connected components via proximity-based union-find
    cv::Mat select_earphone_cluster(const cv::Mat& combined_mask, const cv::Rect& roi,
                                     double& out_area, std::vector<cv::Point>& out_contour) {
        // Step 1: Small open to remove isolated noise pixels
        cv::Mat cleaned;
        cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        cv::morphologyEx(combined_mask, cleaned, cv::MORPH_OPEN, open_kernel);

        // Step 2: Large close to bridge gaps between nearby earphone regions.
        // Black earphones produce scattered candidate blobs with 10-50px gaps.
        // Use 41×41 closing (bridges up to ~40px gaps).
        cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(41, 41));
        cv::morphologyEx(cleaned, cleaned, cv::MORPH_CLOSE, close_kernel);

        // Connected components with statistics
        cv::Mat labels, stats, centroids_cc;
        int n_labels = cv::connectedComponentsWithStats(cleaned, labels, stats, centroids_cc, 8);

        if (n_labels <= 1) {
            out_area = 0;
            return cv::Mat::zeros(combined_mask.size(), CV_8UC1);
        }

        // Ear centroid in pixel coordinates (center of ROI)
        double ear_cu = roi.x + roi.width / 2.0;
        double ear_cv = roi.y + roi.height / 2.0;

        // Collect valid clusters (use lower area threshold for merging candidates)
        struct ClusterInfo {
            int label;
            int area;
            cv::Rect bbox;
            double cx, cy;
        };
        std::vector<ClusterInfo> clusters;

        for (int i = 1; i < n_labels; ++i) {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            // Accept very small fragments for merging — even a 3px blob
            // can bridge two larger clusters via proximity union-find
            if (area < 3) continue;
            // Dynamic max area: half the ROI area (prevents false rejection
            // when 41×41 close merges a legitimate earphone cluster into a
            // large blob). For ROI=250×250, max=31250.
            int max_cluster_area = std::max(15000, roi.width * roi.height / 2);
            if (area > max_cluster_area) continue;

            ClusterInfo ci;
            ci.label = i;
            ci.area = area;
            ci.bbox = cv::Rect(
                stats.at<int>(i, cv::CC_STAT_LEFT),
                stats.at<int>(i, cv::CC_STAT_TOP),
                stats.at<int>(i, cv::CC_STAT_WIDTH),
                stats.at<int>(i, cv::CC_STAT_HEIGHT));
            ci.cx = centroids_cc.at<double>(i, 0);
            ci.cy = centroids_cc.at<double>(i, 1);
            clusters.push_back(ci);
        }

        if (clusters.empty()) {
            out_area = 0;
            return cv::Mat::zeros(combined_mask.size(), CV_8UC1);
        }

        // --- Proximity-based cluster merging via Union-Find ---
        // Even after the large closing, some gaps may remain. Merge clusters
        // whose bounding boxes are within merge_dist pixels of each other.
        const int merge_dist = 60;

        // Simple union-find
        std::vector<int> parent(clusters.size());
        std::iota(parent.begin(), parent.end(), 0);

        std::function<int(int)> uf_find = [&](int x) -> int {
            return parent[x] == x ? x : parent[x] = uf_find(parent[x]);
        };
        auto uf_unite = [&](int a, int b) {
            parent[uf_find(a)] = uf_find(b);
        };

        for (size_t i = 0; i < clusters.size(); ++i) {
            for (size_t j = i + 1; j < clusters.size(); ++j) {
                const auto& a = clusters[i].bbox;
                const auto& b = clusters[j].bbox;
                // Chebyshev distance between bounding boxes
                int dx = std::max(0, std::max(a.x, b.x) -
                                      std::min(a.x + a.width, b.x + b.width));
                int dy = std::max(0, std::max(a.y, b.y) -
                                      std::min(a.y + a.height, b.y + b.height));
                int dist = static_cast<int>(std::sqrt(dx * dx + dy * dy));

                if (dist <= merge_dist) {
                    uf_unite(static_cast<int>(i), static_cast<int>(j));
                }
            }
        }

        // Group clusters by root
        std::map<int, std::vector<size_t>> groups;
        for (size_t i = 0; i < clusters.size(); ++i) {
            groups[uf_find(static_cast<int>(i))].push_back(i);
        }

        // Score each merged group: prefer large total area + close to ear center
        int best_root = -1;
        double best_score = -1;

        for (const auto& [root, members] : groups) {
            int total_area = 0;
            double sum_cx = 0, sum_cy = 0;
            int min_bx = INT_MAX, min_by = INT_MAX, max_bx = 0, max_by = 0;
            for (size_t idx : members) {
                total_area += clusters[idx].area;
                sum_cx += clusters[idx].cx * clusters[idx].area;
                sum_cy += clusters[idx].cy * clusters[idx].area;
                // Track merged group bounding box
                min_bx = std::min(min_bx, clusters[idx].bbox.x);
                min_by = std::min(min_by, clusters[idx].bbox.y);
                max_bx = std::max(max_bx, clusters[idx].bbox.x + clusters[idx].bbox.width);
                max_by = std::max(max_by, clusters[idx].bbox.y + clusters[idx].bbox.height);
            }
            if (total_area < min_area_) continue;

            double group_cx = sum_cx / total_area;
            double group_cy = sum_cy / total_area;
            double dist = std::sqrt((group_cx - ear_cu) * (group_cx - ear_cu) +
                                    (group_cy - ear_cv) * (group_cy - ear_cv));

            // Shape scoring: earphones are elongated (aspect ratio > 2)
            // Bonus for elongated clusters, slight penalty for round/compact
            int bb_w = std::max(1, max_bx - min_bx);
            int bb_h = std::max(1, max_by - min_by);
            double aspect = static_cast<double>(std::max(bb_w, bb_h)) /
                            static_cast<double>(std::min(bb_w, bb_h));
            double shape_bonus = std::clamp(aspect / 2.5, 0.5, 1.5);

            double score = static_cast<double>(total_area) * shape_bonus / (1.0 + dist * 0.1);
            if (score > best_score) {
                best_score = score;
                best_root = root;
            }
        }

        if (best_root < 0) {
            out_area = 0;
            return cv::Mat::zeros(combined_mask.size(), CV_8UC1);
        }

        // Create mask for best merged group (all member clusters)
        cv::Mat result = cv::Mat::zeros(combined_mask.size(), CV_8UC1);
        int total_area = 0;
        for (size_t idx : groups[best_root]) {
            cv::Mat cluster_mask;
            cv::compare(labels, clusters[idx].label, cluster_mask, cv::CMP_EQ);
            cv::bitwise_or(result, cluster_mask, result);
            total_area += clusters[idx].area;
        }

        // If multiple clusters were merged, close the gap between them
        if (groups[best_root].size() > 1) {
            cv::Mat fill_kernel = cv::getStructuringElement(
                cv::MORPH_ELLIPSE, cv::Size(15, 15));
            cv::morphologyEx(result, result, cv::MORPH_CLOSE, fill_kernel);
            // Recount after filling
            total_area = cv::countNonZero(result);
        }

        out_area = static_cast<double>(total_area);

        // Extract contour for visualization
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(result.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (!contours.empty()) {
            size_t largest = 0;
            for (size_t i = 1; i < contours.size(); ++i) {
                if (cv::contourArea(contours[i]) > cv::contourArea(contours[largest]))
                    largest = i;
            }
            out_contour = contours[largest];
        }

        return result;
    }

    // ========================================================================
    // Robust PCA (Trimmed)
    // ========================================================================

    PCAResult compute_robust_pca(const std::vector<Eigen::Vector3d>& points) {
        PCAResult result;
        result.n_points = static_cast<int>(points.size());

        if (points.size() < 5) return result;

        // Compute initial centroid
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (const auto& p : points) centroid += p;
        centroid /= static_cast<double>(points.size());

        // Compute distances from centroid
        std::vector<std::pair<double, size_t>> distances;
        distances.reserve(points.size());
        for (size_t i = 0; i < points.size(); ++i) {
            distances.push_back({(points[i] - centroid).squaredNorm(), i});
        }
        std::sort(distances.begin(), distances.end());

        // Keep (1 - pca_trim_pct_) closest to centroid
        int n_keep = static_cast<int>(
            distances.size() * (1.0 - pca_trim_pct_));
        n_keep = std::max(n_keep, 5);
        n_keep = std::min(n_keep, static_cast<int>(distances.size()));

        std::vector<Eigen::Vector3d> trimmed;
        trimmed.reserve(n_keep);
        for (int i = 0; i < n_keep; ++i) {
            trimmed.push_back(points[distances[i].second]);
        }

        // Recompute centroid on trimmed set
        centroid = Eigen::Vector3d::Zero();
        for (const auto& p : trimmed) centroid += p;
        centroid /= static_cast<double>(trimmed.size());

        // Covariance matrix
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for (const auto& p : trimmed) {
            Eigen::Vector3d d = p - centroid;
            cov += d * d.transpose();
        }
        cov /= static_cast<double>(trimmed.size());

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);

        result.axis = solver.eigenvectors().col(2).normalized();
        result.centroid = centroid;
        result.eigenvalues = solver.eigenvalues();
        result.n_points = static_cast<int>(trimmed.size());
        result.valid = true;

        // Ensure axis points roughly toward camera (+Z for protrusion)
        if (result.axis.z() < 0) result.axis = -result.axis;

        return result;
    }

    // ========================================================================
    // ArUco Detection (optional)
    // ========================================================================

    ArUcoResult detect_aruco(const cv::Mat& color_image,
                              double fx, double fy, double cx, double cy) {
        ArUcoResult result;
#if HAS_ARUCO == 0
        (void)color_image; (void)fx; (void)fy; (void)cx; (void)cy;
        return result;
#else
        if (color_image.empty()) return result;

        // Camera matrix (assuming rectified images, no distortion)
        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
            fx, 0, cx,
            0, fy, cy,
            0,  0,  1);
        cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);

        try {
#if HAS_ARUCO == 1
            // OpenCV < 4.7 API
            auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
            auto params = cv::aruco::DetectorParameters::create();

            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            cv::aruco::detectMarkers(color_image, dict, corners, ids, params);

            if (ids.empty()) return result;

            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(
                corners, static_cast<float>(aruco_marker_size_),
                camera_matrix, dist_coeffs, rvecs, tvecs);

            // Use first detected marker
            cv::Mat R;
            cv::Rodrigues(rvecs[0], R);

            // Ear normal = marker Z-axis (column 2 of rotation matrix)
            result.ear_normal = Eigen::Vector3d(
                R.at<double>(0, 2), R.at<double>(1, 2), R.at<double>(2, 2));
            result.marker_tvec = Eigen::Vector3d(
                tvecs[0][0], tvecs[0][1], tvecs[0][2]);
            result.detected = true;
#elif HAS_ARUCO == 2
            // OpenCV >= 4.7 API
            auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
            cv::aruco::ArucoDetector detector(dict);

            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            detector.detectMarkers(color_image, corners, ids);

            if (ids.empty()) return result;

            std::vector<cv::Vec3d> rvecs, tvecs;
            // For 4.7+, use solvePnP per marker
            for (size_t i = 0; i < ids.size(); ++i) {
                std::vector<cv::Point3f> obj_pts = {
                    {-static_cast<float>(aruco_marker_size_)/2, static_cast<float>(aruco_marker_size_)/2, 0},
                    { static_cast<float>(aruco_marker_size_)/2, static_cast<float>(aruco_marker_size_)/2, 0},
                    { static_cast<float>(aruco_marker_size_)/2,-static_cast<float>(aruco_marker_size_)/2, 0},
                    {-static_cast<float>(aruco_marker_size_)/2,-static_cast<float>(aruco_marker_size_)/2, 0}
                };
                cv::Vec3d rvec, tvec;
                cv::solvePnP(obj_pts, corners[i], camera_matrix, dist_coeffs, rvec, tvec);
                rvecs.push_back(rvec);
                tvecs.push_back(tvec);
            }

            cv::Mat R;
            cv::Rodrigues(rvecs[0], R);
            result.ear_normal = Eigen::Vector3d(
                R.at<double>(0, 2), R.at<double>(1, 2), R.at<double>(2, 2));
            result.marker_tvec = Eigen::Vector3d(
                tvecs[0][0], tvecs[0][1], tvecs[0][2]);
            result.detected = true;
#endif
            if (result.detected) {
                // Ensure normal points toward camera
                if (result.ear_normal.z() > 0)
                    result.ear_normal = -result.ear_normal;
            }
        } catch (const std::exception& e) {
            RCLCPP_DEBUG(get_logger(), "ArUco detection failed: %s", e.what());
        }
        return result;
#endif
    }

    // ========================================================================
    // Confidence Scoring
    // ========================================================================

    double compute_confidence(const PCAResult& pca, double area,
                               const DetectionChannelStats& stats,
                               bool aruco_detected) {
        if (!pca.valid || pca.n_points < 5) return 0.0;

        // 1. Point density (enough for reliable PCA? target: 150+)
        double point_score = std::min(1.0, static_cast<double>(pca.n_points) / 150.0);

        // 2. Elongation (earphones are elongated, eigenvalue ratio > 2)
        double ev0 = std::max(pca.eigenvalues(0), 1e-12);
        double ev1 = std::max(pca.eigenvalues(1), 1e-12);
        double ev2 = std::max(pca.eigenvalues(2), 1e-12);
        double elongation = ev2 / ev1;
        double shape_score = std::min(1.0, elongation / 3.0);

        // 3. Planarity penalty (earphone should be line-like, not plane-like)
        double planarity = ev1 / ev0;
        double planarity_factor = (planarity > 5.0) ? 0.7 : 1.0;

        // 4. Area coverage
        double area_score = std::min(1.0, area / 300.0);

        // 5. Zero-depth penalty (black material absorbs IR)
        double total_roi = static_cast<double>(
            std::max(1, stats.zero_depth_in_roi + stats.diff_pixels + stats.plane_pixels));
        double zero_ratio = static_cast<double>(stats.zero_depth_in_roi) / total_roi;
        double zero_penalty = (zero_ratio > 0.3) ? 0.6 : 1.0;

        // 6. Detection channel bonus
        double channel_bonus = 1.0;
        if (stats.hole_pixels > min_area_) {
            channel_bonus = 1.15;  // hole detection = very reliable
        }
        if (stats.diff_pixels > min_area_ && stats.plane_pixels > min_area_) {
            channel_bonus = std::max(channel_bonus, 1.1);
        }
        if (stats.primary_channel == "rgb") {
            channel_bonus = 0.6;  // RGB-only is less reliable for 3D
        }

        // 7. ArUco bonus (if detected, angle reference is more reliable)
        double aruco_bonus = aruco_detected ? 1.1 : 1.0;

        double confidence =
            (point_score * 0.25 + shape_score * 0.25 +
             area_score * 0.25 + (1.0 - zero_ratio) * 0.25) *
            planarity_factor * zero_penalty * channel_bonus * aruco_bonus;

        return std::clamp(confidence, 0.0, 1.0);
    }

    // ========================================================================
    // Core Measurement Pipeline (single trial)
    // ========================================================================

    SingleMeasurement run_pipeline(const cv::Mat& current_avg, const cv::Mat& cur_color,
                                    double fx, double fy, double cx, double cy) {
        SingleMeasurement m;

        // Check size match
        if (baseline_avg_.size() != current_avg.size()) {
            m.detail = "Depth frame size mismatch with baseline";
            return m;
        }

        // --- Channel A: Noise-adaptive depth differencing ---
        cv::Mat mask_diff = detect_by_depth_diff(current_avg, ear_roi_, m.stats);

        // --- Channel B: Plane distance foreground (with inner margin) ---
        cv::Mat mask_plane = detect_by_plane_distance(
            current_avg, ear_roi_, fx, fy, cx, cy, m.stats);

        // --- Channel D: Depth hole detection (IR absorption) ---
        cv::Mat mask_holes = detect_depth_holes(current_avg, ear_roi_, m.stats);

        // --- Hysteresis: low-threshold depth diff for candidate expansion ---
        // The high threshold (adaptive_min) captures reliable earphone pixels,
        // but filters out 3-7mm edge pixels that ARE visible in depth_diff.png.
        // Low threshold (sigma × 1.0) captures these edge pixels as CANDIDATES
        // (not seeds), so region growing can reach them from high-threshold seeds.
        cv::Mat mask_diff_low = cv::Mat::zeros(current_avg.rows, current_avg.cols, CV_8UC1);
        int low_diff_count = 0;
        for (int y = ear_roi_.y; y < ear_roi_.y + ear_roi_.height; ++y) {
            const float* row_base = baseline_avg_.ptr<float>(y);
            const float* row_cur  = current_avg.ptr<float>(y);
            const float* row_noise = noise_map_.ptr<float>(y);
            uint8_t* row_mask_low = mask_diff_low.ptr<uint8_t>(y);

            for (int x = ear_roi_.x; x < ear_roi_.x + ear_roi_.width; ++x) {
                float db = row_base[x];
                float dc = row_cur[x];
                if (db < 1.0f || dc < 1.0f) continue;

                double diff = static_cast<double>(db) - static_cast<double>(dc);
                float local_sigma = row_noise[x];
                // Low threshold: max(1.5mm, sigma × 1.5)
                // vs high threshold: max(2.0mm, sigma × 2.5)
                // Floor raised to 1.5mm to avoid flooding in low-noise scenarios
                float adaptive_low = std::max(
                    static_cast<float>(min_diff_mm_) * 0.75f,
                    local_sigma * 1.5f);

                if (diff >= adaptive_low && diff <= max_diff_mm_) {
                    row_mask_low[x] = 255;
                    low_diff_count++;
                }
            }
        }

        RCLCPP_DEBUG(get_logger(),
            "Hysteresis diff: high=%d low=%d (ratio=%.1fx)",
            m.stats.diff_pixels, low_diff_count,
            m.stats.diff_pixels > 0 ? static_cast<double>(low_diff_count) / m.stats.diff_pixels : 0.0);

        // Store per-channel masks for debug
        m.debug_mask_diff = mask_diff.clone();
        m.debug_mask_diff_low = mask_diff_low.clone();
        m.debug_mask_holes = mask_holes.clone();
        m.debug_mask_plane = mask_plane.clone();

        // --- Merge channels ---
        //
        // Strategy depends on signal quality:
        //   - Enough diff seeds (>=50): region-grow from diff into candidates
        //   - Hole-dominated (black earphone absorbs IR, few diff pixels):
        //     use candidates directly with aggressive closing, then cluster
        //   - Sparse diff (3..49): grow with larger kernel
        //   - No signal: use all candidates
        //
        // Black earphones are the hard case: they absorb structured-light IR,
        // producing large zero-depth "hole" regions and very few actual depth-
        // diff pixels. Region growing from sparse diff seeds often fails
        // because the gap between the earphone tip blob and the stem arc
        // in the candidate mask is too wide for the seeds to bridge.

        cv::Mat combined;
        int diff_count = cv::countNonZero(mask_diff);
        int hole_count = m.stats.hole_pixels;

        // Merge hole + diff + plane into base candidate mask.
        // NOTE: mask_diff_low is added ONLY in the region-growing path below,
        // NOT here, because in hole-dominated scenarios (low noise + dense
        // low-threshold pixels) it would flood the entire ROI.
        cv::Mat candidates;
        cv::bitwise_or(mask_diff, mask_holes, candidates);
        cv::bitwise_or(candidates, mask_plane, candidates);

        // Save base candidates for post-clustering rotated-rect recovery
        cv::Mat candidates_for_fill = candidates.clone();

        // Determine if this is a black-earphone detection where depth is unreliable.
        // Black earphones absorb IR → massive zero-depth regions in ROI, very few
        // actual depth-diff pixels.
        //
        // IMPORTANT: Only use blob-connect when diff seeds are TRULY sparse (< 50).
        // Even with massive holes (1500+), if diff >= 50, region-growing from diff
        // seeds is BETTER because it constrains the mask to earphone-adjacent pixels
        // and avoids merging ear-canal holes with earphone holes.
        // A ratio check (holes > diff*5) was tried but caused diff=173 to wrongly
        // enter blob-connect, which merged the entire ear canal into the mask.
        bool hole_dominated = (diff_count < 50 &&
            (hole_count >= min_area_ ||
             m.stats.zero_depth_in_roi > static_cast<int>(ear_roi_.width * ear_roi_.height * 0.05)));

        if (hole_dominated) {
            // --- Hole-dominated: connect nearby large candidate blobs ---
            //
            // For black earphones the candidate pixels (holes + plane) form
            // several distinct blobs with 30-60px gaps.  Morphological closing
            // alone can't bridge these gaps without a huge kernel that also
            // connects noise.
            //
            // Better approach: "connect the dots"
            //   1. Find connected components in the ROI candidates
            //   2. For blobs with area >= blob_min, compute centroids
            //   3. Draw thick lines between centroids within connect_dist
            //   4. Moderate closing to smooth the joined shape
            //   5. Cluster scoring rejects far-away noise

            combined = cv::Mat::zeros(candidates.size(), CV_8UC1);
            cv::Mat roi_cand = candidates(ear_roi_).clone();

            // Find blobs in ROI
            cv::Mat cc_labels, cc_stats, cc_cents;
            int n_cc = cv::connectedComponentsWithStats(
                roi_cand, cc_labels, cc_stats, cc_cents, 8);

            struct CandBlob {
                cv::Point2f center;
                int area;
            };
            std::vector<CandBlob> blobs;
            const int blob_min = 10;  // minimum pixels to be a "significant" blob

            for (int i = 1; i < n_cc; ++i) {
                int area = cc_stats.at<int>(i, cv::CC_STAT_AREA);
                if (area >= blob_min) {
                    blobs.push_back({
                        cv::Point2f(
                            static_cast<float>(cc_cents.at<double>(i, 0)),
                            static_cast<float>(cc_cents.at<double>(i, 1))),
                        area
                    });
                }
            }

            // Draw thick connecting lines between nearby blobs
            const float connect_dist = 80.0f;
            const int line_thickness = 12;
            cv::Mat connected = roi_cand.clone();

            for (size_t i = 0; i < blobs.size(); ++i) {
                for (size_t j = i + 1; j < blobs.size(); ++j) {
                    float dx = blobs[i].center.x - blobs[j].center.x;
                    float dy = blobs[i].center.y - blobs[j].center.y;
                    float dist = std::sqrt(dx * dx + dy * dy);
                    if (dist <= connect_dist) {
                        cv::line(connected,
                            cv::Point(static_cast<int>(blobs[i].center.x),
                                      static_cast<int>(blobs[i].center.y)),
                            cv::Point(static_cast<int>(blobs[j].center.x),
                                      static_cast<int>(blobs[j].center.y)),
                            cv::Scalar(255), line_thickness);
                    }
                }
            }

            // Moderate closing to smooth the connected shape
            cv::Mat smooth_k = cv::getStructuringElement(
                cv::MORPH_ELLIPSE, cv::Size(21, 21));
            cv::morphologyEx(connected, connected, cv::MORPH_CLOSE, smooth_k);

            connected.copyTo(combined(ear_roi_));

            RCLCPP_INFO(get_logger(),
                "Hole-dominated blob connect: %zu blobs found (min_area=%d), "
                "connect_dist=%.0f",
                blobs.size(), blob_min, connect_dist);

        } else if (diff_count >= 3) {
            // --- Good diff signal: region-grow from diff seeds ---
            // Add low-threshold diff to candidates HERE (safe because region
            // growing is self-limiting — only reaches pixels connected to
            // high-threshold seeds). This enriches connectivity between
            // fragments without flooding the entire ROI.
            cv::bitwise_or(candidates, mask_diff_low, candidates);

            // Bridge gap between diff edges and hole centers.
            // Black earphones have a "dead zone" between the diff edge pixels
            // (where depth transitions from ear surface to earphone) and the
            // hole center pixels (where IR is fully absorbed). This dead zone
            // has valid depth with small differences (< threshold), so NO
            // detection channel fires there. Region growing can't cross it.
            //
            // Fix: dilate both masks and take their overlap as bridge pixels.
            // Only adds candidates in the narrow zone BETWEEN diff and holes,
            // not everywhere.
            if (hole_count >= min_area_) {
                cv::Mat diff_exp, hole_exp, bridge;
                cv::Mat bridge_k = cv::getStructuringElement(
                    cv::MORPH_ELLIPSE, cv::Size(25, 25));
                cv::dilate(mask_diff, diff_exp, bridge_k);
                cv::dilate(mask_holes, hole_exp, bridge_k);
                cv::bitwise_and(diff_exp, hole_exp, bridge);
                cv::bitwise_or(candidates, bridge, candidates);

                int bridge_px = cv::countNonZero(bridge);
                RCLCPP_DEBUG(get_logger(),
                    "Diff-hole bridge: %d pixels added to candidates", bridge_px);
            }

            // Close candidate gaps first
            {
                cv::Mat cand_close_k = cv::getStructuringElement(
                    cv::MORPH_ELLIPSE, cv::Size(11, 11));
                cv::morphologyEx(candidates, candidates, cv::MORPH_CLOSE, cand_close_k);
            }

            combined = mask_diff.clone();
            cv::Mat grow_kernel = cv::getStructuringElement(
                cv::MORPH_ELLIPSE, cv::Size(15, 15));

            for (int iter = 0; iter < 20; ++iter) {
                cv::Mat grown;
                cv::dilate(combined, grown, grow_kernel);
                cv::Mat new_region;
                cv::bitwise_and(grown, candidates, new_region);
                int old_count = cv::countNonZero(combined);
                cv::bitwise_or(combined, new_region, combined);
                int new_count = cv::countNonZero(combined);
                if (new_count == old_count) break;
            }
            cv::Mat close_kernel = cv::getStructuringElement(
                cv::MORPH_ELLIPSE, cv::Size(7, 7));
            cv::morphologyEx(combined, combined, cv::MORPH_CLOSE, close_kernel);

        } else if (hole_count >= min_area_) {
            // --- Some holes but not dominant: grow from plane-filtered holes ---
            cv::Mat hole_seeds;
            cv::Mat plane_dilated;
            cv::Mat pl_k = cv::getStructuringElement(
                cv::MORPH_ELLIPSE, cv::Size(15, 15));
            cv::dilate(mask_plane, plane_dilated, pl_k);
            cv::bitwise_and(mask_holes, plane_dilated, hole_seeds);

            if (cv::countNonZero(hole_seeds) >= min_area_) {
                combined = hole_seeds.clone();
                cv::Mat grow_kernel = cv::getStructuringElement(
                    cv::MORPH_ELLIPSE, cv::Size(11, 11));
                for (int iter = 0; iter < 15; ++iter) {
                    cv::Mat grown;
                    cv::dilate(combined, grown, grow_kernel);
                    cv::Mat new_region;
                    cv::bitwise_and(grown, candidates, new_region);
                    int old_count = cv::countNonZero(combined);
                    cv::bitwise_or(combined, new_region, combined);
                    if (cv::countNonZero(combined) == old_count) break;
                }
            } else {
                combined = candidates.clone();
                cv::Mat close_kernel = cv::getStructuringElement(
                    cv::MORPH_ELLIPSE, cv::Size(21, 21));
                cv::morphologyEx(combined, combined, cv::MORPH_CLOSE, close_kernel);
            }
        } else {
            // --- No signal at all: use all candidates ---
            combined = candidates.clone();
        }

        m.debug_candidates = candidates.clone();

        // Compute avg noise for stats
        {
            double ns = 0; int nc = 0;
            for (int y = ear_roi_.y; y < ear_roi_.y + ear_roi_.height; ++y) {
                const float* nr = noise_map_.ptr<float>(y);
                for (int x = ear_roi_.x; x < ear_roi_.x + ear_roi_.width; ++x) {
                    if (nr[x] > 0) { ns += nr[x]; nc++; }
                }
            }
            m.stats.avg_noise_sigma = nc > 0 ? ns / nc : 3.0;
        }

        int combined_pixels = cv::countNonZero(combined);

        // --- Fallback: if A+B produced too few pixels, try Channel C (RGB) ---
        bool used_rgb_fallback = false;
        if (combined_pixels < min_area_) {
            // Try relaxed thresholds first (50% lower min_diff)
            double saved_min = min_diff_mm_;
            min_diff_mm_ *= 0.5;
            DetectionChannelStats relaxed_stats;
            cv::Mat mask_relaxed = detect_by_depth_diff(current_avg, ear_roi_, relaxed_stats);
            min_diff_mm_ = saved_min;

            cv::bitwise_or(combined, mask_relaxed, combined);
            combined_pixels = cv::countNonZero(combined);
            m.stats.diff_pixels += relaxed_stats.diff_pixels;

            if (combined_pixels < min_area_) {
                // RGB fallback
                cv::Mat mask_rgb = detect_by_rgb_diff(cur_color, ear_roi_, m.stats);
                if (!mask_rgb.empty()) {
                    cv::bitwise_or(combined, mask_rgb, combined);
                    combined_pixels = cv::countNonZero(combined);
                    used_rgb_fallback = true;
                }
            }
        }

        // Determine primary detection channel
        if (m.stats.hole_pixels >= min_area_) {
            m.stats.primary_channel = "hole";
            if (m.stats.diff_pixels >= min_area_) m.stats.primary_channel = "hole+diff";
        } else if (m.stats.diff_pixels >= min_area_ && m.stats.plane_pixels >= min_area_) {
            m.stats.primary_channel = "diff+plane";
        } else if (m.stats.diff_pixels >= min_area_) {
            m.stats.primary_channel = "diff";
        } else if (m.stats.plane_pixels >= min_area_) {
            m.stats.primary_channel = "plane";
        } else if (used_rgb_fallback && m.stats.rgb_pixels >= min_area_) {
            m.stats.primary_channel = "rgb";
        } else {
            m.stats.primary_channel = "none";
        }

        // --- Clustering: select best earphone cluster ---
        m.debug_combined = combined.clone();
        m.earphone_mask = select_earphone_cluster(
            combined, ear_roi_, m.area, m.contour);

        // --- Rotated-rect candidate recovery ---
        // After clustering selects the best earphone region, recover candidate
        // pixels inside the cluster's oriented bounding box that were missed
        // by region growing. This fills internal gaps without expanding beyond
        // the earphone's spatial extent.
        if (m.area >= min_area_ && m.contour.size() >= 5) {
            cv::RotatedRect rrect = cv::minAreaRect(m.contour);

            // Expand rect by 10px each side to catch edge pixels
            rrect.size.width += 20.0f;
            rrect.size.height += 20.0f;

            // Create mask from expanded rotated rect
            cv::Mat rrect_mask = cv::Mat::zeros(m.earphone_mask.size(), CV_8UC1);
            cv::Point2f vertices[4];
            rrect.points(vertices);
            std::vector<cv::Point> rrect_pts;
            for (int i = 0; i < 4; ++i) {
                rrect_pts.emplace_back(
                    static_cast<int>(vertices[i].x),
                    static_cast<int>(vertices[i].y));
            }
            cv::fillConvexPoly(rrect_mask, rrect_pts, cv::Scalar(255));

            // Accept candidate pixels within expanded rect
            cv::Mat extra;
            cv::bitwise_and(rrect_mask, candidates_for_fill, extra);

            int old_area = static_cast<int>(m.area);
            cv::bitwise_or(m.earphone_mask, extra, m.earphone_mask);

            // Light cleanup: remove isolated noise pixels added by fill
            cv::Mat clean_k = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
            cv::morphologyEx(m.earphone_mask, m.earphone_mask, cv::MORPH_OPEN, clean_k);

            m.area = cv::countNonZero(m.earphone_mask);

            // Update contour if area increased
            if (m.area > old_area) {
                std::vector<std::vector<cv::Point>> new_contours;
                cv::findContours(m.earphone_mask.clone(), new_contours,
                                  cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                if (!new_contours.empty()) {
                    size_t largest = 0;
                    for (size_t i = 1; i < new_contours.size(); ++i) {
                        if (cv::contourArea(new_contours[i]) > cv::contourArea(new_contours[largest]))
                            largest = i;
                    }
                    m.contour = new_contours[largest];
                }

                RCLCPP_INFO(get_logger(),
                    "Rotated-rect fill: %d → %d pixels (+%d)",
                    old_area, static_cast<int>(m.area),
                    static_cast<int>(m.area) - old_area);
            }
        }

        if (m.area < min_area_) {
            m.detail = "No earphone cluster found: "
                "diff_px=" + std::to_string(m.stats.diff_pixels) +
                " plane_px=" + std::to_string(m.stats.plane_pixels) +
                " hole_px=" + std::to_string(m.stats.hole_pixels) +
                " rgb_px=" + std::to_string(m.stats.rgb_pixels) +
                " zero_px=" + std::to_string(m.stats.zero_depth_in_roi) +
                " combined_px=" + std::to_string(combined_pixels) +
                " hole_dom=" + std::string(hole_dominated ? "Y" : "N") +
                " noise_sigma=" + std::to_string(m.stats.avg_noise_sigma);
            return m;
        }

        // --- Extract 3D points & depth diffs from earphone mask ---
        std::vector<Eigen::Vector3d> points_3d;
        std::vector<double> depth_diffs;
        int n_zero_in_mask = 0;

        for (int y = ear_roi_.y; y < ear_roi_.y + ear_roi_.height; ++y) {
            const float* row_base = baseline_avg_.ptr<float>(y);
            const float* row_cur  = current_avg.ptr<float>(y);
            const uint8_t* row_mask = m.earphone_mask.ptr<uint8_t>(y);

            for (int x = ear_roi_.x; x < ear_roi_.x + ear_roi_.width; ++x) {
                if (row_mask[x] == 0) continue;

                float dc = row_cur[x];
                float db = row_base[x];

                if (dc < 1.0f) {
                    n_zero_in_mask++;
                    // For depth-hole pixels: use baseline depth as approximation
                    // (the earphone is at roughly the ear surface depth)
                    if (db > 100.0f) {
                        double Z = db / 1000.0;
                        double X = (x - cx) * Z / fx;
                        double Y = (y - cy) * Z / fy;
                        points_3d.emplace_back(X, Y, Z);
                        // DO NOT push to depth_diffs — we can't measure actual
                        // protrusion for hole pixels. Only real depth-diff pixels
                        // contribute to the insertion depth estimate.
                    }
                    continue;
                }

                double Z = dc / 1000.0;
                double X = (x - cx) * Z / fx;
                double Y = (y - cy) * Z / fy;
                points_3d.emplace_back(X, Y, Z);

                if (db > 1.0f) {
                    depth_diffs.push_back(static_cast<double>(db) - static_cast<double>(dc));
                }
            }
        }

        // Handle sparse point cloud
        if (static_cast<int>(points_3d.size()) < 5) {
            m.detail = "Too few 3D points: " + std::to_string(points_3d.size()) +
                       " (zero_in_mask=" + std::to_string(n_zero_in_mask) + ")";
            return m;
        }

        bool is_sparse = static_cast<int>(points_3d.size()) < min_area_;

        // --- Determine if depth data is unreliable for 3D PCA ---
        //
        // For black earphones, the vast majority of 3D points come from either:
        //   (a) hole pixels projected at BASELINE depth → all share same Z
        //   (b) plane pixels that are part of the ear surface → also flat
        // In both cases, the 3D point cloud is degenerate (planar), and PCA
        // finds the in-plane spread direction rather than the earphone's axis.
        //
        // Detection: use the hole_dominated flag from channel merging (which
        // considers diff_count, hole_count, and zero_depth_in_roi), OR check
        // if too many mask pixels have zero current depth.
        bool mask_hole_dominated = hole_dominated ||
            (n_zero_in_mask > static_cast<int>(points_3d.size()) / 3);

        // --- Robust 3D PCA (always compute for centroid/stats) ---
        PCAResult pca = compute_robust_pca(points_3d);
        if (!pca.valid) {
            m.detail = "PCA failed";
            return m;
        }

        double ev_ratio = pca.eigenvalues(2) / std::max(pca.eigenvalues(1), 1e-12);
        bool pca_unstable = (ev_ratio < 1.5);

        Eigen::Vector3d effective_axis = pca.axis;
        bool used_2d_pca = false;

        if (mask_hole_dominated || pca_unstable) {
            // =========================================================
            // 2D PCA path — replaces broken 3D PCA for black earphones
            // =========================================================
            //
            // Problem: 3D PCA fails because hole pixels are projected at
            // baseline depth → flat point cloud → PCA finds in-plane spread,
            // not earphone axis. The old fallback (ear plane normal) always
            // reports the same angle regardless of earphone orientation.
            //
            // Solution: 2D PCA on earphone mask pixels gives the earphone's
            // shape direction in the image. Compare this to the ear canal's
            // 2D projection to get the actual insertion tilt angle.

            // --- Step 1: Trimmed 2D PCA on earphone mask ---
            //
            // Collect mask pixels, then keep only the 70% closest to the
            // centroid. This naturally removes:
            //   - Earphone wire/cable (thin, extends far from bud center)
            //   - Earphone stem/handle (wider but still extends away)
            //   - Edge noise pixels
            // Only the compact earphone bud remains for PCA.

            std::vector<cv::Point2f> all_mask_pts;
            for (int y = ear_roi_.y; y < ear_roi_.y + ear_roi_.height; ++y) {
                const uint8_t* row = m.earphone_mask.ptr<uint8_t>(y);
                for (int x = ear_roi_.x; x < ear_roi_.x + ear_roi_.width; ++x) {
                    if (row[x]) all_mask_pts.emplace_back(
                        static_cast<float>(x), static_cast<float>(y));
                }
            }

            // Compute centroid
            float cx_mask = 0, cy_mask = 0;
            for (const auto& pt : all_mask_pts) {
                cx_mask += pt.x;
                cy_mask += pt.y;
            }
            if (!all_mask_pts.empty()) {
                cx_mask /= all_mask_pts.size();
                cy_mask /= all_mask_pts.size();
            }

            // Sort by distance to centroid, keep closest 70%
            std::vector<std::pair<float, size_t>> dist_idx;
            dist_idx.reserve(all_mask_pts.size());
            for (size_t i = 0; i < all_mask_pts.size(); ++i) {
                float dx = all_mask_pts[i].x - cx_mask;
                float dy = all_mask_pts[i].y - cy_mask;
                dist_idx.push_back({dx * dx + dy * dy, i});
            }
            std::sort(dist_idx.begin(), dist_idx.end());

            int n_keep = std::max(20, static_cast<int>(dist_idx.size() * 0.7));
            n_keep = std::min(n_keep, static_cast<int>(dist_idx.size()));

            std::vector<cv::Point2f> mask_pts_2d;
            mask_pts_2d.reserve(n_keep);
            for (int i = 0; i < n_keep; ++i) {
                mask_pts_2d.push_back(all_mask_pts[dist_idx[i].second]);
            }

            double angle_2d_deg = 0;

            if (mask_pts_2d.size() >= 20) {
                cv::Mat pts_mat(mask_pts_2d.size(), 2, CV_32F);
                for (size_t i = 0; i < mask_pts_2d.size(); ++i) {
                    pts_mat.at<float>(i, 0) = mask_pts_2d[i].x;
                    pts_mat.at<float>(i, 1) = mask_pts_2d[i].y;
                }
                cv::PCA pca_2d(pts_mat, cv::Mat(), cv::PCA::DATA_AS_ROW);

                float ev0_2d = pca_2d.eigenvalues.at<float>(0);
                float ev1_2d = std::max(pca_2d.eigenvalues.at<float>(1), 1e-6f);
                float ev_ratio_2d = ev0_2d / ev1_2d;

                // Only meaningful if mask is elongated (earphone-like shape)
                if (ev_ratio_2d > 1.3f) {
                    float dir_u = pca_2d.eigenvectors.at<float>(0, 0);
                    float dir_v = pca_2d.eigenvectors.at<float>(0, 1);

                    // --- Step 2: Project ear canal direction to 2D ---
                    // ear_plane_.normal ≈ (nx, ny, nz) with nz ≈ -1
                    // In image plane, canal direction ∝ (nx*fx, ny*fy)
                    double ear_2d_u = -ear_plane_.normal.x() * fx;
                    double ear_2d_v = -ear_plane_.normal.y() * fy;
                    double ear_2d_len = std::sqrt(
                        ear_2d_u * ear_2d_u + ear_2d_v * ear_2d_v);

                    if (ear_2d_len > 1.0) {
                        // Ear canal has a meaningful 2D projection
                        ear_2d_u /= ear_2d_len;
                        ear_2d_v /= ear_2d_len;

                        // Angle between earphone's 2D axis and ear canal's 2D direction
                        double cos_2d = std::abs(
                            static_cast<double>(dir_u) * ear_2d_u +
                            static_cast<double>(dir_v) * ear_2d_v);
                        angle_2d_deg = std::acos(
                            std::clamp(cos_2d, 0.0, 1.0)) * 180.0 / M_PI;
                    } else {
                        // Ear canal is perpendicular to image → tilt from elongation
                        // A perfectly inserted earphone appears circular (ev_ratio ≈ 1)
                        // More elongated = more tilted
                        angle_2d_deg = std::atan(
                            std::sqrt(std::max(0.0, ev_ratio_2d - 1.0))) * 180.0 / M_PI;
                    }

                    // Construct effective_axis that produces this angle
                    // relative to the camera Z axis (reference)
                    double angle_rad = angle_2d_deg * M_PI / 180.0;
                    // Direction of tilt in 3D camera frame
                    Eigen::Vector3d tilt_dir(
                        static_cast<double>(dir_u) / fx,
                        static_cast<double>(dir_v) / fy, 0);
                    if (tilt_dir.norm() > 1e-9) tilt_dir.normalize();

                    Eigen::Vector3d canal_dir = -ear_plane_.normal;
                    if (canal_dir.z() < 0) canal_dir = -canal_dir;

                    effective_axis = (canal_dir * std::cos(angle_rad) +
                                     tilt_dir * std::sin(angle_rad)).normalized();
                    if (effective_axis.z() < 0) effective_axis = -effective_axis;

                    used_2d_pca = true;

                    RCLCPP_INFO(get_logger(),
                        "2D PCA: dir=(%.3f,%.3f) ev_ratio_2d=%.1f "
                        "ear_2d=(%.3f,%.3f) angle_2d=%.1f° axis=(%.3f,%.3f,%.3f)",
                        dir_u, dir_v, ev_ratio_2d,
                        ear_2d_u, ear_2d_v, angle_2d_deg,
                        effective_axis.x(), effective_axis.y(), effective_axis.z());
                }
            }

            if (!used_2d_pca) {
                // Fallback: ear plane normal (mask too compact for 2D PCA)
                effective_axis = -ear_plane_.normal;
                if (effective_axis.z() < 0) effective_axis = -effective_axis;

                RCLCPP_INFO(get_logger(),
                    "Hole-dominated: mask too compact for 2D PCA, "
                    "using ear plane normal. n_zero=%d/%zu",
                    n_zero_in_mask, points_3d.size());
            }
        }

        // --- ArUco: compute angle relative to ear normal if available ---
        ArUcoResult aruco;
        if (enable_aruco_ && !cur_color.empty()) {
            aruco = detect_aruco(cur_color, fx, fy, cx, cy);
        }

        // --- Compute angle ---
        Eigen::Vector3d reference_axis(0, 0, 1);  // camera optical axis
        if (aruco.detected) {
            reference_axis = -aruco.ear_normal;  // ear canal direction
        }

        double cos_angle = effective_axis.dot(reference_axis);
        cos_angle = std::clamp(cos_angle, -1.0, 1.0);
        double angle_deg = std::acos(cos_angle) * 180.0 / M_PI;

        // --- Compute depth (insertion/protrusion from ear surface) ---
        double median_depth = 0;
        if (!depth_diffs.empty()) {
            // Use median of actual depth differences (from non-hole pixels)
            std::sort(depth_diffs.begin(), depth_diffs.end());
            median_depth = depth_diffs[depth_diffs.size() / 2];
        } else if (!points_3d.empty()) {
            // Fallback: estimate depth from point cloud distance to ear plane
            // (when all detected pixels are holes with no real depth diff)
            std::vector<double> plane_dists;
            for (const auto& p : points_3d) {
                double d = std::abs(ear_plane_.signed_distance(p)) * 1000.0;  // m → mm
                plane_dists.push_back(d);
            }
            std::sort(plane_dists.begin(), plane_dists.end());
            median_depth = plane_dists[plane_dists.size() / 2];
        }

        // --- Confidence ---
        double confidence = compute_confidence(pca, m.area, m.stats, aruco.detected);

        // Apply penalties for degraded conditions
        if (is_sparse) {
            double sparse_penalty = static_cast<double>(points_3d.size()) /
                                    static_cast<double>(min_area_);
            confidence *= sparse_penalty * 0.5;
        }
        if (pca_unstable) {
            confidence *= 0.6;
        }
        if (used_rgb_fallback && m.stats.primary_channel == "rgb") {
            confidence *= 0.5;
        }
        confidence = std::clamp(confidence, 0.0, 1.0);

        // --- Fill result ---
        m.valid = true;
        m.angle_deg = angle_deg;
        m.depth_mm = median_depth;
        m.confidence = confidence;
        m.axis = effective_axis;
        m.centroid = pca.centroid;
        m.n_points = pca.n_points;

        char buf[768];
        std::snprintf(buf, sizeof(buf),
            "Angle=%.1f deg  Depth=%.1fmm  Conf=%.2f  Points=%d  "
            "Area=%.0f  Channel=%s  Axis=(%.3f,%.3f,%.3f)  "
            "EV_ratio=%.1f%s%s%s%s",
            angle_deg, median_depth, confidence, pca.n_points,
            m.area, m.stats.primary_channel.c_str(),
            effective_axis.x(), effective_axis.y(), effective_axis.z(),
            ev_ratio,
            mask_hole_dominated ? (used_2d_pca ? " [2D_PCA]" : " [EAR_PLANE_AXIS]") :
                (pca_unstable ? " [PCA_UNSTABLE]" : ""),
            is_sparse ? " [SPARSE]" : "",
            aruco.detected ? " [ARUCO]" : "",
            hole_dominated ? " [HOLE_MERGE]" : "");
        m.detail = buf;

        return m;
    }

    // ========================================================================
    // MeasureEarphone Service Handler
    // ========================================================================

    void handle_measure(
        const std::shared_ptr<vision_server::srv::MeasureEarphone::Request> req,
        std::shared_ptr<vision_server::srv::MeasureEarphone::Response> resp)
    {
        // --- Grab state ---
        std::deque<cv::Mat> buffer_copy;
        cv::Mat cur_color;
        double fx, fy, cx, cy;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (!has_baseline_) {
                resp->success = false;
                resp->message = "No baseline captured. Call capture_baseline first.";
                return;
            }
            if (depth_buffer_.empty()) {
                resp->success = false;
                resp->message = "No depth frames available";
                return;
            }
            if (!has_info_) {
                resp->success = false;
                resp->message = "No camera intrinsics available";
                return;
            }
            buffer_copy = depth_buffer_;
            if (last_color_) {
                try {
                    auto cv_c = cv_bridge::toCvShare(last_color_, "bgr8");
                    cur_color = cv_c->image.clone();
                } catch (...) {}
            }
            fx = fx_; fy = fy_; cx = cx_; cy = cy_;
        }

        int n_frames = static_cast<int>(buffer_copy.size());
        if (n_frames < 3) {
            resp->success = false;
            resp->message = "Insufficient depth frames (need >= 3, have "
                            + std::to_string(n_frames) + ")";
            return;
        }

        // --- Compute current averaged depth ---
        int n_use = std::min(n_frames, avg_frames_);
        cv::Mat current_avg = compute_median_depth(buffer_copy, n_use);
        if (current_avg.empty()) {
            resp->success = false;
            resp->message = "Failed to compute current depth median";
            return;
        }

        // --- Primary measurement ---
        SingleMeasurement primary = run_pipeline(current_avg, cur_color, fx, fy, cx, cy);

        if (!primary.valid) {
            // --- Failed: save diagnostics and return ---
            resp->success = false;
            resp->message = "Detection failed: " + primary.detail;

            if (!req->file_tag.empty()) {
                resp->saved_path = save_debug_failure(
                    req->file_tag, current_avg, cur_color,
                    primary.stats, primary.detail);

                // Also save per-channel debug masks for diagnosis
                if (!resp->saved_path.empty()) {
                    fs::path dbg_dir(resp->saved_path);
                    if (!primary.debug_mask_diff.empty())
                        cv::imwrite((dbg_dir / "mask_diff.png").string(), primary.debug_mask_diff);
                    if (!primary.debug_mask_diff_low.empty())
                        cv::imwrite((dbg_dir / "mask_diff_low.png").string(), primary.debug_mask_diff_low);
                    if (!primary.debug_mask_holes.empty())
                        cv::imwrite((dbg_dir / "mask_holes.png").string(), primary.debug_mask_holes);
                    if (!primary.debug_mask_plane.empty())
                        cv::imwrite((dbg_dir / "mask_plane.png").string(), primary.debug_mask_plane);
                    if (!primary.debug_candidates.empty())
                        cv::imwrite((dbg_dir / "candidates.png").string(), primary.debug_candidates);
                    if (!primary.debug_combined.empty())
                        cv::imwrite((dbg_dir / "combined_before_cluster.png").string(), primary.debug_combined);
                    if (!primary.earphone_mask.empty())
                        cv::imwrite((dbg_dir / "earphone_mask.png").string(), primary.earphone_mask);
                }
            }
            RCLCPP_WARN(get_logger(), "Measurement failed: %s", primary.detail.c_str());
            return;
        }

        // --- Multi-trial validation (optional) ---
        double final_angle = primary.angle_deg;
        double final_depth = primary.depth_mm;
        double final_confidence = primary.confidence;
        std::string trial_info;

        if (enable_multi_trial_ && n_frames >= 6 &&
            primary.confidence > 0.2 && primary.confidence < 0.8)
        {
            // Trial 2: first half of buffer
            cv::Mat avg_first = compute_median_depth(buffer_copy, n_use / 2);
            // Trial 3: last half but offset
            std::deque<cv::Mat> second_half(
                buffer_copy.begin() + n_frames / 2, buffer_copy.end());
            cv::Mat avg_second = compute_median_depth(second_half, n_use / 2);

            std::vector<double> angles = {primary.angle_deg};
            std::vector<double> depths = {primary.depth_mm};

            if (!avg_first.empty()) {
                auto m2 = run_pipeline(avg_first, cur_color, fx, fy, cx, cy);
                if (m2.valid) {
                    angles.push_back(m2.angle_deg);
                    depths.push_back(m2.depth_mm);
                }
            }
            if (!avg_second.empty()) {
                auto m3 = run_pipeline(avg_second, cur_color, fx, fy, cx, cy);
                if (m3.valid) {
                    angles.push_back(m3.angle_deg);
                    depths.push_back(m3.depth_mm);
                }
            }

            if (angles.size() >= 2) {
                // Compute standard deviation
                double angle_mean = std::accumulate(angles.begin(), angles.end(), 0.0) / angles.size();
                double depth_mean = std::accumulate(depths.begin(), depths.end(), 0.0) / depths.size();

                double angle_var = 0, depth_var = 0;
                for (size_t i = 0; i < angles.size(); ++i) {
                    angle_var += (angles[i] - angle_mean) * (angles[i] - angle_mean);
                    depth_var += (depths[i] - depth_mean) * (depths[i] - depth_mean);
                }
                double angle_std = std::sqrt(angle_var / angles.size());
                double depth_std = std::sqrt(depth_var / depths.size());

                // Use median of trials
                std::sort(angles.begin(), angles.end());
                std::sort(depths.begin(), depths.end());
                final_angle = angles[angles.size() / 2];
                final_depth = depths[depths.size() / 2];

                // Penalize confidence if measurements are unstable
                if (angle_std > 5.0 || depth_std > 3.0) {
                    double stability = 1.0 / (1.0 + angle_std / 10.0 + depth_std / 5.0);
                    final_confidence *= stability;
                }

                char tb[128];
                std::snprintf(tb, sizeof(tb),
                    " | trials=%zu angle_std=%.1f depth_std=%.1f",
                    angles.size(), angle_std, depth_std);
                trial_info = tb;
            }
        }

        final_confidence = std::clamp(final_confidence, 0.0, 1.0);

        // --- Build response ---
        resp->success = true;
        resp->angle_deg = final_angle;
        resp->depth_mm = final_depth;
        resp->confidence = final_confidence;

        std::string reliability;
        if (final_confidence < min_confidence_) {
            reliability = " [LOW_CONFIDENCE]";
        }

        resp->message = primary.detail + trial_info + reliability;

        RCLCPP_INFO(get_logger(), "Measurement: angle=%.1f depth=%.1f conf=%.2f%s",
            final_angle, final_depth, final_confidence,
            reliability.c_str());

        // --- Save results ---
        if (!req->file_tag.empty()) {
            resp->saved_path = save_results(
                req->file_tag, current_avg, cur_color,
                primary.earphone_mask, primary.contour,
                final_angle, final_depth, final_confidence,
                primary.axis, primary.centroid,
                primary.stats, primary.detail + trial_info,
                fx, fy, cx, cy, primary);
        }
    }

    // ========================================================================
    // Save Results & Debug
    // ========================================================================

    std::string save_results(
        const std::string& tag,
        const cv::Mat& current_avg, const cv::Mat& cur_color,
        const cv::Mat& mask, const std::vector<cv::Point>& contour,
        double angle_deg, double depth_mm, double confidence,
        const Eigen::Vector3d& axis, const Eigen::Vector3d& centroid,
        const DetectionChannelStats& stats, const std::string& detail,
        double fx, double fy, double cx, double cy,
        const SingleMeasurement& meas = {})
    {
        auto stamp = this->now().nanoseconds();
        fs::path dir = fs::path(save_dir_) / tag / std::to_string(stamp);
        try { fs::create_directories(dir); } catch (...) { return ""; }

        // --- Depth visualizations ---
        auto save_depth_vis = [](const cv::Mat& d_f32, const fs::path& path) {
            cv::Mat u16, vis;
            d_f32.convertTo(u16, CV_16UC1);
            cv::normalize(u16, vis, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            cv::applyColorMap(vis, vis, cv::COLORMAP_JET);
            cv::imwrite(path.string(), vis);
        };

        save_depth_vis(baseline_avg_, dir / "baseline_depth.png");
        save_depth_vis(current_avg, dir / "measure_depth.png");

        // Depth difference visualization (includes hole regions)
        {
            cv::Mat diff_vis = cv::Mat::zeros(baseline_avg_.size(), CV_32FC1);
            cv::Mat hole_vis = cv::Mat::zeros(baseline_avg_.size(), CV_8UC1);
            for (int y = 0; y < baseline_avg_.rows; ++y) {
                const float* rb = baseline_avg_.ptr<float>(y);
                const float* rc = current_avg.ptr<float>(y);
                float* rd = diff_vis.ptr<float>(y);
                uint8_t* rh = hole_vis.ptr<uint8_t>(y);
                for (int x = 0; x < baseline_avg_.cols; ++x) {
                    if (rb[x] > 1.0f && rc[x] > 1.0f) {
                        rd[x] = rb[x] - rc[x];
                    } else if (rb[x] > 100.0f && rc[x] < 1.0f) {
                        // Depth hole: mark as large positive diff for visualization
                        rd[x] = 30.0f;  // max visible diff
                        rh[x] = 255;
                    }
                }
            }
            cv::Mat vis8;
            // Map [-10, 30] mm range to [0, 255]
            diff_vis = (diff_vis + 10.0f) * (255.0f / 40.0f);
            diff_vis = cv::max(diff_vis, 0.0f);
            diff_vis = cv::min(diff_vis, 255.0f);
            diff_vis.convertTo(vis8, CV_8UC1);
            cv::Mat vis_color;
            cv::applyColorMap(vis8, vis_color, cv::COLORMAP_TURBO);

            // Overlay hole regions in bright magenta for visibility
            for (int y = 0; y < hole_vis.rows; ++y) {
                const uint8_t* rh = hole_vis.ptr<uint8_t>(y);
                auto* vc = vis_color.ptr<cv::Vec3b>(y);
                for (int x = 0; x < hole_vis.cols; ++x) {
                    if (rh[x]) vc[x] = cv::Vec3b(255, 0, 255);  // magenta = hole
                }
            }
            cv::imwrite((dir / "depth_diff.png").string(), vis_color);

            // Save hole mask separately
            cv::imwrite((dir / "hole_mask.png").string(), hole_vis);
        }

        // Noise map visualization
        {
            cv::Mat noise_vis;
            cv::normalize(noise_map_, noise_vis, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            cv::applyColorMap(noise_vis, noise_vis, cv::COLORMAP_HOT);
            cv::imwrite((dir / "noise_map.png").string(), noise_vis);
        }

        // Color images
        if (!baseline_color_.empty())
            cv::imwrite((dir / "baseline_color.png").string(), baseline_color_);
        if (!cur_color.empty())
            cv::imwrite((dir / "measure_color.png").string(), cur_color);

        // Save raw baseline data so this result dir can be used as baseline source
        cv::imwrite((dir / "baseline_avg.exr").string(), baseline_avg_);
        {
            cv::FileStorage meta((dir / "meta.yaml").string(),
                                  cv::FileStorage::WRITE);
            meta << "roi_x" << ear_roi_.x;
            meta << "roi_y" << ear_roi_.y;
            meta << "roi_w" << ear_roi_.width;
            meta << "roi_h" << ear_roi_.height;
            meta << "plane_nx" << ear_plane_.normal.x();
            meta << "plane_ny" << ear_plane_.normal.y();
            meta << "plane_nz" << ear_plane_.normal.z();
            meta << "plane_d" << ear_plane_.d;
            meta << "fx" << fx; meta << "fy" << fy;
            meta << "cx" << cx; meta << "cy" << cy;
            meta.release();
        }

        // Earphone mask
        cv::imwrite((dir / "earphone_mask.png").string(), mask);

        // Debug: per-channel masks and pipeline intermediates
        if (!meas.debug_mask_diff.empty())
            cv::imwrite((dir / "mask_diff.png").string(), meas.debug_mask_diff);
        if (!meas.debug_mask_diff_low.empty())
            cv::imwrite((dir / "mask_diff_low.png").string(), meas.debug_mask_diff_low);
        if (!meas.debug_mask_holes.empty())
            cv::imwrite((dir / "mask_holes.png").string(), meas.debug_mask_holes);
        if (!meas.debug_mask_plane.empty())
            cv::imwrite((dir / "mask_plane.png").string(), meas.debug_mask_plane);
        if (!meas.debug_candidates.empty())
            cv::imwrite((dir / "candidates.png").string(), meas.debug_candidates);
        if (!meas.debug_combined.empty())
            cv::imwrite((dir / "combined_before_cluster.png").string(), meas.debug_combined);

        // ROI visualization on mask
        {
            cv::Mat roi_vis = cv::Mat::zeros(mask.size(), CV_8UC3);
            cv::Mat mask3;
            cv::cvtColor(mask, mask3, cv::COLOR_GRAY2BGR);
            roi_vis = mask3.clone();
            cv::rectangle(roi_vis, ear_roi_, cv::Scalar(0, 255, 0), 2);
            cv::imwrite((dir / "roi_overlay.png").string(), roi_vis);
        }

        // Debug overlay on color image
        if (!cur_color.empty()) {
            cv::Mat overlay = cur_color.clone();
            double sx = static_cast<double>(overlay.cols) / baseline_avg_.cols;
            double sy = static_cast<double>(overlay.rows) / baseline_avg_.rows;

            // Draw ROI
            cv::Rect scaled_roi(
                static_cast<int>(ear_roi_.x * sx), static_cast<int>(ear_roi_.y * sy),
                static_cast<int>(ear_roi_.width * sx), static_cast<int>(ear_roi_.height * sy));
            cv::rectangle(overlay, scaled_roi, cv::Scalar(255, 255, 0), 1);

            // Draw contour
            if (!contour.empty()) {
                std::vector<cv::Point> scaled_contour;
                for (const auto& pt : contour) {
                    scaled_contour.emplace_back(
                        static_cast<int>(pt.x * sx),
                        static_cast<int>(pt.y * sy));
                }
                cv::drawContours(overlay,
                    std::vector<std::vector<cv::Point>>{scaled_contour},
                    0, cv::Scalar(0, 255, 0), 2);
            }

            // Project centroid and draw axis arrow
            if (centroid.z() > 0.01) {
                int cu = static_cast<int>((centroid.x() * fx / centroid.z() + cx) * sx);
                int cv_pt = static_cast<int>((centroid.y() * fy / centroid.z() + cy) * sy);

                double arrow_len = 60.0;
                int ax = cu + static_cast<int>(axis.x() / std::max(axis.z(), 0.01) * fx * 0.02 * sx * arrow_len);
                int ay = cv_pt + static_cast<int>(axis.y() / std::max(axis.z(), 0.01) * fy * 0.02 * sy * arrow_len);
                cv::arrowedLine(overlay, cv::Point(cu, cv_pt), cv::Point(ax, ay),
                                cv::Scalar(0, 0, 255), 2, cv::LINE_AA, 0, 0.3);
                cv::circle(overlay, cv::Point(cu, cv_pt), 5, cv::Scalar(0, 255, 255), -1);
            }

            // Text annotations
            int text_y = 25;
            auto put_text = [&](const std::string& txt, cv::Scalar color = cv::Scalar(0, 255, 255)) {
                cv::putText(overlay, txt, cv::Point(10, text_y),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
                text_y += 25;
            };

            char buf[128];
            std::snprintf(buf, sizeof(buf), "Angle: %.1f deg", angle_deg);
            put_text(buf);
            std::snprintf(buf, sizeof(buf), "Depth: %.1f mm", depth_mm);
            put_text(buf);
            std::snprintf(buf, sizeof(buf), "Conf: %.2f", confidence);
            put_text(buf, confidence >= min_confidence_
                ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255));
            std::snprintf(buf, sizeof(buf), "Channel: %s", stats.primary_channel.c_str());
            put_text(buf);
            std::snprintf(buf, sizeof(buf), "Noise: %.1f mm", stats.avg_noise_sigma);
            put_text(buf);

            cv::imwrite((dir / "debug_overlay.png").string(), overlay);
        }

        // result.json
        {
            std::ofstream ofs((dir / "result.json").string());
            if (ofs.is_open()) {
                ofs << "{\n"
                    << "  \"angle_deg\": " << angle_deg << ",\n"
                    << "  \"depth_mm\": " << depth_mm << ",\n"
                    << "  \"confidence\": " << confidence << ",\n"
                    << "  \"axis\": [" << axis.x() << ", " << axis.y() << ", " << axis.z() << "],\n"
                    << "  \"centroid_m\": [" << centroid.x() << ", " << centroid.y() << ", " << centroid.z() << "],\n"
                    << "  \"roi\": [" << ear_roi_.x << ", " << ear_roi_.y << ", " << ear_roi_.width << ", " << ear_roi_.height << "],\n"
                    << "  \"plane_normal\": [" << ear_plane_.normal.x() << ", " << ear_plane_.normal.y() << ", " << ear_plane_.normal.z() << "],\n"
                    << "  \"plane_d\": " << ear_plane_.d << ",\n"
                    << "  \"stats\": {\n"
                    << "    \"diff_pixels\": " << stats.diff_pixels << ",\n"
                    << "    \"plane_pixels\": " << stats.plane_pixels << ",\n"
                    << "    \"hole_pixels\": " << stats.hole_pixels << ",\n"
                    << "    \"rgb_pixels\": " << stats.rgb_pixels << ",\n"
                    << "    \"zero_depth_in_roi\": " << stats.zero_depth_in_roi << ",\n"
                    << "    \"avg_noise_sigma_mm\": " << stats.avg_noise_sigma << ",\n"
                    << "    \"primary_channel\": \"" << stats.primary_channel << "\"\n"
                    << "  },\n"
                    << "  \"detail\": \"" << detail << "\",\n"
                    << "  \"camera_ns\": \"" << camera_ns_ << "\",\n"
                    << "  \"timestamp_ns\": " << stamp << "\n"
                    << "}\n";
            }
        }

        RCLCPP_INFO(get_logger(), "Results saved to %s", dir.string().c_str());
        return dir.string();
    }

    std::string save_debug_failure(
        const std::string& tag,
        const cv::Mat& current_avg, const cv::Mat& cur_color,
        const DetectionChannelStats& stats, const std::string& detail)
    {
        auto stamp = this->now().nanoseconds();
        fs::path dir = fs::path(save_dir_) / (tag + "_FAILED") / std::to_string(stamp);
        try { fs::create_directories(dir); } catch (...) { return ""; }

        // Save depth visualizations for offline debugging
        auto save_depth_vis = [](const cv::Mat& d_f32, const fs::path& path) {
            cv::Mat u16, vis;
            d_f32.convertTo(u16, CV_16UC1);
            cv::normalize(u16, vis, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            cv::applyColorMap(vis, vis, cv::COLORMAP_JET);
            cv::imwrite(path.string(), vis);
        };

        save_depth_vis(baseline_avg_, dir / "baseline_depth.png");
        save_depth_vis(current_avg, dir / "measure_depth.png");

        if (!baseline_color_.empty())
            cv::imwrite((dir / "baseline_color.png").string(), baseline_color_);
        if (!cur_color.empty())
            cv::imwrite((dir / "measure_color.png").string(), cur_color);

        // Depth difference map (includes holes in magenta)
        {
            cv::Mat diff_vis = cv::Mat::zeros(baseline_avg_.size(), CV_32FC1);
            cv::Mat hole_vis = cv::Mat::zeros(baseline_avg_.size(), CV_8UC1);
            for (int y = 0; y < baseline_avg_.rows; ++y) {
                const float* rb = baseline_avg_.ptr<float>(y);
                const float* rc = current_avg.ptr<float>(y);
                float* rd = diff_vis.ptr<float>(y);
                uint8_t* rh = hole_vis.ptr<uint8_t>(y);
                for (int x = 0; x < baseline_avg_.cols; ++x) {
                    if (rb[x] > 1.0f && rc[x] > 1.0f) {
                        rd[x] = rb[x] - rc[x];
                    } else if (rb[x] > 100.0f && rc[x] < 1.0f) {
                        rd[x] = 30.0f;
                        rh[x] = 255;
                    }
                }
            }
            cv::Mat vis8;
            diff_vis = (diff_vis + 10.0f) * (255.0f / 40.0f);
            diff_vis = cv::max(diff_vis, 0.0f);
            diff_vis = cv::min(diff_vis, 255.0f);
            diff_vis.convertTo(vis8, CV_8UC1);
            cv::Mat vis_color;
            cv::applyColorMap(vis8, vis_color, cv::COLORMAP_TURBO);
            for (int y = 0; y < hole_vis.rows; ++y) {
                const uint8_t* rh = hole_vis.ptr<uint8_t>(y);
                auto* vc = vis_color.ptr<cv::Vec3b>(y);
                for (int x = 0; x < hole_vis.cols; ++x) {
                    if (rh[x]) vc[x] = cv::Vec3b(255, 0, 255);
                }
            }
            cv::imwrite((dir / "depth_diff.png").string(), vis_color);
        }

        // Noise map
        {
            cv::Mat noise_vis;
            cv::normalize(noise_map_, noise_vis, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            cv::applyColorMap(noise_vis, noise_vis, cv::COLORMAP_HOT);
            cv::imwrite((dir / "noise_map.png").string(), noise_vis);
        }

        // ROI on color
        if (!cur_color.empty()) {
            cv::Mat overlay = cur_color.clone();
            double sx = static_cast<double>(overlay.cols) / baseline_avg_.cols;
            double sy = static_cast<double>(overlay.rows) / baseline_avg_.rows;
            cv::Rect scaled_roi(
                static_cast<int>(ear_roi_.x * sx), static_cast<int>(ear_roi_.y * sy),
                static_cast<int>(ear_roi_.width * sx), static_cast<int>(ear_roi_.height * sy));
            cv::rectangle(overlay, scaled_roi, cv::Scalar(0, 0, 255), 2);
            cv::putText(overlay, "DETECTION FAILED", cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
            cv::imwrite((dir / "debug_overlay.png").string(), overlay);
        }

        // Failure report
        {
            std::ofstream ofs((dir / "failure_report.json").string());
            if (ofs.is_open()) {
                ofs << "{\n"
                    << "  \"status\": \"FAILED\",\n"
                    << "  \"reason\": \"" << detail << "\",\n"
                    << "  \"stats\": {\n"
                    << "    \"diff_pixels\": " << stats.diff_pixels << ",\n"
                    << "    \"plane_pixels\": " << stats.plane_pixels << ",\n"
                    << "    \"hole_pixels\": " << stats.hole_pixels << ",\n"
                    << "    \"rgb_pixels\": " << stats.rgb_pixels << ",\n"
                    << "    \"zero_depth_in_roi\": " << stats.zero_depth_in_roi << ",\n"
                    << "    \"avg_noise_sigma_mm\": " << stats.avg_noise_sigma << ",\n"
                    << "    \"primary_channel\": \"" << stats.primary_channel << "\"\n"
                    << "  },\n"
                    << "  \"roi\": [" << ear_roi_.x << ", " << ear_roi_.y << ", " << ear_roi_.width << ", " << ear_roi_.height << "],\n"
                    << "  \"plane_normal\": [" << ear_plane_.normal.x() << ", " << ear_plane_.normal.y() << ", " << ear_plane_.normal.z() << "],\n"
                    << "  \"plane_d\": " << ear_plane_.d << ",\n"
                    << "  \"camera_ns\": \"" << camera_ns_ << "\",\n"
                    << "  \"timestamp_ns\": " << this->now().nanoseconds() << "\n"
                    << "}\n";
            }
        }

        RCLCPP_INFO(get_logger(), "Failure diagnostics saved to %s", dir.string().c_str());
        return dir.string();
    }

    // ========================================================================
    // Member Variables
    // ========================================================================

    // --- Parameters ---
    std::string camera_ns_;
    int avg_frames_;
    double min_diff_mm_, max_diff_mm_;
    int min_area_, roi_w_, roi_h_;
    double plane_inlier_mm_;
    int plane_ransac_iters_;
    double plane_fg_min_mm_, plane_fg_max_mm_;
    double pca_trim_pct_;
    double noise_sigma_scale_;
    bool enable_aruco_;
    double aruco_marker_size_;
    bool enable_multi_trial_;
    double min_confidence_;
    int roi_margin_;
    std::string save_dir_;

    // --- Camera intrinsics ---
    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
    bool has_info_ = false;

    // --- Frame buffer ---
    std::deque<cv::Mat> depth_buffer_;  // ring buffer of 16UC1 frames

    // --- Baseline state ---
    cv::Mat baseline_avg_;    // CV_32FC1, temporal median (mm)
    cv::Mat baseline_color_;
    cv::Mat noise_map_;       // CV_32FC1, per-pixel noise sigma (mm)
    Plane ear_plane_;
    cv::Rect ear_roi_;
    bool has_baseline_ = false;

    // --- Latest frames ---
    sensor_msgs::msg::Image::SharedPtr last_color_;
    std::mutex mtx_;

    // --- ROS interfaces ---
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      sub_depth_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      sub_color_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
    rclcpp::Service<vision_server::srv::CaptureBaseline>::SharedPtr srv_baseline_;
    rclcpp::Service<vision_server::srv::MeasureEarphone>::SharedPtr srv_measure_;
};

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EarphoneInspectorNode>());
    rclcpp::shutdown();
    return 0;
}
