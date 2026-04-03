// =============================================================================
// collision_detector_node.cpp
// =============================================================================
// 基于深度相机点云 + TF 的机械臂实时碰撞检测节点
//
// 处理流水线:
//   PointCloud2 → ROI裁剪 → 地面去除 → Stride降采样 → 自身过滤 → 距离计算 → 阈值判断
//
// 设计决策:
//   - 不使用 PCL: 避免大型依赖, 直接操作 PointCloud2 raw data 更快
//   - 球体模型: 工程上最简单可靠, 精度足够(vs 胶囊体/OBB, 实现复杂收益小)
//   - Stride 降采样: 比 voxel grid 快(无需建哈希表), 均匀性足够
//   - Timer 驱动: 解耦点云帧率和检测帧率, 点云30Hz但检测10Hz足矣
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
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
#include <algorithm>

using namespace std::chrono_literals;

// =============================================================================
// 辅助结构体
// =============================================================================

/// 球体碰撞模型: 表示一个 link 的碰撞包围体
struct Sphere {
    double x, y, z;       // 球心 (相机坐标系)
    double r;             // 半径 (米)
    std::string name;     // link 名称
};

/// 检测结果: 单次检测的汇总信息
struct DetectionResult {
    double min_distance = std::numeric_limits<double>::max();
    std::string nearest_link;
    int obstacle_count = 0;
    // 最近障碍物点 (相机坐标系, 用于可视化)
    double nearest_x = 0, nearest_y = 0, nearest_z = 0;
    // 最近的球心位置 (用于可视化连线)
    double nearest_sphere_x = 0, nearest_sphere_y = 0, nearest_sphere_z = 0;
};

// =============================================================================
// CollisionDetectorNode
// =============================================================================

class CollisionDetectorNode : public rclcpp::Node {
public:
    CollisionDetectorNode() : Node("collision_detector") {
        declare_all_parameters();
        load_all_parameters();

        // --- TF ---
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // --- 订阅点云 ---
        subscribe_point_cloud(pc_topic_);

        // --- 动态切换点云话题 ---
        sub_set_topic_ = create_subscription<std_msgs::msg::String>(
            "/collision_detector/set_topic", 10,
            [this](std_msgs::msg::String::SharedPtr msg) {
                if (msg->data.empty() || msg->data == pc_topic_) return;
                RCLCPP_INFO(get_logger(), "切换点云话题: %s -> %s",
                            pc_topic_.c_str(), msg->data.c_str());
                pc_topic_ = msg->data;
                { std::lock_guard<std::mutex> lk(pc_mutex_); last_pc_.reset(); }
                subscribe_point_cloud(pc_topic_);
            });

        // --- 机械臂硬件碰撞状态 ---
        sub_state_ = create_subscription<duco_msg::msg::DucoRobotState>(
            "/duco_cobot/robot_state", 10,
            [this](duco_msg::msg::DucoRobotState::SharedPtr msg) {
                if (msg->collision) hw_collision_.store(true);
            });

        // --- 发布者 ---
        pub_status_ = create_publisher<common_msgs::msg::CollisionStatus>(
            "/collision_detector/status", 10);
        pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/collision_detector/markers", 10);

        // --- 服务客户端 ---
        client_pause_ = create_client<std_srvs::srv::SetBool>("/system/pause_task");

        // --- 定时检测 ---
        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / detection_hz_));
        timer_ = create_wall_timer(period, [this]() { run_detection(); });

        RCLCPP_INFO(get_logger(),
            "CollisionDetector 已启动 | topic=%s | hz=%.0f | stride=%d | roi=%s | ground=%s | viz=%s",
            pc_topic_.c_str(), detection_hz_, stride_,
            enable_roi_ ? "ON" : "OFF",
            enable_ground_removal_ ? "ON" : "OFF",
            enable_viz_ ? "ON" : "OFF");
    }

private:
    // ==========================================================================
    // 参数声明与加载
    // ==========================================================================
    void declare_all_parameters() {
        // 点云输入
        declare_parameter("point_cloud_topic",  "/camera/depth/points");
        // 性能
        declare_parameter("detection_hz",       10.0);
        declare_parameter("downsample_stride",  4);
        // 球体模型
        declare_parameter("link_names", std::vector<std::string>{
            "base_link","link_1","link_2","link_3","link_4","link_5","link_6"});
        declare_parameter("link_radii", std::vector<double>{
            0.15, 0.12, 0.10, 0.09, 0.08, 0.07, 0.06});
        declare_parameter("self_filter_margin", 0.04);
        // ROI
        declare_parameter("enable_roi",   true);
        declare_parameter("roi_x_min",   -0.8);
        declare_parameter("roi_x_max",    0.8);
        declare_parameter("roi_y_min",   -0.8);
        declare_parameter("roi_y_max",    0.8);
        declare_parameter("roi_z_min",    0.3);
        declare_parameter("roi_z_max",    1.5);
        // 地面去除
        declare_parameter("enable_ground_removal", true);
        declare_parameter("ground_z",             -0.01);
        // 阈值
        declare_parameter("emergency_threshold", 0.03);
        declare_parameter("warning_threshold",   0.10);
        declare_parameter("caution_threshold",   0.20);
        // 控制
        declare_parameter("auto_pause",              true);
        declare_parameter("emergency_confirm_frames", 3);
        // 可视化
        declare_parameter("enable_visualization",    true);
    }

    void load_all_parameters() {
        pc_topic_      = get_parameter("point_cloud_topic").as_string();
        detection_hz_  = get_parameter("detection_hz").as_double();
        stride_        = get_parameter("downsample_stride").as_int();
        link_names_    = get_parameter("link_names").as_string_array();
        link_radii_    = get_parameter("link_radii").as_double_array();
        self_margin_   = get_parameter("self_filter_margin").as_double();

        enable_roi_    = get_parameter("enable_roi").as_bool();
        roi_x_min_     = get_parameter("roi_x_min").as_double();
        roi_x_max_     = get_parameter("roi_x_max").as_double();
        roi_y_min_     = get_parameter("roi_y_min").as_double();
        roi_y_max_     = get_parameter("roi_y_max").as_double();
        roi_z_min_     = get_parameter("roi_z_min").as_double();
        roi_z_max_     = get_parameter("roi_z_max").as_double();

        enable_ground_removal_ = get_parameter("enable_ground_removal").as_bool();
        ground_z_              = get_parameter("ground_z").as_double();

        thr_emergency_ = get_parameter("emergency_threshold").as_double();
        thr_warning_   = get_parameter("warning_threshold").as_double();
        thr_caution_   = get_parameter("caution_threshold").as_double();

        auto_pause_           = get_parameter("auto_pause").as_bool();
        emergency_confirm_n_  = get_parameter("emergency_confirm_frames").as_int();

        enable_viz_    = get_parameter("enable_visualization").as_bool();

        // 保证 radii 长度匹配 link_names
        while (link_radii_.size() < link_names_.size()) {
            link_radii_.push_back(0.10);
        }

        // 预计算球体半径平方 (self filter 用, 避免重复计算 sqrt)
        self_filter_r2_.resize(link_names_.size());
        for (size_t i = 0; i < link_names_.size(); ++i) {
            double r = link_radii_[i] + self_margin_;
            self_filter_r2_[i] = r * r;
        }
    }

    // ==========================================================================
    // 点云订阅
    // ==========================================================================
    void subscribe_point_cloud(const std::string& topic) {
        // 使用 SensorDataQoS: BEST_EFFORT + VOLATILE, 与相机驱动匹配
        sub_pc_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(pc_mutex_);
                last_pc_ = msg;
            });
    }

    // ==========================================================================
    // 核心检测流水线
    // ==========================================================================
    void run_detection() {
        auto out = common_msgs::msg::CollisionStatus();

        // ---- Step 0: 硬件碰撞 (最高优先级) ----
        if (hw_collision_.exchange(false)) {
            out.status = "emergency";
            out.min_distance = 0.0;
            out.nearest_link = "hardware";
            out.obstacle_points = 0;
            out.message = "硬件碰撞检测触发!";
            pub_status_->publish(out);
            try_pause();
            RCLCPP_ERROR(get_logger(), "%s", out.message.c_str());
            return;
        }

        // ---- Step 1: 获取点云 ----
        sensor_msgs::msg::PointCloud2::SharedPtr pc;
        {
            std::lock_guard<std::mutex> lk(pc_mutex_);
            pc = last_pc_;
        }

        if (!pc) {
            out.status = "sensor_unavailable";
            out.message = "等待点云数据...";
            pub_status_->publish(out);
            return;
        }

        double age = (now() - rclcpp::Time(pc->header.stamp)).seconds();
        if (age > 1.0) {
            out.status = "sensor_unavailable";
            out.message = "点云数据过期 (" + std::to_string(static_cast<int>(age*1000)) + "ms)";
            pub_status_->publish(out);
            return;
        }

        // ---- Step 2: 查找 TF, 构建球体模型 ----
        const std::string& cloud_frame = pc->header.frame_id;
        rclcpp::Time cloud_time(pc->header.stamp);

        std::vector<Sphere> spheres;
        spheres.reserve(link_names_.size());
        for (size_t i = 0; i < link_names_.size(); ++i) {
            try {
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
                // 个别 link TF 丢失不致命, 跳过即可
                RCLCPP_DEBUG(get_logger(), "TF 查找失败: %s -> %s: %s",
                            cloud_frame.c_str(), link_names_[i].c_str(), ex.what());
            }
        }
        if (spheres.empty()) {
            out.status = "tf_unavailable";
            out.message = "无法获取任何 link 的 TF";
            pub_status_->publish(out);
            return;
        }

        // ---- Step 2.5: 查找 base_link -> camera 的 TF (地面去除用) ----
        bool have_base_tf = false;
        geometry_msgs::msg::TransformStamped base_to_cam;
        if (enable_ground_removal_) {
            try {
                base_to_cam = tf_buffer_->lookupTransform(
                    cloud_frame, "base_link", cloud_time, 80ms);
                have_base_tf = true;
            } catch (const tf2::TransformException&) {
                // 地面���除退化: 无 base_link TF 时跳过地面去除
            }
        }

        // ---- Step 3: 解析点云字段偏移 ----
        int off_x = -1, off_y = -1, off_z = -1;
        for (const auto& f : pc->fields) {
            if (f.name == "x") off_x = f.offset;
            else if (f.name == "y") off_y = f.offset;
            else if (f.name == "z") off_z = f.offset;
        }
        if (off_x < 0 || off_y < 0 || off_z < 0) {
            out.status = "error";
            out.message = "点云缺少 xyz 字段";
            pub_status_->publish(out);
            return;
        }

        // ---- Step 4: 遍历点云, 执行检测流水线 ----
        const uint8_t* data = pc->data.data();
        uint32_t pstep = pc->point_step;
        size_t npts = static_cast<size_t>(pc->width) * pc->height;

        DetectionResult result;
        size_t stride = static_cast<size_t>(stride_);

        // 统计信息 (调试用)
        size_t total_processed = 0;
        size_t roi_filtered = 0;
        size_t ground_filtered = 0;
        size_t self_filtered = 0;

        for (size_t idx = 0; idx < npts; idx += stride) {
            const uint8_t* pt = data + idx * pstep;
            float fx, fy, fz;
            std::memcpy(&fx, pt + off_x, sizeof(float));
            std::memcpy(&fy, pt + off_y, sizeof(float));
            std::memcpy(&fz, pt + off_z, sizeof(float));

            // ---- 跳过无效点 ----
            if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(fz)) continue;

            double px = fx, py = fy, pz = fz;
            total_processed++;

            // ---- ROI 裁剪 (相机坐标系) ----
            // 为什么在相机坐标系做 ROI:
            //   点云原生就是相机坐标系, 不需要做坐标变换, 最快
            //   缺点是 ROI 跟相机安装位置绑定, 相机动了要重新设
            if (enable_roi_) {
                if (px < roi_x_min_ || px > roi_x_max_ ||
                    py < roi_y_min_ || py > roi_y_max_ ||
                    pz < roi_z_min_ || pz > roi_z_max_) {
                    roi_filtered++;
                    continue;
                }
            }

            // ---- 地面去除 (base_link 坐标系) ----
            // 在 base_link 坐标系下, Z=0 是底座安装面, ground_z_ 以下视为地面
            // 只需计算点在 base_link 系下的 Z 分量, 用 TF 逆变换的第三行即可
            if (enable_ground_removal_ && have_base_tf) {

                // 使用 TF 将点从相机系变换到 base_link 系 (仅 Z 分量)
                //
                // lookupTransform(target=cloud_frame, source=base_link) 返回:
                //   p_cloud = R * p_base + t  (将 base_link 点变换到 cloud_frame)
                // 逆变换: p_base = R^T * (p_cloud - t)
                // 我们只需要 p_base.z = R^T 第三行 . (p_cloud - t)
                //   R^T 第三行 = R 第三列 = [r02, r12, r22]
                //   r02 = 2(xz + wy),  r12 = 2(yz - wx),  r22 = 1 - 2(xx + yy)
                const auto& t = base_to_cam.transform.translation;
                const auto& q = base_to_cam.transform.rotation;

                double r02 = 2.0*(q.x*q.z + q.w*q.y);
                double r12 = 2.0*(q.y*q.z - q.w*q.x);
                double r22 = 1.0 - 2.0*(q.x*q.x + q.y*q.y);

                double base_z = r02*(px - t.x) + r12*(py - t.y) + r22*(pz - t.z);

                if (base_z < ground_z_) {
                    ground_filtered++;
                    continue;
                }
            }

            // ---- 自身过滤 ----
            // 在球体半径 + margin 内的点是机械臂自身, 跳过
            bool is_self = false;
            for (size_t i = 0; i < spheres.size(); ++i) {
                double dx = px - spheres[i].x;
                double dy = py - spheres[i].y;
                double dz = pz - spheres[i].z;
                double d2 = dx*dx + dy*dy + dz*dz;
                if (d2 < self_filter_r2_[i]) {
                    is_self = true;
                    break;
                }
            }
            if (is_self) {
                self_filtered++;
                continue;
            }

            // ---- 距离计算 ----
            // 对每个球体, 计算点到球面的距离 (= 点到球心的距离 - 半径)
            for (const auto& s : spheres) {
                double dx = px - s.x, dy = py - s.y, dz = pz - s.z;
                double ds = std::sqrt(dx*dx + dy*dy + dz*dz) - s.r;
                if (ds < result.min_distance) {
                    result.min_distance = ds;
                    result.nearest_link = s.name;
                    result.nearest_x = px;
                    result.nearest_y = py;
                    result.nearest_z = pz;
                    result.nearest_sphere_x = s.x;
                    result.nearest_sphere_y = s.y;
                    result.nearest_sphere_z = s.z;
                }
            }

            // 统计 caution zone 内的障碍物点数
            if (result.min_distance < thr_caution_) {
                result.obstacle_count++;
            }
        }

        // ---- Step 5: 填充输出消息 ----
        out.nearest_link = result.nearest_link;
        out.obstacle_points = result.obstacle_count;
        out.min_distance = (result.min_distance == std::numeric_limits<double>::max())
                           ? 999.0 : result.min_distance;

        auto fmt_dist = [](double d) {
            char b[32];
            std::snprintf(b, 32, "%.1fcm", d * 100);
            return std::string(b);
        };

        // ---- Step 6: 阈值判断 + 防抖 ----
        if (result.min_distance <= thr_emergency_) {
            emergency_counter_++;
            if (emergency_counter_ >= emergency_confirm_n_) {
                out.status = "emergency";
                out.message = "紧急! 距离 " + fmt_dist(result.min_distance)
                            + " @ " + result.nearest_link;
                try_pause();
                RCLCPP_ERROR(get_logger(), "%s", out.message.c_str());
            } else {
                // 还在防抖确认中, 报告为 warning
                out.status = "warning";
                out.message = "确认中 (" + std::to_string(emergency_counter_)
                            + "/" + std::to_string(emergency_confirm_n_)
                            + ") " + fmt_dist(result.min_distance)
                            + " @ " + result.nearest_link;
                RCLCPP_WARN(get_logger(), "%s", out.message.c_str());
            }
        } else {
            emergency_counter_ = 0;  // 脱离 emergency 区间, 重置计数
            if (result.min_distance <= thr_warning_) {
                out.status = "warning";
                out.message = "警告: " + fmt_dist(result.min_distance)
                            + " @ " + result.nearest_link;
                RCLCPP_WARN(get_logger(), "%s", out.message.c_str());
            } else if (result.min_distance <= thr_caution_) {
                out.status = "caution";
                out.message = "注意: " + fmt_dist(result.min_distance)
                            + " @ " + result.nearest_link;
            } else {
                out.status = "safe";
                out.message = "安全";
            }
        }

        pub_status_->publish(out);

        // ---- Step 7: 可视化 (独立于检测逻辑) ----
        if (enable_viz_) {
            publish_markers(cloud_frame, spheres, result, out.status);
        }

        // ---- 调试日志 (每100帧打印一次统计) ----
        frame_count_++;
        if (frame_count_ % 100 == 0) {
            RCLCPP_INFO(get_logger(),
                "统计 [帧#%lu]: 处理=%zu ROI过滤=%zu 地面过滤=%zu 自身过滤=%zu 障碍物点=%d 最近=%.3fm",
                frame_count_, total_processed, roi_filtered, ground_filtered,
                self_filtered, result.obstacle_count, out.min_distance);
        }
    }

    // ==========================================================================
    // 自动暂停
    // ==========================================================================
    void try_pause() {
        if (!auto_pause_) return;
        if (!client_pause_->service_is_ready()) {
            RCLCPP_WARN(get_logger(), "暂停服务不可用, 无法自动暂停!");
            return;
        }
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = true;
        client_pause_->async_send_request(req);
        RCLCPP_WARN(get_logger(), "已发送自动暂停请求");
    }

    // ==========================================================================
    // RViz 可视化
    // ==========================================================================
    void publish_markers(const std::string& frame,
                         const std::vector<Sphere>& spheres,
                         const DetectionResult& result,
                         const std::string& status)
    {
        visualization_msgs::msg::MarkerArray markers;
        auto stamp = now();
        int id = 0;

        // 颜色定义
        auto make_color = [](float r, float g, float b, float a) {
            std_msgs::msg::ColorRGBA c;
            c.r = r; c.g = g; c.b = b; c.a = a;
            return c;
        };

        // 根据状态选择球体颜色
        std_msgs::msg::ColorRGBA sphere_color;
        if (status == "emergency") {
            sphere_color = make_color(1.0, 0.0, 0.0, 0.4);  // 红色
        } else if (status == "warning") {
            sphere_color = make_color(1.0, 0.5, 0.0, 0.3);  // 橙色
        } else if (status == "caution") {
            sphere_color = make_color(1.0, 1.0, 0.0, 0.2);  // 黄色
        } else {
            sphere_color = make_color(0.0, 1.0, 0.0, 0.15); // 绿色
        }

        // ---- 球体模型 ----
        for (const auto& s : spheres) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = frame;
            m.header.stamp = stamp;
            m.ns = "collision_spheres";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = s.x;
            m.pose.position.y = s.y;
            m.pose.position.z = s.z;
            m.pose.orientation.w = 1.0;
            m.scale.x = m.scale.y = m.scale.z = s.r * 2.0;
            m.color = sphere_color;
            m.lifetime = rclcpp::Duration::from_seconds(0.3);
            markers.markers.push_back(m);
        }

        // ---- 最近点标记 ----
        if (result.min_distance < thr_caution_ &&
            result.min_distance < std::numeric_limits<double>::max())
        {
            // 最近障碍物点 (红色小球)
            visualization_msgs::msg::Marker nearest;
            nearest.header.frame_id = frame;
            nearest.header.stamp = stamp;
            nearest.ns = "nearest_point";
            nearest.id = id++;
            nearest.type = visualization_msgs::msg::Marker::SPHERE;
            nearest.action = visualization_msgs::msg::Marker::ADD;
            nearest.pose.position.x = result.nearest_x;
            nearest.pose.position.y = result.nearest_y;
            nearest.pose.position.z = result.nearest_z;
            nearest.pose.orientation.w = 1.0;
            nearest.scale.x = nearest.scale.y = nearest.scale.z = 0.03;
            nearest.color = make_color(1.0, 0.0, 0.0, 1.0);
            nearest.lifetime = rclcpp::Duration::from_seconds(0.3);
            markers.markers.push_back(nearest);

            // 连线: 最近点 → 最近球心
            visualization_msgs::msg::Marker line;
            line.header.frame_id = frame;
            line.header.stamp = stamp;
            line.ns = "nearest_line";
            line.id = id++;
            line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action = visualization_msgs::msg::Marker::ADD;
            line.scale.x = 0.005;  // 线宽 5mm
            line.color = make_color(1.0, 0.2, 0.2, 0.8);
            line.lifetime = rclcpp::Duration::from_seconds(0.3);
            line.pose.orientation.w = 1.0;

            geometry_msgs::msg::Point p1, p2;
            p1.x = result.nearest_x;
            p1.y = result.nearest_y;
            p1.z = result.nearest_z;
            p2.x = result.nearest_sphere_x;
            p2.y = result.nearest_sphere_y;
            p2.z = result.nearest_sphere_z;
            line.points.push_back(p1);
            line.points.push_back(p2);
            markers.markers.push_back(line);

            // 距离文本
            visualization_msgs::msg::Marker text;
            text.header.frame_id = frame;
            text.header.stamp = stamp;
            text.ns = "distance_text";
            text.id = id++;
            text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::msg::Marker::ADD;
            text.pose.position.x = (result.nearest_x + result.nearest_sphere_x) / 2;
            text.pose.position.y = (result.nearest_y + result.nearest_sphere_y) / 2;
            text.pose.position.z = (result.nearest_z + result.nearest_sphere_z) / 2 + 0.05;
            text.pose.orientation.w = 1.0;
            text.scale.z = 0.03;  // 文字高度 3cm
            char buf[64];
            std::snprintf(buf, 64, "%.1fcm", result.min_distance * 100);
            text.text = buf;
            text.color = make_color(1.0, 1.0, 1.0, 1.0);
            text.lifetime = rclcpp::Duration::from_seconds(0.3);
            markers.markers.push_back(text);
        }

        // ---- ROI 包围盒 ----
        if (enable_roi_) {
            visualization_msgs::msg::Marker roi;
            roi.header.frame_id = frame;
            roi.header.stamp = stamp;
            roi.ns = "roi_box";
            roi.id = id++;
            roi.type = visualization_msgs::msg::Marker::LINE_LIST;
            roi.action = visualization_msgs::msg::Marker::ADD;
            roi.scale.x = 0.003;  // 线宽 3mm
            roi.color = make_color(0.3, 0.7, 1.0, 0.4);
            roi.lifetime = rclcpp::Duration::from_seconds(0.5);
            roi.pose.orientation.w = 1.0;

            // 12条边组成线框盒
            double x0 = roi_x_min_, x1 = roi_x_max_;
            double y0 = roi_y_min_, y1 = roi_y_max_;
            double z0 = roi_z_min_, z1 = roi_z_max_;
            auto pt = [](double x, double y, double z) {
                geometry_msgs::msg::Point p;
                p.x = x; p.y = y; p.z = z;
                return p;
            };
            // 底面 4 条边
            roi.points.push_back(pt(x0,y0,z0)); roi.points.push_back(pt(x1,y0,z0));
            roi.points.push_back(pt(x1,y0,z0)); roi.points.push_back(pt(x1,y1,z0));
            roi.points.push_back(pt(x1,y1,z0)); roi.points.push_back(pt(x0,y1,z0));
            roi.points.push_back(pt(x0,y1,z0)); roi.points.push_back(pt(x0,y0,z0));
            // 顶面 4 条边
            roi.points.push_back(pt(x0,y0,z1)); roi.points.push_back(pt(x1,y0,z1));
            roi.points.push_back(pt(x1,y0,z1)); roi.points.push_back(pt(x1,y1,z1));
            roi.points.push_back(pt(x1,y1,z1)); roi.points.push_back(pt(x0,y1,z1));
            roi.points.push_back(pt(x0,y1,z1)); roi.points.push_back(pt(x0,y0,z1));
            // 竖直 4 条边
            roi.points.push_back(pt(x0,y0,z0)); roi.points.push_back(pt(x0,y0,z1));
            roi.points.push_back(pt(x1,y0,z0)); roi.points.push_back(pt(x1,y0,z1));
            roi.points.push_back(pt(x1,y1,z0)); roi.points.push_back(pt(x1,y1,z1));
            roi.points.push_back(pt(x0,y1,z0)); roi.points.push_back(pt(x0,y1,z1));
            markers.markers.push_back(roi);
        }

        pub_markers_->publish(markers);
    }

    // ==========================================================================
    // 成员变量
    // ==========================================================================

    // 参数
    std::string pc_topic_;
    double detection_hz_;
    int stride_;
    std::vector<std::string> link_names_;
    std::vector<double> link_radii_;
    double self_margin_;
    std::vector<double> self_filter_r2_;   // 预计算: (radius + margin)^2

    bool enable_roi_;
    double roi_x_min_, roi_x_max_;
    double roi_y_min_, roi_y_max_;
    double roi_z_min_, roi_z_max_;

    bool enable_ground_removal_;
    double ground_z_;

    double thr_emergency_, thr_warning_, thr_caution_;
    bool auto_pause_;
    int emergency_confirm_n_;
    bool enable_viz_;

    // 状态
    int emergency_counter_ = 0;
    unsigned long frame_count_ = 0;

    // ROS 对象
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    sensor_msgs::msg::PointCloud2::SharedPtr last_pc_;
    std::mutex pc_mutex_;
    std::atomic<bool> hw_collision_{false};

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_set_topic_;
    rclcpp::Subscription<duco_msg::msg::DucoRobotState>::SharedPtr sub_state_;
    rclcpp::Publisher<common_msgs::msg::CollisionStatus>::SharedPtr pub_status_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_pause_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// =============================================================================
// main
// =============================================================================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CollisionDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
