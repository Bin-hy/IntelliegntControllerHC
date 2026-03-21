#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include <filesystem>
#include <string>
#include <fstream>
#include <cmath>
#include "vision_server/srv/save_image.hpp"
#include "vision_server/srv/save_point_cloud.hpp"
#include "rclcpp/wait_for_message.hpp"

namespace fs = std::filesystem;

class ImageSaverNode : public rclcpp::Node {
public:
    ImageSaverNode() : Node("image_saver_node") {
        this->declare_parameter("save_dir", ".");
        save_dir_ = this->get_parameter("save_dir").as_string();

        try {
            if (!fs::exists(save_dir_)) {
                fs::create_directories(save_dir_);
                RCLCPP_INFO(this->get_logger(), "Created save directory: %s", save_dir_.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create directory: %s", e.what());
        }

        RCLCPP_INFO(this->get_logger(), "Image Saver ready. Save dir: %s", save_dir_.c_str());

        save_service_ = this->create_service<vision_server::srv::SaveImage>(
            "save_image",
            std::bind(&ImageSaverNode::handle_save_image, this,
                      std::placeholders::_1, std::placeholders::_2));

        save_pc_service_ = this->create_service<vision_server::srv::SavePointCloud>(
            "save_point_cloud",
            std::bind(&ImageSaverNode::handle_save_point_cloud, this,
                      std::placeholders::_1, std::placeholders::_2));
    }

private:
    void handle_save_image(
        const std::shared_ptr<vision_server::srv::SaveImage::Request> request,
        std::shared_ptr<vision_server::srv::SaveImage::Response> response)
    {
        sensor_msgs::msg::Image msg;
        bool received = rclcpp::wait_for_message(
            msg, this->shared_from_this(), request->topic_name, std::chrono::seconds(5));
        if (!received) {
            response->success = false;
            response->message = "No image received from " + request->topic_name;
            return;
        }
        cv_bridge::CvImagePtr cv_ptr;
        try {
            if (msg.encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
                msg.encoding == sensor_msgs::image_encodings::MONO16) {
                auto tmp = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
                cv::Mat normalized;
                cv::normalize(tmp->image, normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
                cv_ptr = std::make_shared<cv_bridge::CvImage>(tmp->header, "mono8", normalized);
            } else if (msg.encoding == sensor_msgs::image_encodings::MONO8) {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
            } else {
                cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            }
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("cv_bridge error: ") + e.what();
            return;
        }
        auto stamp = this->now().nanoseconds();
        std::string file_tag = request->file_tag.empty() ? "image" : request->file_tag;
        fs::path base_dir(save_dir_);
        fs::path tag_path(file_tag);
        fs::path target_dir = base_dir;
        std::string prefix;
        if (tag_path.has_parent_path()) {
            target_dir /= tag_path.parent_path();
            prefix = tag_path.filename().string();
        } else {
            prefix = tag_path.string();
        }
        if (prefix.empty()) {
            prefix = "image";
        }
        try {
            if (!fs::exists(target_dir)) {
                fs::create_directories(target_dir);
            }
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Failed to create directory: ") + e.what();
            return;
        }
        fs::path filename = target_dir / (prefix + "_" + std::to_string(stamp) + ".png");
        bool ok = cv::imwrite(filename.string(), cv_ptr->image);
        response->success = ok;
        response->message = ok ? filename.string() : ("Failed to write " + filename.string());
    }

    void handle_save_point_cloud(
        const std::shared_ptr<vision_server::srv::SavePointCloud::Request> request,
        std::shared_ptr<vision_server::srv::SavePointCloud::Response> response)
    {
        sensor_msgs::msg::PointCloud2 msg;
        bool received = rclcpp::wait_for_message(
            msg, this->shared_from_this(), request->topic_name, std::chrono::seconds(5));
        if (!received) {
            response->success = false;
            response->message = "No point cloud received from " + request->topic_name;
            return;
        }

        // Parse point cloud fields
        int x_offset = -1, y_offset = -1, z_offset = -1;
        for (const auto& field : msg.fields) {
            if (field.name == "x") x_offset = field.offset;
            else if (field.name == "y") y_offset = field.offset;
            else if (field.name == "z") z_offset = field.offset;
        }
        if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
            response->success = false;
            response->message = "Point cloud missing x/y/z fields";
            return;
        }

        // Compute centroid and collect valid points
        struct Point3D { float x, y, z; };
        std::vector<Point3D> valid_points;
        valid_points.reserve(msg.width * msg.height / 2);

        double sum_x = 0, sum_y = 0, sum_z = 0;
        int valid_count = 0;

        for (uint32_t row = 0; row < msg.height; ++row) {
            const uint8_t* row_data = msg.data.data() + row * msg.row_step;
            for (uint32_t col = 0; col < msg.width; ++col) {
                const uint8_t* point_data = row_data + col * msg.point_step;
                float x, y, z;
                memcpy(&x, point_data + x_offset, sizeof(float));
                memcpy(&y, point_data + y_offset, sizeof(float));
                memcpy(&z, point_data + z_offset, sizeof(float));

                if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
                    valid_points.push_back({x, y, z});
                    sum_x += x;
                    sum_y += y;
                    sum_z += z;
                    valid_count++;
                }
            }
        }

        if (valid_count == 0) {
            response->success = false;
            response->message = "No valid points in point cloud";
            return;
        }

        response->centroid_x = sum_x / valid_count;
        response->centroid_y = sum_y / valid_count;
        response->centroid_z = sum_z / valid_count;
        response->num_points = valid_count;

        // Build save path
        auto stamp = this->now().nanoseconds();
        std::string file_tag = request->file_tag.empty() ? "pointcloud" : request->file_tag;
        fs::path base_dir(save_dir_);
        fs::path tag_path(file_tag);
        fs::path target_dir = base_dir;
        std::string prefix;
        if (tag_path.has_parent_path()) {
            target_dir /= tag_path.parent_path();
            prefix = tag_path.filename().string();
        } else {
            prefix = tag_path.string();
        }
        if (prefix.empty()) prefix = "pointcloud";

        try {
            if (!fs::exists(target_dir)) {
                fs::create_directories(target_dir);
            }
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Failed to create directory: ") + e.what();
            return;
        }

        // Save PLY file
        fs::path ply_file = target_dir / (prefix + "_" + std::to_string(stamp) + ".ply");
        {
            std::ofstream ofs(ply_file.string());
            if (!ofs.is_open()) {
                response->success = false;
                response->message = "Failed to open file: " + ply_file.string();
                return;
            }
            ofs << "ply\n";
            ofs << "format ascii 1.0\n";
            ofs << "element vertex " << valid_count << "\n";
            ofs << "property float x\n";
            ofs << "property float y\n";
            ofs << "property float z\n";
            ofs << "end_header\n";
            for (const auto& p : valid_points) {
                ofs << p.x << " " << p.y << " " << p.z << "\n";
            }
            ofs.close();
        }

        // Save position CSV alongside PLY
        fs::path csv_file = target_dir / (prefix + "_position_" + std::to_string(stamp) + ".csv");
        {
            std::ofstream ofs(csv_file.string());
            if (ofs.is_open()) {
                ofs << "centroid_x,centroid_y,centroid_z,num_points\n";
                ofs << response->centroid_x << ","
                    << response->centroid_y << ","
                    << response->centroid_z << ","
                    << valid_count << "\n";
                ofs.close();
            }
        }

        response->success = true;
        response->file_path = ply_file.string();
        response->message = "Saved " + std::to_string(valid_count) + " points. Centroid: ("
            + std::to_string(response->centroid_x) + ", "
            + std::to_string(response->centroid_y) + ", "
            + std::to_string(response->centroid_z) + ")";
    }

    std::string save_dir_;
    rclcpp::Service<vision_server::srv::SaveImage>::SharedPtr save_service_;
    rclcpp::Service<vision_server::srv::SavePointCloud>::SharedPtr save_pc_service_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSaverNode>());
    rclcpp::shutdown();
    return 0;
}
