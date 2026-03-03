#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include <filesystem>
#include <string>
#include <vector>
#include <map>
#include <chrono>
#include "std_srvs/srv/trigger.hpp"
#include "common_msgs/msg/device_status.hpp"
#include "vision_server/srv/save_image.hpp"
#include "rclcpp/wait_for_message.hpp"

namespace fs = std::filesystem;

class ImageSaverNode : public rclcpp::Node {
public:
    ImageSaverNode() : Node("image_saver_node") {
        // Declare and get the save_dir parameter
        this->declare_parameter("save_dir", ".");
        save_dir_ = this->get_parameter("save_dir").as_string();

        RCLCPP_INFO(this->get_logger(), "Image Saver initialized. Save Directory: %s", save_dir_.c_str());

        // Create directory if it doesn't exist
        try {
            if (!fs::exists(save_dir_)) {
                fs::create_directories(save_dir_);
                RCLCPP_INFO(this->get_logger(), "Created save directory: %s", save_dir_.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create directory: %s", e.what());
        }

        // Subscribe to common topics (example pattern)
        // In a real system, we might discover topics dynamically or use a config
        // For now, let's subscribe to a wildcard or known camera topics if possible, 
        // but ROS2 doesn't support wildcard subscriptions easily without a discovery loop.
        // Assuming we receive a command to save or subscribe to specific topics.
        
        status_pub_ = this->create_publisher<common_msgs::msg::DeviceStatus>("/system/device_status", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ImageSaverNode::publish_status, this));
        save_service_ = this->create_service<vision_server::srv::SaveImage>(
            "save_image",
            std::bind(&ImageSaverNode::handle_save_image, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void handle_save_image(
        const std::shared_ptr<vision_server::srv::SaveImage::Request> request,
        std::shared_ptr<vision_server::srv::SaveImage::Response> response) {
        sensor_msgs::msg::Image msg;
        bool received = rclcpp::wait_for_message(
            msg, this->shared_from_this(), request->topic_name, std::chrono::seconds(2));
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
        std::string filename = save_dir_ + "/" + file_tag + "_" + std::to_string(stamp) + ".png";
        bool ok = cv::imwrite(filename, cv_ptr->image);
        response->success = ok;
        response->message = ok ? filename : ("Failed to write " + filename);
    }
    void publish_status() {
        auto msg = common_msgs::msg::DeviceStatus();
        msg.device_name = "vision_server";
        msg.device_type = "vision_system";
        msg.status = "ready";
        status_pub_->publish(msg);
    }

    std::string save_dir_;
    rclcpp::Publisher<common_msgs::msg::DeviceStatus>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<vision_server::srv::SaveImage>::SharedPtr save_service_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSaverNode>());
    rclcpp::shutdown();
    return 0;
}
