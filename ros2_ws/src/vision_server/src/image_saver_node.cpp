#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include <filesystem>
#include <string>
#include <vector>
#include <map>
#include "std_srvs/srv/trigger.hpp"
#include "common_msgs/msg/device_status.hpp"

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
        
        // For demonstration, let's just publish status
        status_pub_ = this->create_publisher<common_msgs::msg::DeviceStatus>("device_status", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ImageSaverNode::publish_status, this));
        
        // Example: Subscribe to a specific camera topic if known
        // sub_ = this->create_subscription<sensor_msgs::msg::Image>(...);
    }

private:
    void publish_status() {
        auto msg = common_msgs::msg::DeviceStatus();
        msg.device_name = "vision_server";
        msg.device_type = "vision_system";
        msg.status = "Running. Save Dir: " + save_dir_;
        status_pub_->publish(msg);
    }

    std::string save_dir_;
    rclcpp::Publisher<common_msgs::msg::DeviceStatus>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSaverNode>());
    rclcpp::shutdown();
    return 0;
}
