#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>

class ImageSaverNode : public rclcpp::Node
{
public:
    ImageSaverNode() : Node("image_saver_node")
    {
        // Parameters
        this->declare_parameter<std::string>("image_topic", "/camera/color/image_raw");
        this->declare_parameter<std::string>("save_dir", "/home/user/IntelliegntControllerHC/photos");
        this->declare_parameter<std::string>("service_name", "save_image");

        std::string image_topic;
        this->get_parameter("image_topic", image_topic);
        this->get_parameter("save_dir", save_dir_);
        std::string service_name;
        this->get_parameter("service_name", service_name);

        // Ensure save directory exists
        if (!std::filesystem::exists(save_dir_)) {
            std::filesystem::create_directories(save_dir_);
        }

        // Subscriber
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic, 10, std::bind(&ImageSaverNode::image_callback, this, std::placeholders::_1));

        // Service
        service_ = this->create_service<std_srvs::srv::Trigger>(
            service_name, std::bind(&ImageSaverNode::save_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Image Saver Node started.");
        RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", image_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Saving to: %s", save_dir_.c_str());
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        last_image_ = msg;
    }

    void save_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!last_image_) {
            response->success = false;
            response->message = "No image received yet.";
            RCLCPP_WARN(this->get_logger(), "Save requested, but no image received.");
            return;
        }

        try {
            // Convert ROS image to OpenCV image
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(last_image_, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                response->success = false;
                response->message = "Failed to convert image.";
                return;
            }

            // Generate filename with timestamp
            auto now = std::chrono::system_clock::now();
            auto in_time_t = std::chrono::system_clock::to_time_t(now);

            std::stringstream ss;
            ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
            std::string filename = save_dir_ + "/photo_" + ss.str() + ".jpg";

            // Save image
            if (cv::imwrite(filename, cv_ptr->image)) {
                response->success = true;
                response->message = "Saved to " + filename;
                RCLCPP_INFO(this->get_logger(), "Image saved: %s", filename.c_str());
            } else {
                response->success = false;
                response->message = "Failed to write file.";
                RCLCPP_ERROR(this->get_logger(), "Failed to write image to %s", filename.c_str());
            }

        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Exception: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "Exception during save: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
    sensor_msgs::msg::Image::SharedPtr last_image_;
    std::string save_dir_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSaverNode>());
    rclcpp::shutdown();
    return 0;
}
