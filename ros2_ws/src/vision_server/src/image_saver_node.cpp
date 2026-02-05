#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "vision_server/srv/save_image.hpp"
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <future>

class ImageSaverNode : public rclcpp::Node
{
public:
    ImageSaverNode() : Node("image_saver_node")
    {
        // Parameters
        this->declare_parameter<std::string>("save_dir", "/home/user/IntelliegntControllerHC/photos");
        this->get_parameter("save_dir", save_dir_);

        // Ensure base save directory exists
        if (!std::filesystem::exists(save_dir_)) {
            std::filesystem::create_directories(save_dir_);
        }

        // Service
        service_ = this->create_service<vision_server::srv::SaveImage>(
            "save_image", std::bind(&ImageSaverNode::save_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Image Saver Node started.");
        RCLCPP_INFO(this->get_logger(), "Base Save Dir: %s", save_dir_.c_str());
    }

private:
    void save_callback(const std::shared_ptr<vision_server::srv::SaveImage::Request> request,
                       std::shared_ptr<vision_server::srv::SaveImage::Response> response)
    {
        std::string topic = request->topic_name;
        std::string tag = request->file_tag;

        RCLCPP_INFO(this->get_logger(), "Request to save topic: %s with tag: %s", topic.c_str(), tag.c_str());

        // Create a promise to store the received image
        auto promise = std::make_shared<std::promise<sensor_msgs::msg::Image::SharedPtr>>();
        auto future = promise->get_future();

        // Create a temporary subscription
        // We use a unique callback group to ensure it can run in parallel if using MultiThreadedExecutor?
        // Actually, since we are inside a service callback, we are occupying a thread.
        // We need the subscription callback to run on another thread.
        // If we use MultiThreadedExecutor, this should be fine if we have enough threads.
        
        rclcpp::SubscriptionOptions options;
        // options.callback_group = ...; // Default group is fine if we have threads.

        auto sub = this->create_subscription<sensor_msgs::msg::Image>(
            topic, 
            10, 
            [promise](const sensor_msgs::msg::Image::SharedPtr msg) {
                try {
                    promise->set_value(msg);
                } catch (...) {
                    // Promise might be already set if multiple messages come quickly
                }
            }
        );

        // Wait for image (timeout 3 seconds)
        if (future.wait_for(std::chrono::seconds(3)) == std::future_status::timeout) {
            response->success = false;
            response->message = "Timeout waiting for image on topic: " + topic;
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return; // sub will be destroyed here
        }

        auto msg = future.get();

        // Process and Save
        try {
            cv_bridge::CvImagePtr cv_ptr;
            // Handle different encodings
            if (msg->encoding == "16UC1") {
                 // Depth Image
                 cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            } else {
                 // Default to BGR8
                 // Check if it is rgb8 or bgr8
                 if (msg->encoding == "rgb8") {
                     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
                 } else {
                     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                 }
            }

            // Generate Path
            auto now = std::chrono::system_clock::now();
            auto in_time_t = std::chrono::system_clock::to_time_t(now);
            std::tm tm_struct = *std::localtime(&in_time_t);

            // Folder: YYYY-MM-DD
            std::stringstream ss_date;
            ss_date << std::put_time(&tm_struct, "%Y-%m-%d");
            std::string date_folder = save_dir_ + "/" + ss_date.str();

            if (!std::filesystem::exists(date_folder)) {
                std::filesystem::create_directories(date_folder);
            }

            // File: YYYYMMDD-HHMMSS-Type.jpg
            std::stringstream ss_file;
            ss_file << std::put_time(&tm_struct, "%Y%m%d-%H%M%S") << "-" << tag << ".jpg";
            std::string filepath = date_folder + "/" + ss_file.str();

            // Save
            // If depth, maybe save as PNG to preserve 16bit? Or normalize?
            // User asked for "image storage", usually for viewing.
            // But raw depth is better as PNG 16bit.
            // If we save as JPG, it will be lossy.
            // Let's use PNG for depth, JPG for color?
            // User said ".jpg" in prompt? "年月日-时间戳-照片类型" (didn't specify extension, but I used jpg before).
            // Let's stick to what works best. 16UC1 cannot be saved as standard JPG easily without conversion.
            // I will save as png for depth, jpg for color.
            
            bool save_success = false;
            if (msg->encoding == "16UC1") {
                // Save as PNG 16bit
                if (filepath.substr(filepath.length() - 4) == ".jpg") {
                     filepath.replace(filepath.length() - 4, 4, ".png");
                }
                save_success = cv::imwrite(filepath, cv_ptr->image);
            } else {
                save_success = cv::imwrite(filepath, cv_ptr->image);
            }

            if (save_success) {
                response->success = true;
                response->message = "Saved: " + filepath;
                RCLCPP_INFO(this->get_logger(), "Saved: %s", filepath.c_str());
            } else {
                response->success = false;
                response->message = "cv::imwrite failed";
            }

        } catch (const cv_bridge::Exception& e) {
            response->success = false;
            response->message = std::string("cv_bridge exception: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }

    rclcpp::Service<vision_server::srv::SaveImage>::SharedPtr service_;
    std::string save_dir_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSaverNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
