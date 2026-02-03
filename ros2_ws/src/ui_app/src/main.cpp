#include <QApplication>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include "ui_app/ros_node.hpp"
#include "ui_app/app_window.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RosNode>();
  
  // Executor in the main thread
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  QApplication app(argc, argv);
  AppWindow w(node);
  w.setWindowTitle("机器臂手上位机 -目前仅机器臂");
  w.resize(800, 600); // Increased size for tabs
  w.show();
  
  // Use QTimer to spin ROS 2 executor periodically in the main thread
  QTimer timer;
  QObject::connect(&timer, &QTimer::timeout, [&exec](){
    // spin_some() will process all available work and return immediately
    exec.spin_some();
  });
  // 10ms interval = 100Hz, should be enough for UI
  timer.start(10);

  int ret = app.exec();
  
  rclcpp::shutdown();
  
  return ret;
}
