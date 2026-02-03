工业上位机 ROS2 工作区

结构
- src 目录用于存放各 ROS2 包
- 使用 colcon build 在工作区根目录构建

环境准备（Ubuntu 22.04 / aarch64）
1. 修复软件源（如遇镜像 403）
   sudo cp /etc/apt/sources.list /etc/apt/sources.list.bak.$(date +%s)
   sudo bash -c 'cat > /etc/apt/sources.list <<EOF
   deb http://ports.ubuntu.com/ubuntu-ports jammy main restricted universe multiverse
   deb http://ports.ubuntu.com/ubuntu-ports jammy-updates main restricted universe multiverse
   deb http://ports.ubuntu.com/ubuntu-ports jammy-backports main restricted universe multiverse
   deb http://ports.ubuntu.com/ubuntu-ports jammy-security main restricted universe multiverse
   EOF'
   sudo rm -f /etc/apt/sources.list.d/aliyun*.list
2. 安装依赖
   sudo apt update
   sudo apt install -y cmake build-essential curl gnupg2 lsb-release qtbase5-dev
3. 添加 ROS2 Humble 源并安装
   sudo mkdir -p /etc/apt/keyrings
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg
   echo 'deb [signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main' | sudo tee /etc/apt/sources.list.d/ros2.list
   sudo apt update
   sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime ros-humble-sensor-msgs ros-humble-ament-cmake

构建
1. source /opt/ros/humble/setup.bash
2. cd ros2_ws && colcon build

运行
1. 记录节点
   source /opt/ros/humble/setup.bash
   cd ros2_ws
   export LD_LIBRARY_PATH=$PWD/install/common_msgs/lib:$PWD/install/logger_node/lib:$LD_LIBRARY_PATH
   ./install/logger_node/lib/logger_node/logger_node
   日志文件位于 ros2_ws/logs/session_*.csv
2. UI 心跳示例
   source /opt/ros/humble/setup.bash
   ./ros2_ws/install/ui_app/lib/ui_app/ui_app
   若出现 GL 报错，可设置软件渲染：
   export QT_OPENGL=software
3. 主题检查
   source /opt/ros/humble/setup.bash
   ros2 topic list
   timeout 3 ros2 topic echo /ui/heartbeat
