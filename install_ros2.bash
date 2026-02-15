# Set locale
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings


# Set up Sources
sudo apt install software-properties-common
sudo add-apt-repository universe

# 
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Install ROS 2
sudo apt update
sudo apt upgrade

# 安装 ROS 2 Jazzy Desktop 版本
sudo apt install ros-jazzy-desktop -y

# 安装 ROS 2 Jazzy 基础版本（可选）
sudo apt install ros-jazzy-ros-base -y

# 安装开发工具（可选）
sudo apt install ros-dev-tools -y
# Environment Setup
# 添加 ROS 2 Jazzy 环境变量到 ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# 使环境变量立即生效
source ~/.bashrc

