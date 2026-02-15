#  初次clone安装依赖
```bash
#  直接执行install.bash脚本 就不用执行下面的了
sudo ./install.bash

# --- 或者分步骤执行install.bash脚本中的每个步骤 ---
# 安装 ros2 的jazzy环境
sudo ./install_ros2.bash
# 使用 rosdep 管理CMake依赖
sudo ./install_rosdep.sh
# 配置orbbec环境
sudo ./install_orbbec_env.bash
sudo ./install_lhand.bash
# 安装应用级依赖
sudo ./install_ui_app.bash
```

# 启动所有
```bash
cd ~/IntelliegntControllerHC/ros2_ws
source install/setup.bash
# 指定机器臂的ip和机器手的网卡id
ros2 launch ui_app unified.launch.py robot_ip:=192.168.192.10 ethercat_channel:=0 
```
