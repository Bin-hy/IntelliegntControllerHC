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
ros2 launch ui_app unified.launch.py robot_ip:=192.168.1.10 ethercat_channel:=0 
```

## 宇叠动捕手套

```bash
sudo systemctl restart hdservice # 重启hd服务
sudo systemctl start hdservice  # 启动hd服务
sudo systemctl status hdservice # 查看hd服务状态
```

浏览器：访问http://localhost:8088/


## lift平台任务队列设置
运行运动到指定位置后停止
接收到指令回零

在断使能的时候设置参数
使能状态下不能设置脉冲和速度的



## 测试指令

DISPLAY=:1 ros2 launch ui_app unified.launch.py robot_ip:=192.168.1.10 ethercat_channel:=1 grasp_camera_ns:='/CV2R1610004H' enable_calibration:=true 2>&1 | tee /home/hwtws/IntelliegntControllerHC/log/calib_$(date +%m%d_%H%M).log