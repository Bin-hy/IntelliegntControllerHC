# 编译环境
- Ubuntu 22.04
- ROS2 Humble

# 示例项目结构
```
lhandpro_ws# 项目工作空间
└── src # 源码目录
        ├── lhandpro_description # 包含urdf及mesh文件

	# 监控实时角度并发布/jonit_states节点文件
	# rviz2、robot_state_publisher、lhandpro_state_publisher的启动脚本
        ├── lhandpro_interfaces# 定义服务接口
        ├── lhandpro_service# 服务运行节点, 映射服务通过api操控灵巧手
        ├── sequence_demo_cpp# 通过服务执行灵巧手动作序列的cpp示例
        └── sequence_demo_py# 通过服务执行灵巧手动作序列的python示例
        └── ros2_lhandpro.conf# 预准备的配置文件
```

# 编译条件
1. lhandpro_ws ROS示例工程的编译
```bash
tar -xzf lhandpro_ws_src.tar.gz -C ./lhandpro_ws # 解压
cd lhandpro_ws
colcon build
source install/setup.bash # 不出意外全部都能编译成功
```

2. 配置环境
```bash
# 复制so到系统库目录,方便后续使用
sudo cp src/lhandpro_service/thirdparty/lib/libLHandProLib.so/usr/local/lib/ # 刷新动态链接器缓存
sudo ldconfig # 赋予原始套接字 + 网络管理权限
sudo setcap cap_net_raw,cap_net_admin+ep
./install/lhandpro_service/lib/lhandpro_service/lhandpro_service
```
打开lhandpro_ws/src/ros2_lhandpro.conf, 编辑conf文件

```bash
# ros2_lhandpro.conf 文件内容, 修改为本机的实际路径
/home/plf/Project/RosProject/lhandpro_ws/install/lhandpro_service/lib
/home/plf/Project/RosProject/lhandpro_ws/install/lhandpro_interfaces/lib
/home/plf/Project/RosProject/lhandpro_ws/install/lhandpro_description/lib
/home/plf/Project/RosProject/lhandpro_ws/install/sequence_demo_cpp/lib
/home/plf/Project/RosProject/lhandpro_ws/install/sequence_demo_py/lib
```

配置执行环境
```bash
# 复制conf文件到系统目录
sudo cp src/ros2_lhandpro.conf /etc/ld.so.conf.d/
# 刷新动态链接器缓存
sudo ldconfig
```

3. 执行控制服务
```bash
ros2 launch lhandpro_service lhandpro.launch.py
# 如果配置没出错, 那么正常控制台会输出网口列表
# 并自动连接代码中所指定的默认网口, 请和实际EtherCAT使用网口对应
```

4. 执行示例控制
```bash
# 启动后则会自动执行准备好的动作序列
ros2 launch sequence_demo_py sequence.launch.py # 或者执行C++版本的示例程序
ros2 launch sequence_demo_cpp sequence.launch.py
```

5. 执行rviz2的仿真显示
```bash
ros2 launch lhandpro_description display_lhandpro.launch.py 
# 执行该脚本, 会启动robot_state_publisher节点
# 启动lhandpro_state_publisher节点, 用来通过服务监控角度变化, 并通过/joint_states发布给rviz2更新显示# 启动rviz2节点, 并加载准备好的rviz配置config
# 加载完成顺利的话则能在rviz2中看到灵巧手的模型
# 此时执行4步骤中的控制示例, 则能看到实际灵巧手在运动, 并且rviz2中的模型也在同时运动
```