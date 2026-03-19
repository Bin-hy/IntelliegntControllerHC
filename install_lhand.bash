# https://zcnrg1e5hv0a.feishu.cn/wiki/KN1SwOqSxidq3dkxBuDcCLoinch
# 复制库文件到系统库目录
sudo cp ros2_ws/install/lhandpro_service/thirdparty/lib/libLHandProLib.so /usr/local/lib

# 更新系统库缓存
sudo ldconfig

# # 配置执行环境
# sudo cp ros2_ws/src/ros2_lhandpro.conf /etc/ld.so.conf.d/
# sudo ldconfig

# echo "配置文件已成功复制到 /etc/ld.so.conf.d/ros2_lhandpro.conf "


PROJECT_ROOT=$PWD

# 复制配置文件
sudo rm -rf /etc/ld.so.conf.d/ros2_lhandpro.conf
# 配置动态库路径
echo "$PROJECT_ROOT/ros2_ws/install/lhandpro_service/lib" >> /etc/ld.so.conf.d/ros2_lhandpro.conf
echo "$PROJECT_ROOT/ros2_ws/install/lhandpro_interfaces/lib" >> /etc/ld.so.conf.d/ros2_lhandpro.conf
echo "$PROJECT_ROOT/ros2_ws/install/lhandpro_description/lib" >> /etc/ld.so.conf.d/ros2_lhandpro.conf
echo "$PROJECT_ROOT/ros2_ws/install/sequence_demo_cpp/lib" >> /etc/ld.so.conf.d/ros2_lhandpro.conf
echo "$PROJECT_ROOT/ros2_ws/install/sequence_demo_py/lib" >> /etc/ld.so.conf.d/ros2_lhandpro.conf

sudo ldconfig

echo "配置文件已成功复制到 /etc/ld.so.conf.d/ros2_lhandpro.conf"
