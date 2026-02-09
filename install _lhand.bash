# https://zcnrg1e5hv0a.feishu.cn/wiki/KN1SwOqSxidq3dkxBuDcCLoinch
# 复制库文件到系统库目录
sudo cp ros2_ws/install/lhandpro_service/thirdparty/lib/libLHandProLib.so /usr/local/lib

# 更新系统库缓存
sudo ldconfig

# 配置执行环境
sudo cp ros2_ws/src/ros2_lhandpro.conf /etc/ld.so.conf.d/
sudo ldconfig