# RobotApp 打包指南

本文档说明如何将 RobotApp（机器臂手上位机）打包为可分发的 Debian `.deb` 软件包。

---

## 目录

1. [目录结构说明](#1-目录结构说明)
2. [前置要求](#2-前置要求)
3. [一键打包流程](#3-一键打包流程)
4. [修改代码后重新打包](#4-修改代码后重新打包)
5. [安装与验证](#5-安装与验证)
6. [卸载](#6-卸载)
7. [各文件说明](#7-各文件说明)
8. [常见问题](#8-常见问题)

---

## 1. 目录结构说明

```
packaging/
├── build_deb.sh                  ← 主打包脚本（唯一需要运行的脚本）
├── PACKAGING_GUIDE.md            ← 本文档
├── .gitignore                    ← 排除 staging/ dist/ 目录
└── deb_template/                 ← 包结构模板（复制进暂存目录）
    ├── DEBIAN/
    │   ├── control               ← 包元数据（名称、版本、依赖）
    │   ├── postinst              ← 安装后脚本（setcap、ldconfig、udev）
    │   ├── prerm                 ← 卸载前脚本（清理 udev 规则）
    │   └── conffiles             ← 声明用户配置文件（升级时保留）
    ├── etc/
    │   └── robotapp/
    │       └── robot.conf        ← 默认硬件配置（用户可修改）
    └── usr/
        ├── bin/
        │   └── robotapp          ← 启动脚本（读取 robot.conf 并调 launch）
        └── share/
            └── applications/
                └── robotapp.desktop  ← 桌面图标/菜单入口
```

打包过程还会自动生成以下目录（被 `.gitignore` 排除，不提交）：

```
packaging/
├── staging/    ← 临时打包目录，打包完成后可删除
└── dist/       ← 输出 .deb 文件的目录
```

---

## 2. 前置要求

在打包机器上安装以下工具：

```bash
# 必须
sudo apt install dpkg-dev libcap2-bin python3-colcon-common-extensions

# ROS 2 Jazzy（已安装则跳过）
# 参考 install_ros2.bash
```

确认 ROS 2 环境可用：

```bash
source /opt/ros/jazzy/setup.bash
ros2 --version   # 应输出 ros2 jazzy 版本
```

---

## 3. 一键打包流程

```bash
cd ~/IntelliegntControllerHC

# 打包（使用默认版本号 0.1.0）
bash packaging/build_deb.sh

# 或指定版本号
bash packaging/build_deb.sh 1.2.0
```

脚本自动完成：

| 步骤 | 内容 |
|------|------|
| 1 | 检查依赖工具 |
| 2 | `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release` 编译整个工作空间 |
| 3 | 将 `ros2_ws/install/` 复制到暂存目录 |
| 4 | 捆绑 `libLHandProLib.so`、`libOrbbecSDK.so.2.7.2`、`libdepthengine.so.2.0` |
| 5 | 复制 Orbbec udev 规则 |
| 6 | 更新 `control` 文件中的版本号和大小，调用 `dpkg-deb` 生成 `.deb` |

成功后输出：

```
packaging/dist/robotapp_0.1.0_amd64.deb
```

---

## 4. 修改代码后重新打包

代码修改后只需重新运行打包脚本，脚本会自动重新编译：

```bash
# 修改代码（例如修改 ui_app/src/app_window.cpp）
# ... 编辑文件 ...

# 重新打包（版本号按需更新）
bash packaging/build_deb.sh 1.2.1
```

### 4.1 只修改了某个包时加速编译

如果只修改了 `ui_app`，可以在脚本外手动编译再打包（修改 `build_deb.sh` 中步骤 1 可选实现）：

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ui_app --cmake-args -DCMAKE_BUILD_TYPE=Release
cd ..
bash packaging/build_deb.sh 1.2.1
```

### 4.2 修改版本号

版本号在打包脚本调用时通过参数指定，会自动写入 `DEBIAN/control`。无需手动编辑 `control` 文件中的版本字段。

### 4.3 修改运行时依赖

编辑 `packaging/deb_template/DEBIAN/control` 中的 `Depends:` 字段，然后重新运行打包脚本。

---

## 5. 安装与验证

### 5.1 在目标机器上安装

```bash
# 安装 .deb 包
sudo dpkg -i packaging/dist/robotapp_0.1.0_amd64.deb

# 如果有依赖缺失，自动补全
sudo apt-get install -f
```

### 5.2 配置硬件参数

安装后编辑硬件配置文件：

```bash
sudo nano /etc/robotapp/robot.conf
```

配置项说明：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ROBOT_IP` | `192.168.1.10` | DUCO 机械臂 IP |
| `ETHERCAT_CHANNEL` | `0` | EtherCAT 网卡索引（第 n 块网卡） |
| `MODEL` | `gcr5_910` | 机械臂型号 |
| `CAMERA1_SERIAL` | 空 | Orbbec 相机序列号（空=自动选择） |
| `CAMERA2_SERIAL` | 空 | 第二台相机序列号 |

### 5.3 验证安装

```bash
# 查看包信息
dpkg -l robotapp

# 检查 EtherCAT 权限
getcap /opt/robotapp/ros2/lib/lhandpro_service/lhandpro_service
# 应输出：.../lhandpro_service = cap_net_raw+ep

# 检查库路径
cat /etc/ld.so.conf.d/robotapp.conf
# 应有两行：/opt/robotapp/lib 和 /opt/robotapp/lib/extensions/depthengine

# 检查 udev 规则
ls /etc/udev/rules.d/99-obsensor-libusb.rules
```

### 5.4 启动应用

```bash
# 命令行直接运行
robotapp

# 或从桌面菜单搜索 "RobotApp" 启动

# 首次运行会在终端打印 admin 初始密码，请保存：
# ========================================
#   FIRST-TIME SETUP
#   Default admin password: Xxxxx1@yyy
#   Please change it after first login.
# ========================================
```

### 5.5 注意：Orbbec 相机 USB 权限

udev 规则在**重新登录或重启后**生效。如果相机无法识别，执行：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
# 然后重新插拔 USB 相机
```

---

## 6. 卸载

```bash
sudo apt remove robotapp
# 保留 /etc/robotapp/robot.conf（用户配置）

sudo apt purge robotapp
# 完全删除，包括配置文件
```

---

## 7. 各文件说明

### `build_deb.sh`

主打包脚本，完成编译→暂存→捆绑库→生成 .deb 的全流程。

关键变量（脚本顶部可调整）：

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `PACKAGE_NAME` | `robotapp` | 包名 |
| `VERSION` | `0.1.0`（可通过参数覆盖） | 版本号 |
| `INSTALL_PREFIX` | `/opt/robotapp` | 安装根目录 |

### `deb_template/DEBIAN/control`

包元数据。需要关注的字段：

- `Version`：由 `build_deb.sh` 自动替换，无需手动改
- `Depends`：列出所有运行时依赖。如果目标系统的 OpenCV/Qt 版本不同，需要在这里调整版本约束
- `Installed-Size`：由脚本自动计算填写

### `deb_template/DEBIAN/postinst`

安装后自动执行，完成：

1. **`ldconfig`**：让系统找到 `/opt/robotapp/lib` 下的私有库
2. **`setcap`**：为 `lhandpro_service` 授予 EtherCAT RAW socket 权限
3. **udev 规则**：复制 Orbbec 规则到 `/etc/udev/rules.d/` 并重载
4. **桌面数据库更新**：使应用图标出现在菜单中

> **为什么不用 RPATH 而用 ldconfig？**
> `setcap` 出于安全考虑会忽略 RPATH，因此带有 `cap_net_raw` 能力的可执行文件
> 必须通过系统库路径（`LD_LIBRARY_PATH` 或 `ldconfig`）找到依赖库。

### `deb_template/usr/bin/robotapp`

Shell 启动脚本，负责：

1. 读取 `/etc/robotapp/robot.conf`
2. `source` ROS 2 环境和工作空间的 `setup.bash`
3. 设置 `LD_LIBRARY_PATH` 包含私有库路径
4. 组装并执行 `ros2 launch ui_app unified.launch.py ...`

### `deb_template/etc/robotapp/robot.conf`

硬件配置文件。已在 `conffiles` 中声明，因此：
- 升级包时不会覆盖用户已修改的配置
- `apt purge` 才会删除（`apt remove` 保留）

---

## 8. 常见问题

### Q: 打包时报 `E: failed to open for writing`

```bash
# 检查 staging/ 目录权限
ls -la packaging/staging/
# 如果是 root 创建的，删除后重试
sudo rm -rf packaging/staging/
bash packaging/build_deb.sh
```

### Q: 安装后运行报 `cannot open shared object file: libLHandProLib.so`

```bash
# 确认 postinst 正确执行了 ldconfig
sudo ldconfig -v 2>&1 | grep LHand

# 手动修复
echo "/opt/robotapp/lib" | sudo tee /etc/ld.so.conf.d/robotapp.conf
sudo ldconfig
```

### Q: lhandpro_service 报 `Operation not permitted`（EtherCAT 权限）

```bash
# 检查是否有 cap_net_raw
getcap /opt/robotapp/ros2/lib/lhandpro_service/lhandpro_service

# 手动授权
sudo setcap cap_net_raw=ep /opt/robotapp/ros2/lib/lhandpro_service/lhandpro_service
```

### Q: Orbbec 相机不识别 / 找不到设备

```bash
# 重载 udev 规则
sudo udevadm control --reload-rules && sudo udevadm trigger

# 检查规则是否存在
ls -l /etc/udev/rules.d/99-obsensor-libusb.rules

# 检查 USB 设备
lsusb | grep -i orbbec
```

### Q: 3D 视图黑屏

```bash
# 尝试软件渲染
export QT_OPENGL=software
robotapp
```

### Q: 在新机器上首次登录后没有看到初始密码

初始密码仅在**用户数据文件不存在时**打印，即第一次启动。数据文件位于：

```bash
# 通常在用户家目录的配置路径
~/.config/RobotApp/users.json   # 或类似路径（取决于 QSettings 存储位置）
```

如果忘记密码，可以删除该文件后重启应用，系统会重新生成随机密码。

### Q: 更改代码后重新打包，版本号如何管理

建议遵循语义化版本（SemVer）：

- 修复 bug：`0.1.0` → `0.1.1`
- 新功能：`0.1.0` → `0.2.0`
- 重大变更：`0.1.0` → `1.0.0`

```bash
bash packaging/build_deb.sh 0.1.1
```
