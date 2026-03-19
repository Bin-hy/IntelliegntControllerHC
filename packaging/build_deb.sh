#!/bin/bash
# =============================================================================
# RobotApp Debian 打包脚本
# 用法：bash packaging/build_deb.sh [版本号]
#        例：bash packaging/build_deb.sh 1.0.0
# 输出：packaging/dist/robotapp_<版本>_amd64.deb
# =============================================================================
set -euo pipefail

# ------------ 可调整参数 -------------------------------------------------------
PACKAGE_NAME="robotapp"
VERSION="${1:-0.1.0}"
ARCH="amd64"
INSTALL_PREFIX="/opt/robotapp"                    # 安装目标根目录

# 路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
WS_DIR="${REPO_ROOT}/ros2_ws"
STAGING="${SCRIPT_DIR}/staging"                   # 临时打包目录（被 .gitignore）
DIST="${SCRIPT_DIR}/dist"                         # 输出目录
DEB_TEMPLATE="${SCRIPT_DIR}/deb_template"

# 第三方库源路径
LHAND_LIB_DIR="${WS_DIR}/src/lhandpro_service/thirdparty/lib"
ORBBEC_LIB_DIR="${WS_DIR}/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/x64"
ORBBEC_UDEV="${WS_DIR}/src/OrbbecSDK_ROS2/orbbec_camera/scripts/99-obsensor-libusb.rules"
# ------------ 结束可调整参数 ---------------------------------------------------

DEB_FILE="${DIST}/${PACKAGE_NAME}_${VERSION}_${ARCH}.deb"

echo "======================================================================"
echo " RobotApp 打包工具  v${VERSION}"
echo " 仓库根目录: ${REPO_ROOT}"
echo " 暂存目录:   ${STAGING}"
echo " 输出文件:   ${DEB_FILE}"
echo "======================================================================"

# ---- 0. 检查依赖工具 ---------------------------------------------------------
for cmd in dpkg-deb colcon setcap; do
    if ! command -v "$cmd" >/dev/null 2>&1; then
        echo "错误：缺少命令 '$cmd'。请先安装：" >&2
        case "$cmd" in
            dpkg-deb) echo "  sudo apt install dpkg-dev" ;;
            colcon)   echo "  sudo apt install python3-colcon-common-extensions" ;;
            setcap)   echo "  sudo apt install libcap2-bin" ;;
        esac
        exit 1
    fi
done

# ---- 1. 编译 Release 版本 ----------------------------------------------------
echo ""
echo "[1/6] 编译 ROS 2 工作空间（Release 模式）..."
source /opt/ros/jazzy/setup.bash
cd "${WS_DIR}"
colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --event-handlers console_cohesion+
echo "编译完成。"

# ---- 2. 清理并初始化暂存目录 -------------------------------------------------
echo ""
echo "[2/6] 初始化暂存目录..."
rm -rf "${STAGING}"
mkdir -p "${STAGING}"
mkdir -p "${DIST}"

# 拷贝 DEBIAN 控制文件模板
cp -r "${DEB_TEMPLATE}/DEBIAN" "${STAGING}/DEBIAN"
# 拷贝配置文件、启动脚本、桌面文件
cp -r "${DEB_TEMPLATE}/etc"        "${STAGING}/etc"
cp -r "${DEB_TEMPLATE}/usr"        "${STAGING}/usr"

# ---- 3. 安装 ROS 2 产物 ------------------------------------------------------
echo ""
echo "[3/6] 复制 ROS 2 安装产物..."
ROS2_STAGING="${STAGING}${INSTALL_PREFIX}/ros2"
mkdir -p "${ROS2_STAGING}"
# 拷贝整个 install 目录（包含所有包的 lib/share/bin）
cp -a "${WS_DIR}/install/." "${ROS2_STAGING}/"

# ---- 4. 捆绑私有第三方库 -----------------------------------------------------
echo ""
echo "[4/6] 捆绑私有第三方库..."
LIB_STAGING="${STAGING}${INSTALL_PREFIX}/lib"
mkdir -p "${LIB_STAGING}"
mkdir -p "${LIB_STAGING}/extensions/depthengine"

# DH116 灵巧手库
if [ -f "${LHAND_LIB_DIR}/libLHandProLib.so" ]; then
    cp "${LHAND_LIB_DIR}/libLHandProLib.so" "${LIB_STAGING}/"
    echo "  ✓ libLHandProLib.so"
else
    echo "警告：未找到 libLHandProLib.so，跳过。" >&2
fi

# Orbbec SDK 主库
if [ -f "${ORBBEC_LIB_DIR}/libOrbbecSDK.so.2.7.2" ]; then
    cp "${ORBBEC_LIB_DIR}/libOrbbecSDK.so.2.7.2" "${LIB_STAGING}/"
    # 创建 soname 符号链接（ldconfig 也会自动创建，但提前放入包内更可靠）
    (cd "${LIB_STAGING}" && ln -sf libOrbbecSDK.so.2.7.2 libOrbbecSDK.so.2)
    echo "  ✓ libOrbbecSDK.so.2.7.2"
else
    echo "警告：未找到 libOrbbecSDK.so.2.7.2，跳过。" >&2
fi

# Orbbec 深度引擎插件
if [ -f "${ORBBEC_LIB_DIR}/extensions/depthengine/libdepthengine.so.2.0" ]; then
    cp "${ORBBEC_LIB_DIR}/extensions/depthengine/libdepthengine.so.2.0" \
       "${LIB_STAGING}/extensions/depthengine/"
    echo "  ✓ libdepthengine.so.2.0"
else
    echo "警告：未找到 libdepthengine.so.2.0，跳过。" >&2
fi

# ---- 5. 安装 udev 规则及其他 share 文件 --------------------------------------
echo ""
echo "[5/6] 复制 udev 规则和共享文件..."
SHARE_STAGING="${STAGING}${INSTALL_PREFIX}/share"
mkdir -p "${SHARE_STAGING}"

if [ -f "${ORBBEC_UDEV}" ]; then
    cp "${ORBBEC_UDEV}" "${SHARE_STAGING}/"
    echo "  ✓ 99-obsensor-libusb.rules"
fi

# ---- 6. 计算已安装大小并构建 .deb -------------------------------------------
echo ""
echo "[6/6] 构建 .deb 包..."

# 更新 control 中的 Installed-Size（单位：KiB）
INSTALLED_KB=$(du -sk "${STAGING}" | cut -f1)
sed -i "s/^Installed-Size: PLACEHOLDER/Installed-Size: ${INSTALLED_KB}/" \
    "${STAGING}/DEBIAN/control"

# 更新 control 中的版本号
sed -i "s/^Version:.*/Version: ${VERSION}/" "${STAGING}/DEBIAN/control"

# 设置脚本权限
chmod 755 "${STAGING}/DEBIAN/postinst"
chmod 755 "${STAGING}/DEBIAN/prerm"
chmod 755 "${STAGING}/usr/bin/robotapp"

# 构建 deb
dpkg-deb --build --root-owner-group "${STAGING}" "${DEB_FILE}"

echo ""
echo "======================================================================"
echo " 打包完成！"
echo " 输出：${DEB_FILE}"
echo " 大小：$(du -sh "${DEB_FILE}" | cut -f1)"
echo ""
echo " 安装命令："
echo "   sudo dpkg -i ${DEB_FILE}"
echo "   sudo apt-get install -f   # 如有依赖缺失"
echo ""
echo " 验证安装："
echo "   dpkg -l robotapp"
echo "   robotapp --help 2>&1 | head -5"
echo "======================================================================"
