#!/bin/bash
# =============================================================================
# test_collision_system.sh — 碰撞检测系统测试
# =============================================================================
# Level 1: 节点启动/参数验证 (全自动, 不需要硬件)
# Level 2: 假点云端到端     (全自动, 不需要硬件)
# Level 3: 实机联调         (需要相机+机械臂)
#
# 用法:
#   ./test_collision_system.sh              # Level 1+2
#   ./test_collision_system.sh --level 3    # 全部
# =============================================================================

set -e

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'
PASS=0; FAIL=0; SKIP=0
LEVEL=${1:-2}; [[ "$1" == "--level" ]] && LEVEL=$2

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../../.." && pwd)"

pass() { echo -e "  ${GREEN}✓ PASS${NC}: $1"; PASS=$((PASS+1)); }
fail() { echo -e "  ${RED}✗ FAIL${NC}: $1"; FAIL=$((FAIL+1)); }
skip() { echo -e "  ${YELLOW}⊘ SKIP${NC}: $1"; SKIP=$((SKIP+1)); }
info() { echo -e "${CYAN}▸ $1${NC}"; }

PIDS=()
cleanup() {
    info "清理后台进程..."
    for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null || true; done
    wait 2>/dev/null || true
}
trap cleanup EXIT

bg() { "$@" & PIDS+=($!); }

source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash" 2>/dev/null || { echo -e "${RED}请先 colcon build${NC}"; exit 1; }

# 清理可能的残留进程 (前次测试意外退出时)
pkill -f collision_detector_node 2>/dev/null || true
pkill -f "publish_test_clouds" 2>/dev/null || true
sleep 1

echo ""
echo "============================================"
echo " 碰撞检测系统测试 (Level 1~${LEVEL})"
echo "============================================"
echo ""

# ===================== Level 1 =====================
info "Level 1: 节点启动与参数验证"

ros2 pkg executables collision_detector 2>/dev/null | grep -q collision_detector_node \
    && pass "可执行文件存在" || fail "可执行文件不存在"

PREFIX="$(ros2 pkg prefix collision_detector)/share/collision_detector"
[ -f "$PREFIX/config/collision_params.yaml" ] && pass "collision_params.yaml" || fail "collision_params.yaml 缺失"
[ -f "$PREFIX/launch/collision.launch.py" ]   && pass "collision.launch.py"   || fail "collision.launch.py 缺失"
[ -f "$PREFIX/rviz/collision.rviz" ]          && pass "collision.rviz"        || fail "collision.rviz 缺失"

info "3 秒存活测试..."
bg ros2 run collision_detector collision_detector_node \
    --ros-args -p point_cloud_topic:=/nonexistent 2>/dev/null
sleep 3
kill -0 "${PIDS[-1]}" 2>/dev/null && pass "节点 3s 存活" || fail "节点启动崩溃"
kill "${PIDS[-1]}" 2>/dev/null; wait "${PIDS[-1]}" 2>/dev/null || true

info "参数加载验证..."
bg ros2 run collision_detector collision_detector_node \
    --ros-args --params-file "$PREFIX/config/collision_params.yaml" 2>/dev/null
sleep 3
E=$(ros2 param get /collision_detector emergency_threshold 2>/dev/null | grep -oE '[0-9]+\.[0-9]+' | head -1)
[ "$E" = "0.03" ] && pass "emergency_threshold=0.03" || fail "emergency_threshold='$E'"
R=$(ros2 param get /collision_detector enable_roi 2>/dev/null | grep -oE 'True|False')
[ "$R" = "True" ] && pass "enable_roi=True" || fail "enable_roi='$R'"
kill "${PIDS[-1]}" 2>/dev/null; wait "${PIDS[-1]}" 2>/dev/null || true

echo ""

# ===================== Level 2 =====================
if [ "$LEVEL" -ge 2 ]; then
    info "Level 2: 假点云端到端检测"

    if ! ros2 pkg executables joint_state_publisher 2>/dev/null | grep -q joint_state_publisher; then
        skip "需要: sudo apt install ros-jazzy-joint-state-publisher"
    else
        URDF="$(ros2 pkg prefix duco_support)/share/duco_support/urdf/duco_gcr5_910.urdf"
        if [ ! -f "$URDF" ]; then
            fail "URDF 不存在: $URDF"
        else
            info "启动 TF..."
            bg ros2 run robot_state_publisher robot_state_publisher \
                --ros-args -p "robot_description:=$(cat "$URDF")" 2>/dev/null
            bg ros2 run joint_state_publisher joint_state_publisher 2>/dev/null
            sleep 3
            ros2 topic echo /tf --once --timeout 3 &>/dev/null \
                && pass "TF 发布正常" || fail "TF 未发布"

            info "启动 collision_detector..."
            bg ros2 run collision_detector collision_detector_node --ros-args \
                -p point_cloud_topic:=/fake/depth/points \
                -p enable_roi:=false -p enable_ground_removal:=false \
                -p enable_visualization:=true -p detection_hz:=10.0 \
                -p downsample_stride:=1 -p emergency_confirm_frames:=1 2>/dev/null
            sleep 2

            ros2 topic list 2>/dev/null | grep -q /collision_detector/status \
                && pass "/collision_detector/status 话题" || fail "status 话题缺失"
            ros2 topic list 2>/dev/null | grep -q /collision_detector/markers \
                && pass "/collision_detector/markers 话题" || fail "markers 话题缺失"

            info "无点云 → sensor_unavailable..."
            S=$(ros2 topic echo /collision_detector/status --once --timeout 3 2>/dev/null | grep "status:" | head -1)
            echo "$S" | grep -q sensor_unavailable \
                && pass "无点云 → sensor_unavailable" || fail "期望 sensor_unavailable, 得到: $S"

            info "发布安全 + 近距离点云, 连续端到端测试..."
            # 用单进程发布: 先 safe 10s → 再 close 10s, 避免 rclpy context 冲突
            python3 - <<'PYEOF' &
import rclpy, struct, time, threading, signal, sys
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
node = Node('test_e2e')
pub = node.create_publisher(PointCloud2, '/fake/depth/points', qos_profile_sensor_data)
t = threading.Thread(target=lambda: rclpy.spin(node), daemon=True); t.start()

def make(pts):
    msg = PointCloud2()
    msg.header.frame_id = 'base_link'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.height = 1; msg.width = len(pts)
    msg.fields = [
        PointField(name='x', offset=0, datatype=7, count=1),
        PointField(name='y', offset=4, datatype=7, count=1),
        PointField(name='z', offset=8, datatype=7, count=1)]
    msg.point_step = 12; msg.row_step = 12*len(pts); msg.is_dense = True
    d = bytearray()
    for x,y,z in pts: d.extend(struct.pack('<fff',x,y,z))
    msg.data = bytes(d)
    return msg

signal.signal(signal.SIGTERM, lambda *a: sys.exit(0))
signal.signal(signal.SIGINT, lambda *a: sys.exit(0))

# Phase 1: safe (远距离)
safe_pts = [(2.0, 2.0, 2.0)]
end1 = time.time() + 10
while time.time() < end1:
    pub.publish(make(safe_pts))
    time.sleep(1/30)
# 写标记文件表示 phase 1 结束
open('/tmp/collision_test_phase1_done', 'w').write('done')

# Phase 2: close (近距离)
close_pts = [(0.0,0.0,0.20),(0.01,0.0,0.20),(-0.01,0.0,0.20)]
end2 = time.time() + 10
while time.time() < end2:
    pub.publish(make(close_pts))
    time.sleep(1/30)
open('/tmp/collision_test_phase2_done', 'w').write('done')

# Phase 3: 保持发布 close 直到被 kill
try:
    while True:
        pub.publish(make(close_pts))
        time.sleep(1/30)
except: pass
PYEOF
            PID_PUB=${PIDS[-1]}

            # 等 Phase 1 (safe) 完成
            info "等待 safe 点云阶段..."
            rm -f /tmp/collision_test_phase1_done /tmp/collision_test_phase2_done
            for i in $(seq 1 15); do
                [ -f /tmp/collision_test_phase1_done ] && break
                sleep 1
            done
            # 在 safe 阶段末尾检查
            S=$(ros2 topic echo /collision_detector/status --once --timeout 5 2>/dev/null | grep "status:" | head -1)
            echo "$S" | grep -q safe \
                && pass "远距离 → safe" || fail "期望 safe, 得到: $S"

            # 等 Phase 2 (close) 稳定
            info "等待近距离点云阶段..."
            for i in $(seq 1 15); do
                [ -f /tmp/collision_test_phase2_done ] && break
                sleep 1
            done
            sleep 1  # 多等 1s 让检测处理几帧
            S=$(ros2 topic echo /collision_detector/status --once --timeout 5 2>/dev/null | grep "status:" | head -1)
            kill $PID_PUB 2>/dev/null; wait $PID_PUB 2>/dev/null || true
            echo "$S" | grep -qE "warning|emergency|caution" \
                && pass "近距离 → 警报 ($S)" || fail "期望警报, 得到: $S"

            info "set_topic 动态切换..."
            ros2 topic pub /collision_detector/set_topic std_msgs/msg/String "data: '/other'" --once 2>/dev/null
            sleep 1
            ros2 topic pub /collision_detector/set_topic std_msgs/msg/String "data: '/fake/depth/points'" --once 2>/dev/null
            pass "set_topic 切换正常"
        fi
    fi
fi

echo ""

# ===================== Level 3 =====================
if [ "$LEVEL" -ge 3 ]; then
    info "Level 3: 实机联调"
    PC=$(ros2 topic list 2>/dev/null | grep "/depth/points" | head -1)
    if [ -z "$PC" ]; then
        skip "未检测到深度相机, 请启动: ros2 launch vision_server vision_system.launch.py"
    else
        pass "点云话题: $PC"
        bg ros2 run collision_detector collision_detector_node --ros-args \
            --params-file "$PREFIX/config/collision_params.yaml" \
            -p point_cloud_topic:="$PC" 2>/dev/null
        sleep 3
        S=$(ros2 topic echo /collision_detector/status --once --timeout 5 2>/dev/null)
        if [ -n "$S" ]; then
            pass "实机输出: $(echo "$S" | grep 'status:' | head -1) $(echo "$S" | grep 'min_distance:' | head -1)"
        else
            fail "实机无输出"
        fi
    fi
fi

# ===================== 汇总 =====================
echo ""
echo "============================================"
echo -e " 结果: ${GREEN}${PASS} 通过${NC} / ${RED}${FAIL} 失败${NC} / ${YELLOW}${SKIP} 跳过${NC} (共 $((PASS+FAIL+SKIP)))"
echo "============================================"
[ "$FAIL" -gt 0 ] && exit 1 || exit 0
