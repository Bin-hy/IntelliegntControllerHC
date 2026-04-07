#!/bin/bash
# eartest.quick.sh - 快速耳机检测测试脚本（支持复用基线）
#
# 用法:
#   bash eartest.quick.sh                    # 自动检测相机，首次采集基线后自动保存
#   bash eartest.quick.sh --reuse            # 复用上次保存的基线（无需摘耳机！）
#   bash eartest.quick.sh --baseline <路径>  # 指定基线目录
#   bash eartest.quick.sh <相机命名空间>     # 指定相机
#
# 示例:
#   # 第一次运行：采集基线，保存，测量
#   bash eartest.quick.sh
#
#   # 后续运行：直接复用上次基线，不用摘耳机
#   bash eartest.quick.sh --reuse
#
#   # 指定某次保存的基线
#   bash eartest.quick.sh --baseline ~/.ros/earphone_inspection/_baselines/1775289191209256171
set -e

source /opt/ros/jazzy/setup.bash
source ~/IntelliegntControllerHC/ros2_ws/install/setup.bash

# ============================================================
# 0. 解析参数
# ============================================================
REUSE_BASELINE=""
BASELINE_PATH=""
CAMERA_NS=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --reuse|-r)
            REUSE_BASELINE="latest"
            shift ;;
        --baseline|-b)
            BASELINE_PATH="$2"
            shift 2 ;;
        *)
            CAMERA_NS="$1"
            shift ;;
    esac
done

# ============================================================
# 1. 选择相机
# ============================================================
if [ -z "$CAMERA_NS" ]; then
    echo "=== 检测可用深度相机 ==="
    DEPTH_TOPICS=$(ros2 topic list 2>/dev/null | grep "depth/image_raw" || true)
    if [ -z "$DEPTH_TOPICS" ]; then
        echo "错误: 没有检测到深度相机话题！请先启动相机。"
        exit 1
    fi

    echo "$DEPTH_TOPICS"
    echo ""

    CAMERAS=()
    while IFS= read -r line; do
        ns=$(echo "$line" | sed 's|/depth/image_raw||')
        CAMERAS+=("$ns")
    done <<< "$DEPTH_TOPICS"

    if [ ${#CAMERAS[@]} -eq 1 ]; then
        CAMERA_NS="${CAMERAS[0]}"
        echo "只有一个相机，自动选择: $CAMERA_NS"
    else
        echo "检测到 ${#CAMERAS[@]} 个深度相机："
        for i in "${!CAMERAS[@]}"; do
            echo "  [$i] ${CAMERAS[$i]}"
        done
        echo ""
        read -rp "请选择相机编号 [0]: " choice
        choice=${choice:-0}
        CAMERA_NS="${CAMERAS[$choice]}"
    fi
fi

echo ""
echo ">>> 使用相机: $CAMERA_NS <<<"
echo ""

# ============================================================
# 2. 启动 inspector 节点
# ============================================================
NODE_NS="earphone_test"

pkill -f "__ns:=/${NODE_NS}" 2>/dev/null || true
sleep 1

echo "启动 earphone_inspector_node (ns=$NODE_NS, camera=$CAMERA_NS) ..."
ros2 run vision_server earphone_inspector_node \
    --ros-args \
    -r __ns:=/$NODE_NS \
    -p camera_ns:="$CAMERA_NS" \
    -p avg_frames:=8 \
    -p min_diff_mm:=2.0 \
    -p max_diff_mm:=50.0 \
    -p min_area_pixels:=30 \
    -p roi_width:=250 \
    -p roi_height:=250 \
    -p enable_aruco:=true \
    -p enable_multi_trial:=true &
INSPECTOR_PID=$!
echo "Inspector PID: $INSPECTOR_PID"

echo "等待节点启动..."
for i in $(seq 1 30); do
    if ros2 service list 2>/dev/null | grep -q "/${NODE_NS}/capture_baseline"; then
        echo "节点就绪! (${i}s)"
        break
    fi
    sleep 1
    if [ "$i" -eq 30 ]; then
        echo "错误: 节点启动超时"
        kill $INSPECTOR_PID 2>/dev/null || true
        exit 1
    fi
done

sleep 2  # 等待帧缓冲填充

cleanup() {
    echo ""
    echo "关闭测试节点 (PID=$INSPECTOR_PID)..."
    kill $INSPECTOR_PID 2>/dev/null || true
    wait $INSPECTOR_PID 2>/dev/null || true
}
trap cleanup EXIT

# ============================================================
# 3. 基线：采集新的 or 加载已有的
# ============================================================
LATEST_BASELINE="$HOME/.ros/earphone_inspection/_baselines/latest"

# Helper: 调用 capture_baseline 并检查是否成功
call_baseline() {
    local load_arg="$1"
    local output
    output=$(ros2 service call /$NODE_NS/capture_baseline vision_server/srv/CaptureBaseline \
        "{load_path: '$load_arg'}" 2>&1)
    echo "$output"
    if echo "$output" | grep -q "success=True"; then
        return 0
    else
        return 1
    fi
}

# Helper: 采集新基线
capture_new_baseline() {
    echo ""
    echo "[步骤1] 确保耳朵前方没有耳机"
    read -rp "按回车采集新基线..."
    echo ""
    if ! call_baseline ""; then
        echo "错误: 基线采集失败！"
        exit 1
    fi
}

BASELINE_OK=false

if [ -n "$BASELINE_PATH" ]; then
    # 用户指定了基线路径
    echo ""
    echo ">>> 加载指定基线: $BASELINE_PATH"
    echo ""
    if call_baseline "$BASELINE_PATH"; then
        BASELINE_OK=true
    else
        echo ""
        echo ">>> 加载失败！该目录可能没有 baseline_avg.exr 文件。"
        echo "    提示: 使用 _baselines/ 目录或包含 baseline_avg.exr 的测量结果目录"
        echo ""
        # 检查是否有 latest 可用
        if [ -L "$LATEST_BASELINE" ] || [ -d "$LATEST_BASELINE" ]; then
            RESOLVED=$(readlink -f "$LATEST_BASELINE")
            read -rp "发现已保存的基线 $RESOLVED，是否使用? [Y/n]: " use_latest
            case "$use_latest" in
                [nN]) capture_new_baseline && BASELINE_OK=true ;;
                *) call_baseline "latest" && BASELINE_OK=true ;;
            esac
        else
            capture_new_baseline && BASELINE_OK=true
        fi
    fi

elif [ "$REUSE_BASELINE" = "latest" ]; then
    # 复用上次基线
    if [ -L "$LATEST_BASELINE" ] || [ -d "$LATEST_BASELINE" ]; then
        RESOLVED=$(readlink -f "$LATEST_BASELINE")
        echo ""
        echo ">>> 复用上次保存的基线: $RESOLVED"
        echo ""
        if call_baseline "latest"; then
            BASELINE_OK=true
        else
            echo ">>> 加载失败，采集新基线"
            capture_new_baseline && BASELINE_OK=true
        fi
    else
        echo ""
        echo ">>> 没有找到已保存的基线，将采集新的基线"
        capture_new_baseline && BASELINE_OK=true
    fi

else
    # 正常模式：有保存的就问是否复用
    if [ -L "$LATEST_BASELINE" ] || [ -d "$LATEST_BASELINE" ]; then
        RESOLVED=$(readlink -f "$LATEST_BASELINE")
        echo ""
        echo "发现已保存的基线: $RESOLVED"
        read -rp "是否复用? [Y/n]: " use_saved
        case "$use_saved" in
            [nN]) capture_new_baseline && BASELINE_OK=true ;;
            *)
                echo ""
                echo ">>> 复用已保存的基线"
                if call_baseline "latest"; then
                    BASELINE_OK=true
                else
                    echo ">>> 加载失败，采集新基线"
                    capture_new_baseline && BASELINE_OK=true
                fi
                ;;
        esac
    else
        capture_new_baseline && BASELINE_OK=true
    fi
fi

if [ "$BASELINE_OK" != "true" ]; then
    echo "错误: 无法获取基线数据，退出。"
    exit 1
fi

# ============================================================
# 4. 测量循环
# ============================================================
ROUND=1
while true; do
    echo ""
    echo "====================================="
    echo "  第 $ROUND 轮测量  |  相机: $CAMERA_NS"
    echo "====================================="

    echo ""
    read -rp "放好耳机后按回车开始测量 (q=退出)... " input
    if [ "$input" = "q" ] || [ "$input" = "Q" ]; then
        break
    fi

    echo ""
    TAG="round${ROUND}_$(date +%Y%m%d_%H%M%S)"
    ros2 service call /$NODE_NS/measure vision_server/srv/MeasureEarphone "{file_tag: '$TAG'}"

    echo ""
    echo "调试图像: ~/.ros/earphone_inspection/$TAG/"

    ROUND=$((ROUND + 1))
done

echo ""
echo "测试结束。共 $((ROUND - 1)) 轮测量。"
echo "所有结果: ls ~/.ros/earphone_inspection/"
echo "已保存基线: ls ~/.ros/earphone_inspection/_baselines/"
