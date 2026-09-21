#!/usr/bin/env bash
# VLM 导航管道端到端验证（mock provider + 静态测试图像，不需要 API key）
#
# 前置：Windows 侧 AirSim 已运行，且已启动 Agent 与 PX4 SITL：
#   scripts/run_agent.sh
#   scripts/run_px4_sitl.sh
#
# 验证内容：图像 → VLM(指向点) → 几何反投影 → 机体速度 → 接口层 → PX4 实际位移
set -uo pipefail

_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${_dir}/env.sh"

IMAGE="${1:-${PX4_SAR_ROOT}/ros2_px4_ws/src/vlm/test/data/test_scene.jpg}"
PROVIDER="${2:-mock}"
IMAGE_TOPIC="${3:-}"
NAV_ARGS_DEFAULT=()
LOG_DIR="${PX4_SAR_ROOT}/logs/$(date +%F)"
mkdir -p "${LOG_DIR}"

read_position () {
    timeout 10 ros2 topic echo --once /fmu/out/vehicle_local_position_v1 2>/dev/null \
        | awk '/^x:/ {x=$2} /^y:/ {y=$2} /^z:/ {z=$2} \
               END {printf "x(N)=%.2f y(E)=%.2f z(D)=%.2f", x, y, z}'
}

echo "== 1) 启动 offboard_bridge =="
ros2 run px4_interface offboard_bridge > "${LOG_DIR}/bridge_console.log" 2>&1 &
BRIDGE_PID=$!
echo "== 1b) 启动相机节点（实时图像模式）=="
ros2 run sensor_bridge airsim_camera --ros-args \
    -p host:="${PX4_SIM_HOST_ADDR}" -p port:=45000 -p depth_interval:=5 \
    > "${LOG_DIR}/airsim_camera_console.log" 2>&1 &
CAMERA_PID=$!
sleep 3

echo "== 2) 解锁 + 进入 OFFBOARD =="
ros2 topic pub --once /offboard_bridge/command std_msgs/String "{data: arm}" >/dev/null
sleep 2
ros2 topic pub --once /offboard_bridge/command std_msgs/String "{data: offboard}" >/dev/null
sleep 2

echo "== 3) 起飞到 3 m =="
timeout 12 ros2 topic pub -r 5 /offboard_bridge/target_pose geometry_msgs/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 3.0}, orientation: {w: 1.0}}}" \
    >/dev/null 2>&1
echo "起飞后位置：$(read_position)"

if [ -n "${IMAGE_TOPIC}" ]; then
    # 注意：不要传 -p image_file:="" —— rcl 会拒绝空值参数并导致节点启动失败
    NAV_ARGS=(-p image_topic:="${IMAGE_TOPIC}")
    echo "== 4) 运行 vlm_navigator（provider=${PROVIDER}，实时图像 ${IMAGE_TOPIC}）=="
else
    NAV_ARGS=(-p image_file:="${IMAGE}" -p image_topic:="/airsim_camera/image")
    echo "== 4) 运行 vlm_navigator（provider=${PROVIDER}，静态图像 ${IMAGE}）=="
fi
timeout 20 ros2 run vlm vlm_navigator --ros-args \
    -p provider:="${PROVIDER}" \
    "${NAV_ARGS[@]}" \
    -p vlm_rate_hz:=0.5 \
    -p base_velocity:=1.0 \
    -p max_speed:=1.5 \
    -p max_yaw_rate:=0.5 \
    -p output_dir:="${LOG_DIR}/vlm_decisions" \
    > "${LOG_DIR}/vlm_navigator_console.log" 2>&1
echo "VLM 阶段结束后位置：$(read_position)"

echo "== 5) 收尾：降落 + 上锁 =="
ros2 topic pub --once /offboard_bridge/command std_msgs/String "{data: land}" >/dev/null
sleep 10
ros2 topic pub --once /offboard_bridge/command std_msgs/String "{data: disarm_force}" >/dev/null
sleep 2
kill "${BRIDGE_PID}" 2>/dev/null
kill "${CAMERA_PID}" 2>/dev/null

echo "== 日志目录：${LOG_DIR} =="
