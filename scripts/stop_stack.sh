#!/usr/bin/env bash
# 停止 ROS 2 仿真栈（含**子节点**）——不再用"只杀 ros2 launch"的偷懒做法。
#
# 背景（2026-10-13 实测）：`pkill -f sim_stack.launch` 只杀掉 launch 父进程，
# offboard_bridge / airsim_camera / local_map / local_planner **会存活**。
# 结果同一话题上有多个发布者：3 个 offboard_bridge 同时发设定点、
# 3 个 local_map 同时发 /local_map/status（新旧版本混在一起），
# 表现为"净空读数前后矛盾、摆位停在不该停的地方"，排查了大半天。
#
# 用法：scripts/stop_stack.sh
set -uo pipefail

patterns=(
    "sim_stack.launch"
    "px4_interface/offboard_bridge"
    "sensor_bridge/airsim_camera"
    "mapping/local_map"
    "planner/local_planner"
)

for pattern in "${patterns[@]}"; do
    # 用中括号避免 pkill 匹配到本脚本自身的命令行
    regex="$(printf '%s' "$pattern" | sed 's/\(.\)$/[\1]/')"
    pkill -f "$regex" 2>/dev/null || true
done

sleep 3
echo "停止后剩余进程计数："
for pattern in "${patterns[@]}"; do
    regex="$(printf '%s' "$pattern" | sed 's/\(.\)$/[\1]/')"
    count="$(pgrep -c -f "$regex" 2>/dev/null)"
    count="${count:-0}"
    printf '  %-32s %s\n' "$pattern" "$count"
done
