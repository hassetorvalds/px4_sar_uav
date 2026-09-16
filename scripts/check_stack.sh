#!/usr/bin/env bash
# 冒烟检查：AirSim -> PX4 -> uXRCE-DDS -> ROS 2 链路是否在线
#
# 前置条件：Windows 侧 AirSim 已运行；本机已启动
#   scripts/run_agent.sh 与 scripts/run_px4_sitl.sh
set -uo pipefail

_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${_dir}/env.sh"

fail=0
topics="$(ros2 topic list 2>/dev/null)"

check_out_topic () {
    local t="$1" hz
    if grep -qx "$t" <<<"$topics"; then
        hz="$(timeout 8 ros2 topic hz "$t" 2>/dev/null | awk '/average rate/ {print $3; exit}')"
        printf '  [ OK ] %-40s hz=%s\n' "$t" "${hz:-n/a}"
    else
        printf '  [FAIL] %-40s 话题不存在\n' "$t"
        fail=1
    fi
}

echo "== PX4 -> ROS 2 (/fmu/out) =="
check_out_topic /fmu/out/vehicle_status_v4
check_out_topic /fmu/out/vehicle_odometry
check_out_topic /fmu/out/vehicle_local_position_v1
check_out_topic /fmu/out/vehicle_attitude

echo "== ROS 2 -> PX4 (/fmu/in，需 offboard 节点运行) =="
for t in /fmu/in/offboard_control_mode /fmu/in/trajectory_setpoint /fmu/in/vehicle_command; do
    if grep -qx "$t" <<<"$topics"; then
        printf '  [ OK ] %s\n' "$t"
    else
        printf '  [skip] %-40s 未启动 offboard 节点时属正常\n' "$t"
    fi
done

echo
if [ "$fail" -eq 0 ]; then
    echo "结果：链路在线"
else
    echo "结果：链路不完整，检查 AirSim 是否运行、agent 与 PX4 是否启动"
fi
exit "$fail"
