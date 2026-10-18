#!/usr/bin/env bash
# 冒烟检查：AirSim -> PX4 -> uXRCE-DDS -> ROS 2 链路是否**真的在出数据**
#
# 前置条件：Windows 侧 AirSim 已运行；本机已启动
#   scripts/run_agent.sh 与 scripts/run_px4_sitl.sh
#
# 与旧版的区别（2026-10-17，外部评审 P2）：旧版"话题存在即 OK"，`hz=n/a` 也算成功，
# 会把"话题已注册但没有数据流"误判为在线。现在：
#   · 每个关键话题有**最低频率**，低于阈值/无读数/采样超时一律 FAIL（退出码非 0）；
#   · 结果同时写入 logs/checks/check_stack_<时间>.log，便于跨提交比较。
set -uo pipefail

_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${_dir}/env.sh" >/dev/null 2>&1

_window=10          # 单个话题采样窗口（秒）
_report_dir="${PX4_SAR_ROOT}/logs/checks"
mkdir -p "${_report_dir}"
_report="${_report_dir}/check_stack_$(date +%Y%m%d-%H%M%S).log"
# 让屏幕输出同时进日志
exec > >(tee "${_report}") 2>&1

fail=0
topics="$(ros2 topic list 2>/dev/null)"

# 用法：check_out_topic <话题> <最低频率 Hz>
check_out_topic () {
    local t="$1" min_hz="$2" hz=""
    if ! grep -qx "$t" <<<"$topics"; then
        printf '  [FAIL] %-40s 话题不存在\n' "$t"
        fail=1
        return
    fi
    # ros2 topic hz 会持续打印，取窗口内最后一个 average rate
    hz="$(timeout "${_window}" ros2 topic hz "$t" 2>/dev/null \
          | awk '/average rate/ {v=$3} END {print v}')"
    if [ -z "$hz" ]; then
        printf '  [FAIL] %-40s 存在但 %ss 内无数据流\n' "$t" "${_window}"
        fail=1
        return
    fi
    if awk -v h="$hz" -v m="$min_hz" 'BEGIN{exit !(h + 0 >= m + 0)}'; then
        printf '  [ OK ] %-40s hz=%.1f（阈值 ≥%s）\n' "$t" "$hz" "$min_hz"
    else
        printf '  [FAIL] %-40s hz=%.1f 低于阈值 ≥%s\n' "$t" "$hz" "$min_hz"
        fail=1
    fi
}

echo "== PX4 -> ROS 2 (/fmu/out)，窗口 ${_window}s =="
check_out_topic /fmu/out/vehicle_status_v4          1
check_out_topic /fmu/out/vehicle_odometry           5
check_out_topic /fmu/out/vehicle_local_position_v1 10
check_out_topic /fmu/out/vehicle_attitude          10

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
    echo "结果：链路在线（所有关键话题频率达标）"
else
    echo "结果：链路不完整/无数据流——检查 AirSim 是否运行、agent 与 PX4 是否启动，"
    echo "      并对照上面每条 FAIL 的原因（不存在 / 无数据 / 频率低）。"
fi
echo "报告：${_report}"
exit "$fail"
