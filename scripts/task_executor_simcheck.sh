#!/usr/bin/env bash
# 无 AirSim/PX4 的执行器集成自检：假 PX4 + 真 task_executor + 报告判定。
#
# 验证：ARM → OFFBOARD → takeoff → goto → hover → return_home → land → DONE
# 不验证：PX4 真实动力学、AirSim 传感器、EKF。
#
# 用法：scripts/task_executor_simcheck.sh
set -uo pipefail

_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${_dir}/env.sh" >/dev/null 2>&1
cd "${PX4_SAR_ROOT}"

# ============ ROS 域隔离（2026-10-18 误 arm 事故后加） ============
# 假 PX4/自检执行器默认跑在独立 ROS_DOMAIN_ID，物理上不可能与真实栈共享
# /fmu/out 或 /offboard_bridge/command。需要与真实栈同域时显式覆盖。
export ROS_DOMAIN_ID="${SIMCHECK_DOMAIN_ID:-77}"

# 域不可达/ros2 不可用：fail-closed，不能假装隔离成功。
if ! timeout 5 ros2 topic list >/dev/null 2>&1; then
    echo "[simcheck] 拒绝运行：ROS_DOMAIN_ID=${ROS_DOMAIN_ID} 不可达或 ros2 不可用。" >&2
    exit 2
fi

# ============ 安全互锁（2026-10-18 实测事故后加） ============
# 假 PX4 与真实栈共用 /offboard_bridge/command 和 /fmu/out：
# 真实 bridge 在跑时，自检执行器的 arm/offboard 会被转发给真 PX4，可能误起飞。
# 默认拒绝在真实 PX4/offboard_bridge 运行时启动自检；确需并跑请显式设
# SIMCHECK_ALLOW_LIVE=1（仅限已确认安全的场景）。
if [ "${SIMCHECK_ALLOW_LIVE:-0}" != "1" ]; then
    live=0
    pgrep -f "px4_sitl_default/bin/p[x]4" >/dev/null 2>&1 && live=1
    pgrep -f "px4_interface/offboard_bridg[e]" >/dev/null 2>&1 && live=1
    if [ "$live" -eq 1 ]; then
        echo "[simcheck] 拒绝运行：检测到真实 PX4 / offboard_bridge 在跑。" >&2
        echo "           simcheck 与真实栈共用话题，可能把 arm/offboard 转发给真机。" >&2
        echo "           请先 scripts/dev_chain.sh stop；确需并跑请设 SIMCHECK_ALLOW_LIVE=1。" >&2
        exit 2
    fi
fi

stamp="$(date +%Y%m%d-%H%M%S)"
mkdir -p logs/tasks
task_file="logs/tasks/simcheck_${stamp}.task.json"
report="logs/tasks/simcheck_${stamp}.report.jsonl"

# 可选：第一个参数覆盖默认任务；SIMCHECK_TIMEOUT 控制执行器超时
task_text="${1:-起飞到 2 米，向北飞 4 米，悬停 2 秒，返航降落}"
exec_timeout="${SIMCHECK_TIMEOUT:-80}"
sim_duration="$((exec_timeout + 10))"

python3 scripts/fly_task.py "${task_text}" --json > "${task_file}"

echo "[simcheck] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}（与真实栈隔离）"
echo "[simcheck] task=${task_file}"
echo "[simcheck] report=${report}"

sim_args=()
[ -n "${SIMCHECK_BLOCK_AFTER:-}" ] && sim_args+=(--block-after "${SIMCHECK_BLOCK_AFTER}")
[ -n "${SIMCHECK_TAKEOVER_AFTER:-}" ] && sim_args+=(--takeover-after "${SIMCHECK_TAKEOVER_AFTER}")
[ -n "${SIMCHECK_EXPECT:-}" ] && sim_args+=(--expect "${SIMCHECK_EXPECT}")

python3 scripts/fake_px4_sim.py --duration "${sim_duration}" \
    --report "${report}" "${sim_args[@]}" &
sim_pid=$!
sleep 2

timeout "${exec_timeout}" ros2 run mission task_executor --ros-args \
    -p task_file:="${task_file}" \
    -p dry_run:=false \
    -p report_path:="${report}" \
    -p status_timeout_s:=2.0 &
exec_pid=$!

wait "${sim_pid}"
rc=$?

kill "${exec_pid}" 2>/dev/null || true
# 不再用 pkill -f：模式可能匹配到调用它的 shell/作业本身（实测会把外层任务一起 SIGTERM）。
wait "${exec_pid}" 2>/dev/null || true
sleep 1

echo "[simcheck] 退出码 ${rc}（0=PASS）"
echo "[simcheck] 报告末尾："
tail -5 "${report}" 2>/dev/null
exit "${rc}"
