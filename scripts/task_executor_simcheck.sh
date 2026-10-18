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

stamp="$(date +%Y%m%d-%H%M%S)"
mkdir -p logs/tasks
task_file="logs/tasks/simcheck_${stamp}.task.json"
report="logs/tasks/simcheck_${stamp}.report.jsonl"

# 可选：第一个参数覆盖默认任务；SIMCHECK_TIMEOUT 控制执行器超时
task_text="${1:-起飞到 2 米，向北飞 4 米，悬停 2 秒，返航降落}"
exec_timeout="${SIMCHECK_TIMEOUT:-80}"
sim_duration="$((exec_timeout + 10))"

python3 scripts/fly_task.py "${task_text}" --json > "${task_file}"

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
