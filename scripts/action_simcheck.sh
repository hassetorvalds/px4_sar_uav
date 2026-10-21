#!/usr/bin/env bash
# ExecuteTask Action 端到端自检：假 PX4 + task_executor + task_action_server +
# task_action_client，全部跑在独立 ROS_DOMAIN_ID（默认 77），不碰真机。
set -euo pipefail

_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${_dir}/env.sh" >/dev/null 2>&1
cd "${PX4_SAR_ROOT}"

export ROS_DOMAIN_ID="${ACTION_SIMCHECK_DOMAIN_ID:-77}"
if [ "${ACTION_SIMCHECK_ALLOW_LIVE:-0}" != "1" ]; then
    if pgrep -f "px4_sitl_default/bin/p[x]4" >/dev/null 2>&1; then
        echo "[action-simcheck] 拒绝运行：检测到真实 PX4。独立域理论上安全，" >&2
        echo "                  但为避免再次误 arm，默认不并跑；确需并跑设 ACTION_SIMCHECK_ALLOW_LIVE=1。" >&2
        exit 2
    fi
fi

stamp="$(date +%Y%m%d-%H%M%S)"
mkdir -p logs/tasks
task_file="logs/tasks/actionsimcheck_${stamp}.task.json"
report="logs/tasks/actionsimcheck_${stamp}.report.jsonl"
log="logs/tasks/actionsimcheck_${stamp}.log"

cancel_after="${ACTION_SIMCHECK_CANCEL_AFTER:-0}"
disarm_after="${ACTION_SIMCHECK_DISARM_AFTER:-0}"
cancel_after_result="${ACTION_SIMCHECK_CANCEL_AFTER_RESULT:-0}"
cancel_unknown="${ACTION_SIMCHECK_CANCEL_UNKNOWN:-0}"
client_args=()
if [ "${cancel_unknown}" = "1" ]; then
    python3 scripts/fly_task.py "起飞到 2 米，悬停 1 秒，降落" --json > "${task_file}"
    client_args+=(--cancel-unknown)
elif [ "${cancel_after_result}" = "1" ]; then
    python3 scripts/fly_task.py "起飞到 2 米，悬停 1 秒，降落" --json > "${task_file}"
    client_args+=(--cancel-after-result)
elif [ "${disarm_after}" != "0" ]; then
    python3 scripts/fly_task.py "起飞到 2 米，悬停 20 秒，降落" --json > "${task_file}"
elif [ "${cancel_after}" != "0" ]; then
    python3 scripts/fly_task.py "起飞到 2 米，原地扫描 30 秒，降落" --json > "${task_file}"
    client_args=(--cancel-after "${cancel_after}")
    [ "${ACTION_SIMCHECK_CANCEL_TWICE:-0}" = "1" ] && client_args+=(--cancel-twice)
else
    python3 scripts/fly_task.py "起飞到 2 米，悬停 1 秒，降落" --json > "${task_file}"
fi

sim_args=()
[ "${disarm_after}" != "0" ] && sim_args+=(--disarm-after "${disarm_after}")
setsid python3 scripts/fake_px4_sim.py --duration 70 --report "${report}" \
    "${sim_args[@]}" > "logs/tasks/actionsimcheck_${stamp}.fake.log" 2>&1 &
sim_pid=$!
setsid ros2 run mission task_executor --ros-args \
    -p dry_run:=false -p report_path:="${report}" \
    > "logs/tasks/actionsimcheck_${stamp}.executor.log" 2>&1 &
exec_pid=$!
setsid ros2 run mission task_action_server \
    > "logs/tasks/actionsimcheck_${stamp}.server.log" 2>&1 &
action_pid=$!

cleanup() {
    for pid in "${sim_pid}" "${exec_pid}" "${action_pid}"; do
        kill -- -"${pid}" 2>/dev/null || kill "${pid}" 2>/dev/null || true
    done
    wait "${sim_pid}" "${exec_pid}" "${action_pid}" 2>/dev/null || true
}
trap cleanup EXIT

echo "[action-simcheck] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[action-simcheck] task=${task_file}"
echo "[action-simcheck] report=${report}"
sleep 5

set +e
timeout 70 ros2 run mission task_action_client \
    --task-file "${task_file}" --timeout 60 "${client_args[@]}" 2>&1 | tee "${log}"
client_rc=${PIPESTATUS[0]}
set -e

echo "[action-simcheck] client exit=${client_rc}"
echo "[action-simcheck] 报告事件："
grep -o '"event": "[^"]*"' "${report}" 2>/dev/null | sort | uniq -c | tail -20 || true

if [ "${cancel_unknown}" = "1" ]; then
    if ! grep -q 'unknown cancel：return_code=2' "${log}" 2>/dev/null; then
        echo "[action-simcheck] FAIL（unknown cancel 未 ERROR_UNKNOWN_GOAL_ID）" >&2
        exit 1
    fi
    if [ "${client_rc}" -ne 0 ]; then
        echo "[action-simcheck] FAIL（unknown cancel client 退出码 ${client_rc}）" >&2
        exit 1
    fi
    echo "[action-simcheck] PASS（unknown cancel）"
    exit 0
fi
if [ "${cancel_after_result}" = "1" ]; then
    if ! grep -q 'return_code=3' "${log}" 2>/dev/null; then
        echo "[action-simcheck] FAIL（终态 cancel 未 ERROR_GOAL_TERMINATED）" >&2
        exit 1
    fi
    if [ "${client_rc}" -ne 0 ]; then
        echo "[action-simcheck] FAIL（终态 cancel client 退出码 ${client_rc}）" >&2
        exit 1
    fi
    echo "[action-simcheck] PASS（terminal cancel）"
    exit 0
fi
if [ "${disarm_after}" != "0" ]; then
    # 外部 disarm：executor 应报 external_land + done，且不再发 land_fast
    if ! grep -q '"event": "external_land"' "${report}" 2>/dev/null; then
        echo "[action-simcheck] FAIL（没有 external_land）" >&2
        exit 1
    fi
    if ! grep -q '"event": "done"' "${report}" 2>/dev/null; then
        echo "[action-simcheck] FAIL（外部着陆后没有 done）" >&2
        exit 1
    fi
    if [ "${client_rc}" -ne 5 ] && [ "${client_rc}" -ne 0 ]; then
        echo "[action-simcheck] FAIL（external land 场景 client 退出码 ${client_rc}）" >&2
        exit 1
    fi
    echo "[action-simcheck] PASS（external_land）"
    exit 0
fi
if [ "${cancel_after}" != "0" ]; then
    # 取消场景：client 返回 5（success=False），报告应有 cancel_requested + done
    if ! grep -q '"reason": "cancel_requested"' "${report}" 2>/dev/null; then
        echo "[action-simcheck] FAIL（没有 cancel_requested abort）" >&2
        exit 1
    fi
    if ! grep -q '"event": "done"' "${report}" 2>/dev/null; then
        echo "[action-simcheck] FAIL（取消后没有 done）" >&2
        exit 1
    fi
    if [ "${client_rc}" -ne 5 ] && [ "${client_rc}" -ne 0 ]; then
        echo "[action-simcheck] FAIL（cancel 场景 client 退出码 ${client_rc}）" >&2
        exit 1
    fi
    if ! grep -q 'active cancel response: return_code=0 goals_canceling=1' "${log}" 2>/dev/null; then
        echo "[action-simcheck] FAIL（active cancel 不是 ERROR_NONE/1 goal）" >&2
        exit 1
    fi
    echo "[action-simcheck] PASS（cancel）"
    exit 0
fi
if [ "${client_rc}" -ne 0 ]; then
    echo "[action-simcheck] FAIL（client 非 0）" >&2
    exit 1
fi
if ! grep -q '"event": "done"' "${report}" 2>/dev/null; then
    echo "[action-simcheck] FAIL（报告里没有 done）" >&2
    exit 1
fi
if grep -q '"event": "abort"' "${report}" 2>/dev/null; then
    echo "[action-simcheck] FAIL（报告里有 abort）" >&2
    exit 1
fi
echo "[action-simcheck] PASS"
