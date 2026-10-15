#!/usr/bin/env bash
# 开发链路一键起停（用户习惯：每次暂离彻底关闭 WSL，回来再全部拉起）。
#
# 用法：
#   scripts/dev_chain.sh start     # 隧道 → Agent → PX4(含电池阈值清零) → ROS 栈 → 监督层，并自检
#   scripts/dev_chain.sh stop      # 反向全停（栈/监督层/VLM/驱动/PX4/Agent/隧道）
#   scripts/dev_chain.sh status    # 只看现状（各组件是否在线 + 远端 health）
#
# 注意：
#   · `start` 里的自检**不含 VLM 节点**（它只在试验期间存在）；飞行前的 fail-closed 门禁仍是
#     `scripts/preflight.sh`（由 run_detour_trial.sh 在启动 VLM 节点后调用）。
#   · PX4 重启**不会**把飞机搬回世界出生点（只重设 PX4 本地原点，2026-10-15 更正）；
#     固定起始位姿由驱动用 `start_pose_enabled` 实现（不调 AirSim reset，不断 lockstep）。
set -uo pipefail

_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${_dir}/env.sh" >/dev/null 2>&1
cd "${PX4_SAR_ROOT}"
LOG_DIR="${PX4_SAR_LOG_DIR:-/tmp/px4logs}"
mkdir -p "$LOG_DIR"

count() { pgrep -c -f "$1" 2>/dev/null || true; }

tunnel_ok() {
    curl -s -m 5 http://127.0.0.1:18080/health 2>/dev/null | grep -q '"ready":true'
}

do_status() {
    echo "== 开发链路状态 =="
    printf '  %-28s %s\n' "VLM 隧道 + 远端 /health" "$(tunnel_ok && echo 'OK (ready)' || echo 'DOWN')"
    for pair in "MicroXRCE-DDS Agent|MicroXRCEAge[n]t" \
                "PX4 SITL|px4_sitl_default/bin/p[x]4" \
                "offboard_bridge|install/px4_interface/lib/px4_interface/offboard_bridge" \
                "airsim_camera|install/sensor_bridge/lib/sensor_bridge/airsim_camera" \
                "local_map|install/mapping/lib/mapping/local_map" \
                "local_planner|install/planner/lib/planner/local_planner" \
                "velocity_arbiter|install/px4_interface/lib/px4_interface/velocity_arbiter" \
                "flight_supervisor|flight_superviso[r].py" \
                "VLM 节点(试验期)|install/vlm/lib/vlm/vlm_navigat[o]r" \
                "驱动脚本(试验期)|vlm_closed_loo[p].py"; do
        label="${pair%%|*}"; pat="${pair##*|}"
        printf '  %-28s %s\n' "$label" "$(count "$pat")"
    done
}

do_start() {
    echo "== 1/5 VLM 隧道 =="
    "${_dir}/vlm_tunnel.sh" start 2>&1 | tail -2
    if ! tunnel_ok; then
        echo "远端 /health 未 ready —— 服务器侧服务可能没起，先与服务器侧确认再继续。" >&2
        return 1
    fi

    echo "== 2/5 Micro XRCE-DDS Agent =="
    if [ "$(count "MicroXRCEAge[n]t")" = "0" ]; then
        setsid nohup bash "${_dir}/run_agent.sh" > "${LOG_DIR}/agent.log" 2>&1 &
        sleep 4
    fi
    printf '  Agent 实例数: %s（期望 1）\n' "$(count "MicroXRCEAge[n]t")"

    echo "== 3/5 PX4 SITL（含电池阈值清零）=="
    "${_dir}/restart_px4_sitl.sh" 2>&1 | tail -4

    echo "== 4/5 ROS 栈 =="
    if [ "$(count "install/px4_interface/lib/px4_interface/offboard_bridge")" = "0" ]; then
        setsid nohup bash "${_dir}/bringup.sh" camera_rate_hz:=2.5 > "${LOG_DIR}/bringup.log" 2>&1 &
    fi

    echo "== 5/5 落地监督层 =="
    if [ "$(count "flight_superviso[r].py")" = "0" ]; then
        setsid nohup python3 "${_dir}/flight_supervisor.py" --ros-args \
            -p timeout_s:=5.0 -p arm_grace_s:=6.0 > "${LOG_DIR}/supervisor.log" 2>&1 &
    fi
    sleep 14

    echo
    do_status
    echo
    echo "提醒：飞行用 scripts/run_detour_trial.sh <标签>（它会起 VLM 节点并跑 fail-closed preflight）。"
}

do_stop() {
    echo "== 停 VLM/驱动 =="
    for pat in "vlm_closed_loo[p].py" "install/vlm/lib/vlm/vlm_navigat[o]r" "ros2 run vlm vlm_navigat[o]r"; do
        for pid in $(pgrep -f "$pat" 2>/dev/null); do kill "$pid" 2>/dev/null; done
    done
    echo "== 停 ROS 栈 =="
    "${_dir}/stop_stack.sh" 2>&1 | tail -7
    echo "== 停监督层 =="
    for pid in $(pgrep -f "flight_superviso[r].py" 2>/dev/null); do kill "$pid" 2>/dev/null; done
    echo "== 停 PX4 / Agent =="
    for pid in $(pgrep -f "px4_sitl_default/bin/p[x]4" 2>/dev/null); do kill "$pid" 2>/dev/null; done
    for pid in $(pgrep -f "MicroXRCEAge[n]t" 2>/dev/null); do kill "$pid" 2>/dev/null; done
    sleep 3
    echo "== 停隧道 =="
    "${_dir}/vlm_tunnel.sh" stop 2>&1 | tail -2
    sleep 1
    echo
    do_status
    echo
    echo "（AirSim/UE4 由你在 Windows 侧决定是否保留；本项目不会再发任何指令。）"
}

case "${1:-status}" in
    start)  do_start ;;
    stop)   do_stop ;;
    status) do_status ;;
    *) echo "用法：$0 [start|stop|status]" >&2; exit 2 ;;
esac
