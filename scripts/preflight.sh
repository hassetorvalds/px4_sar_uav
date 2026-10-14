#!/usr/bin/env bash
# 起飞前 fail-closed 检查（服务器侧 2026-10-14 建议：把"单组 VLM"脚本化，别只放 checklist）
#
# 为什么：同型并发已经发生三次（m2 抢 m3、m4 抢 m5、m5 重跑触发会话重置）。
# 人容易漏 checklist，脚本不会。**任何一项不合格就 exit 1，不飞。**
#
# 用法：
#   scripts/preflight.sh                # 只检查
#   SESSION_ID="$(scripts/preflight.sh --session)" && echo "$SESSION_ID"   # 检查通过并生成新 session id
set -uo pipefail

_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${_dir}/env.sh" >/dev/null 2>&1

fail=0
note() { printf '  [%s] %-42s %s\n' "$1" "$2" "$3"; }
need_one() {
    local label="$1" pattern="$2" count
    count="$(pgrep -c -f "$pattern" 2>/dev/null || true)"
    count="${count:-0}"
    if [ "$count" = "1" ]; then
        note OK "$label" "1"
    else
        note FAIL "$label" "期望 1，实际 $count"
        fail=1
    fi
}

echo "== 起飞前检查（fail-closed）=="
# 单组关键进程（每个节点取"最内层可执行文件"计数，避免把进程树算成多个）
need_one "VLM 节点（vlm_navigator）" "install/vlm/lib/vlm/vlm_navigator"
need_one "offboard_bridge" "install/px4_interface/lib/px4_interface/offboard_bridge"
need_one "airsim_camera" "install/sensor_bridge/lib/sensor_bridge/airsim_camera"
need_one "local_map" "install/mapping/lib/mapping/local_map"
need_one "flight_supervisor" "scripts/flight_superviso[r].py"
need_one "PX4 SITL" "px4_sitl_default/bin/p[x]4"
need_one "Micro XRCE-DDS Agent" "MicroXRCEAge[n]t"

# PX4 → ROS 链路（用 echo 判断，比 hz 快且不依赖统计窗口）
if timeout 8 ros2 topic echo --once /fmu/out/vehicle_local_position_v1 2>/dev/null | grep -q '^z:'; then
    note OK "/fmu/out 有数据" "vehicle_local_position 可读"
else
    note FAIL "/fmu/out 无数据" "检查 Agent/PX4"
    fail=1
fi

# 地图是否在建图（frames>0）
status="$(timeout 6 ros2 topic echo --once /local_map/status 2>/dev/null | head -1 || true)"
frames="$(printf '%s' "$status" | sed -n 's/.*frames=\([0-9]*\).*/\1/p')"
if [ -n "$frames" ] && [ "$frames" -gt 0 ]; then
    note OK "local_map frames" "$frames"
else
    note FAIL "local_map frames" "0 或拿不到（检查相机/PX4 位置）"
    fail=1
fi

# 远端服务（隧道）
if curl -s -m 4 http://127.0.0.1:18080/health | grep -q '"ready":true'; then
    note OK "远端 /health" "ready"
else
    note FAIL "远端 /health" "不可达（隧道/服务？）"
    fail=1
fi

if [ "$fail" != "0" ]; then
    echo
    echo "结果：不合格 → **不要起飞**。先按 PID 清理陈旧节点、恢复缺失组件后重跑本检查。"
    echo "（清理示例：scripts/stop_stack.sh；VLM 节点用 pgrep 取 PID 后 kill）"
    exit 1
fi

if [ "${1:-}" = "--session" ]; then
    # 全新 session id：每次飞行都必须换新（复用会触发服务端会话重置）
    printf 's%s\n' "$(date +%m%d-%H%M%S)"
else
    echo
    echo "结果：合格 → 可以起飞。记得用**全新 session_id**（scripts/preflight.sh --session 生成）。"
fi
