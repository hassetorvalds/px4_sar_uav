#!/usr/bin/env bash
# "保持超时脱困（obstacle_escape）"的**飞行中**验证（服务器侧 2026-10-14 第 ④ 条）。
#
# 为什么要单独一个脚本：安全层只在 **armed + OFFBOARD** 时才生效
# （地面注入会被 vlm_navigator 短路成 guard=idle，验证不到脱困）。所以必须真飞。
#
# 做法：完整复用 run_detour_trial.sh 的流程（清节点 → 新 session → 单组 VLM →
# preflight → 寻障/转场/复验/对准/逼近 → 交权），只在交权瞬间开始向 VLM 节点的
# 地图话题注入"两侧都紧的墙"，逼出"保持 20 s → 脱困 → 再保持 20 s → 二次脱困 → 放弃"。
#
# 用法：scripts/run_escape_trial.sh [标签]
set -uo pipefail

_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${_dir}/env.sh" >/dev/null 2>&1
cd "${PX4_SAR_ROOT}"

label="${1:-escape}"
scenario="${INJECT_SCENARIO:-escape_flight}"     # 可用 INJECT_SCENARIO=blind 跑边界用例
sid="s$(date +%m%d-%H%M%S)"
log_dir="logs/$(date +%Y-%m-%d)"
mkdir -p "$log_dir"

echo "== 脱困验证 ${label}：session=${sid} =="
for pid in $(pgrep -f "install/vlm/lib/vlm/vlm_navigator" 2>/dev/null); do
    [ "$pid" != "$$" ] && kill "$pid" 2>/dev/null
done
sleep 3

rm -rf "vlm_decisions_${sid}"
# VLM 节点：地图来自**注入**话题；交权后由注入场景接管净空
timeout -s INT 900 ros2 run vlm vlm_navigator --ros-args \
    -p provider:=remote_http -p instruction:="go to the collapsed building ahead" \
    -p image_topic:=/airsim_camera/image -p depth_topic:=/airsim_camera/depth \
    -p map_status_topic:=/inject/status -p dry_run:=true \
    -p vlm_rate_hz:=2.2 -p remote_timeout_s:=3.0 -p warmup_frames:=8 \
    -p base_velocity:=0.3 -p max_speed:=0.3 -p forward_scale:=1.0 -p lateral_scale:=1.0 \
    -p max_lateral_speed:=0.3 -p max_yaw_rate:=0.5 -p yaw_lock_s:=0.0 \
    -p run_duration_s:=150.0 -p geofence_max_distance_m:=15.0 \
    -p avoidance_enabled:=true -p map_stale_s:=1.5 -p stale_forward_limit_mps:=0.1 \
    -p obstacle_slow_start_m:=1.2 -p obstacle_stop_m:=0.6 -p obstacle_release_m:=0.9 \
    -p obstacle_side_release_m:=1.2 -p obstacle_stop_confirm_frames:=3 \
    -p obstacle_world_tolerance_m:=0.4 -p obstacle_candidate_max_m:=4.0 -p obstacle_track_size:=3 \
    -p obstacle_max_turn_deg:=60.0 -p obstacle_creep_mps:=0.05 -p obstacle_creep_open_side_m:=1.5 \
    -p obstacle_avoid_yaw_rate:=0.15 -p hold_max_s:=20.0 -p escape_back_mps:=0.10 \
    -p escape_max_s:=10.0 -p escape_max_m:=1.0 -p escape_max_retries:=1 \
    -p obstacle_blind_zone_m:=0.5 -p obstacle_creep_open_side_m:=2.0 \
    -p obstacle_creep_budget_m:=0.5 -p obstacle_creep_budget_s:=10.0 \
    -p obstacle_release_confirm_frames:=3 \
    -p hold_on_slow_response_ms:=800.0 -p max_consecutive_errors:=3 \
    -p output_dir:="vlm_decisions_${sid}" -p session_id:="${sid}" -p record_frames:=false \
    > "${log_dir}/${label}_${sid}_vlm.log" 2>&1 &
# 收尾清理：驱动结束后停掉本轮 VLM 节点（否则它会挂到 900 s 超时，且让调用方终端会话不结束）
cleanup_vlm() {
    for pid in $(pgrep -f "install/vlm/lib/vlm/vlm_navigat[o]r" 2>/dev/null); do kill "$pid" 2>/dev/null; done
    for pid in $(pgrep -f "ros2 run vlm vlm_navigat[o]r" 2>/dev/null); do kill "$pid" 2>/dev/null; done
    sleep 2
}
trap cleanup_vlm EXIT
sleep 18

if ! "${_dir}/preflight.sh"; then
    echo "preflight 不合格 → **不飞**（脱困验证 ${label} 中止）"
    exit 1
fi

# 驱动器：**只爬升到 2 m 就直接交权**（不做寻障）——脱困验证不需要真障碍，
# 障碍由注入的净空给出，这样每一轮都能稳定复现"保持 → 脱困"。
python3 scripts/vlm_closed_loop.py --ros-args -p start_altitude_m:=2.0 -p run_s:=150.0 \
    -p pre_position:=false \
    -p output_csv:="${log_dir}/${label}_${sid}_detour.csv" \
    > "${log_dir}/${label}_${sid}_driver.log" 2>&1 &
driver_pid=$!

# 等交权（driver 打出 enable）；最多等 150 s
for _ in $(seq 1 150); do
    grep -aq "已向 VLM 节点发送 enable" "${log_dir}/${label}_${sid}_driver.log" 2>/dev/null && break
    kill -0 "$driver_pid" 2>/dev/null || break
    sleep 1
done
if grep -aq "已向 VLM 节点发送 enable" "${log_dir}/${label}_${sid}_driver.log" 2>/dev/null; then
    echo "已交权 → 开始注入'两侧都紧的墙'（逼出脱困）"
    timeout 120 python3 scripts/inject_clearance.py --ros-args \
        -p scenario:="${scenario}" -p topic:=/inject/status \
        > "${log_dir}/${label}_${sid}_inject.log" 2>&1
else
    echo "本轮没有交权（无 enable）→ 未注入；驱动日志尾部："
    tail -5 "${log_dir}/${label}_${sid}_driver.log"
fi
wait "$driver_pid" 2>/dev/null

echo "---- 脱困验证 ${label}（session=${sid}）打分 ----"
python3 scripts/analyze_detour.py "vlm_decisions_${sid}" | tail -14
echo "---- 脱困日志（应出现 脱困/不再后退）----"
grep -aE "脱困|不再后退|恢复常规" "${log_dir}/${label}_${sid}_vlm.log" | tail -8
echo "取证：${log_dir}/${label}_${sid}_{vlm,driver,inject}.log"
