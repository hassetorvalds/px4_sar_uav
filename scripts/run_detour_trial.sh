#!/usr/bin/env bash
# 一次"自动寻障 → 完整绕行"试验（把纪律脚本化：清节点 → 新 session → 单节点 → preflight → 跑 → 打分）
#
# 用法：
#   scripts/run_detour_trial.sh [标签]        # 标签只用于日志文件名
#
# 设计要点（服务器侧 2026-10-14 要求）：
#   1) 起飞前按 PID 清掉所有 VLM 节点，再启动**唯一一组**；
#   2) 每次试验都用**全新 session_id**（复用会触发服务端会话重置）；
#   3) `scripts/preflight.sh` fail-closed，任何一项不合格就不飞；
#   4) 跑完自动用 analyze_detour.py 按定稿口径打分。
set -uo pipefail

_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${_dir}/env.sh" >/dev/null 2>&1
cd "${PX4_SAR_ROOT}"

label="${1:-trial}"
sid="s$(date +%m%d-%H%M%S)"
log_dir="logs/$(date +%Y-%m-%d)"
# 微转角与门限可外部覆盖：诊断 A 用 ROT_DEG=10（配 local_map 的宽锥 ±35°）
# 注意：ROS 参数是 DOUBLE，整数写法（10）会被解析成 INTEGER 而报错 → 这里统一格式化成小数
rot_deg="$(awk -v v="${ROT_DEG:-15}" 'BEGIN{printf "%.1f", v}')"
rot_tol="$(awk -v v="${ROT_TOL:-1.5}" 'BEGIN{printf "%.2f", v}')"
# ② 仲裁接进主线（2026-10-16）：VLM 的速度不再直连 bridge，而是走 velocity_arbiter
# （几何避障 avoid > 语义 vlm）。用 NO_ARBITER=1 可退回直连做对照。
if [ "${NO_ARBITER:-0}" = "1" ]; then
    vlm_velocity_topic="/offboard_bridge/velocity_setpoint"
else
    vlm_velocity_topic="/velocity_arbiter/vlm_cmd"
fi
# 已知结构点位模式（走 C 的闭合回归）：KNOWN_POINTS="东,北;东,北;…"（**AirSim 世界系**）
if [ -n "${KNOWN_POINTS:-}" ]; then
    seek_flag=false
    # 注意：值里含分号，**不要**在赋值时加引号（shell 不会二次解析变量里的引号），
    # 而是在使用处用双引号包住展开（见下面的 -p scan_points:="${points_arg}"）
    points_arg="${KNOWN_POINTS}"
    world_flag=true
else
    seek_flag=true
    points_arg="0,0"
    world_flag=false
fi
mkdir -p "$log_dir"

echo "== 试验 ${label}：session=${sid} =="
# 1) 按 PID 清理陈旧 VLM 节点（避免"残留节点抢占爬升"这类并发）
for pid in $(pgrep -f "install/vlm/lib/vlm/vlm_navigator" 2>/dev/null); do
    kill "$pid" 2>/dev/null
done
sleep 3

# 收尾清理：驱动结束后必须把本轮 VLM 节点也停掉。否则它会以 900 s 超时继续挂着，
# 既占着话题/推理槽，也会让调用方的终端会话一直"看起来还在运行"（2026-10-15 实测）。
cleanup_vlm() {
    for pid in $(pgrep -f "install/vlm/lib/vlm/vlm_navigat[o]r" 2>/dev/null); do
        kill "$pid" 2>/dev/null
    done
    for pid in $(pgrep -f "ros2 run vlm vlm_navigat[o]r" 2>/dev/null); do
        kill "$pid" 2>/dev/null
    done
    sleep 2
}
trap cleanup_vlm EXIT

# 2) 启动唯一一组 VLM 节点（dry_run + timeout，交权时由驱动脚本发 enable）
rm -rf "vlm_decisions_${sid}"
timeout -s INT 900 ros2 run vlm vlm_navigator --ros-args \
    -p provider:=remote_http -p instruction:="go to the collapsed building ahead" -p dry_run:=true \
    -p image_topic:=/airsim_camera/image -p depth_topic:=/airsim_camera/depth \
    -p vlm_rate_hz:=2.2 -p remote_timeout_s:=3.0 -p warmup_frames:=8 \
    -p base_velocity:=0.3 -p max_speed:=0.3 -p forward_scale:=1.0 -p lateral_scale:=1.0 \
    -p velocity_topic:="${vlm_velocity_topic}" \
    -p max_lateral_speed:=0.3 -p max_yaw_rate:=0.5 -p yaw_lock_s:=0.0 \
    -p run_duration_s:=95.0 -p geofence_max_distance_m:=15.0 \
    -p avoidance_enabled:=true -p map_stale_s:=1.5 -p stale_forward_limit_mps:=0.1 \
    -p obstacle_slow_start_m:=1.2 -p obstacle_stop_m:=0.6 -p obstacle_release_m:=0.9 \
    -p obstacle_slow_min_forward_mps:=0.05 \
    -p obstacle_release_confirm_frames:=3 \
    -p obstacle_blind_zone_m:=0.5 \
    -p obstacle_side_release_m:=1.2 -p obstacle_stop_confirm_frames:=3 \
    -p obstacle_world_tolerance_m:=0.4 -p obstacle_candidate_max_m:=4.0 -p obstacle_track_size:=3 \
    -p obstacle_max_turn_deg:=60.0 -p obstacle_creep_mps:=0.05 \
    -p obstacle_avoid_yaw_rate:=0.15 -p hold_on_slow_response_ms:=800.0 -p max_consecutive_errors:=3 \
    -p hold_max_s:=20.0 -p escape_back_mps:=0.10 -p escape_max_s:=10.0 -p escape_max_m:=1.0 \
    -p escape_max_retries:=1 \
    -p output_dir:="vlm_decisions_${sid}" -p session_id:="${sid}" -p record_frames:=false \
    > "${log_dir}/${label}_${sid}_vlm.log" 2>&1 &
sleep 18

# 3) fail-closed 起飞前检查
if ! "${_dir}/preflight.sh"; then
    echo "preflight 不合格 → **不飞**（试验 ${label} 中止）"
    exit 1
fi

# 4) 自动寻障 + 完整绕行
# 硬超时兜底：即使驱动卡住，也不能让整条流水线无限挂着（2026-10-15：曾出现
# "飞机已落地、终端仍 running"；驱动已加"被外力上锁即收工"，这里是第二道闸）
TIMEOUT_DRIVER="${TIMEOUT_DRIVER:-900}"
timeout -s INT "$TIMEOUT_DRIVER" python3 scripts/vlm_closed_loop.py --ros-args -p start_altitude_m:=2.0 -p run_s:=115.0 \
    -p seek_enabled:=${seek_flag} -p scan_points:="${points_arg}" -p scan_points_world:=${world_flag} \
    -p seek_yaw_rate_deg_s:=15.0 -p seek_s:=26.0 \
    -p seek_margin_m:=3.0 -p seek_min_m:=6.0 -p seek_max_m:=35.0 -p seek_ground_margin_m:=0.5 \
    -p seek_min_points:=40 -p seek_min_span_deg:=6.0 -p seek_max_mad_m:=1.0 \
    -p seek_min_median_m:=6.0 -p seek_max_candidates:=3 \
    -p transit_altitude_m:=3.5 -p transit_timeout_s:=45.0 \
    -p transit_speed_mps:=0.5 -p transit_stop_clearance_m:=1.0 \
    -p transit_echo_back_m:=0.25 \
    -p transit_echo_settle_s:=1.5 -p transit_echo_delta_tol_m:=0.08 -p transit_echo_ignore_s:=5.0 -p transit_echo_max_probes:=2 -p transit_echo_max_back_total_m:=0.5 \
    -p start_pose_enabled:=true -p start_world_north_m:=0.0 -p start_world_east_m:=0.0 \
    -p obstacle_creep_open_side_m:=2.0 -p obstacle_creep_budget_m:=0.5 -p obstacle_creep_budget_s:=10.0 \
    -p pre_position:=true -p pre_scan:=true -p scan_yaw_rate_deg_s:=15.0 -p scan_found_m:=4.0 \
    -p scan_side_open_m:=1.5 -p scan_max_s:=25.0 -p verify_s:=2.5 -p verify_rounds:=2 \
    -p align_tolerance_deg:=8.0 -p align_timeout_s:=10.0 -p align_yaw_rate:=0.4 -p realign_max:=3 \
    -p closable_side_m:=2.5 \
    -p rot_verify:=true -p rot_deg:="${rot_deg}" -p rot_rate_deg_s:=12.0 -p rot_settle_s:=2.5 \
    -p rot_tol_m:="${rot_tol}" -p rot_dist_tol_m:="${rot_tol}" -p rot_pose_age_max_s:=0.5 \
    -p pre_speed_mps:=0.15 -p pre_stop_clearance_m:=0.7 -p pre_timeout_s:=35.0 -p pre_stable_samples:=3 \
    -p output_csv:="${log_dir}/${label}_${sid}_detour.csv" \
    > "${log_dir}/${label}_${sid}_driver.log" 2>&1

# 5) 定稿口径打分
echo "---- 试验 ${label}（session=${sid}）打分 ----"
python3 scripts/analyze_detour.py "vlm_decisions_${sid}" | tail -12
echo "取证：${log_dir}/${label}_${sid}_{vlm,driver}.log"
cleanup_vlm
