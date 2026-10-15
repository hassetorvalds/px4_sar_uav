#!/usr/bin/env bash
# 干净重启 PX4 SITL，并自动清零电池阈值（每次 PX4 启动后必须做）。
#
# 为什么需要（2026-10-15 实测）：我们的试验**不做 AirSim reset**，飞机落在哪、下一轮就从哪起飞——
# d1–d9 的起点实测在 (0,0)～(-5.4,-16.8) 之间漂移，各轮场景不可比，命中率因此忽高忽低。
# PX4 启动时会让 AirSim 复位到固定出生点，所以"每轮之间重启 PX4"是让多次试验可比的最小代价做法
# （也是 docs/next-session-handoff.md 里"建议连续实验间隔重启 PX4"的落地）。
#
# 用法：
#   scripts/restart_px4_sitl.sh            # 重启 + 清零电池阈值 + 等 Ready for takeoff
#   scripts/restart_px4_sitl.sh --no-wait  # 只重启，不等就绪
set -uo pipefail

_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${_dir}/env.sh" >/dev/null 2>&1
cd "${PX4_SAR_ROOT}"

LOG_DIR="${PX4_SAR_LOG_DIR:-/tmp/px4logs}"
CTL="/tmp/px4ctl"
mkdir -p "$LOG_DIR"

echo "[px4] 停止现有 PX4 SITL"
for pid in $(pgrep -f "px4_sitl_default/bin/p[x]4" 2>/dev/null); do
    kill "$pid" 2>/dev/null
done
sleep 3

# 用 fifo 做 stdin：这样能直接往 pxh 里敲 param 命令（清零电池阈值）
rm -f "$CTL"
mkfifo "$CTL"
setsid bash -c "sleep 100000 > $CTL" >/dev/null 2>&1 &
setsid nohup bash "${_dir}/run_px4_sitl.sh" < "$CTL" > "${LOG_DIR}/px4.log" 2>&1 &

if [ "${1:-}" = "--no-wait" ]; then
    echo "[px4] 已启动（不等待就绪）"
    exit 0
fi

# 等 "Ready for takeoff"
for _ in $(seq 1 40); do
    grep -aq "Ready for takeoff" "${LOG_DIR}/px4.log" 2>/dev/null && break
    sleep 1
done
if ! grep -aq "Ready for takeoff" "${LOG_DIR}/px4.log" 2>/dev/null; then
    echo "[px4] 警告：20 s 内没看到 'Ready for takeoff'，日志尾部：" >&2
    tail -5 "${LOG_DIR}/px4.log" >&2
fi

for p in BAT_LOW_THR BAT_CRIT_THR BAT_EMERGEN_THR; do
    echo "param set $p 0" > "$CTL"
    sleep 0.3
    echo "param show $p" > "$CTL"     # 回读一次，便于日志里留下"确实为 0"的证据
    sleep 0.3
done
sleep 0.5
echo "[px4] 电池阈值已清零："
tr '\r' '\n' < "${LOG_DIR}/px4.log" | grep -aE "BAT_(LOW|CRIT|EMERGEN)_THR \[" | tail -3 || true
echo "[px4] 就绪（AirSim 已随 PX4 启动复位到出生点）"
