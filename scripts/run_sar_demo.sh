#!/usr/bin/env bash
# SAR Demo v0：一句话 → 搜索/扫描 → 代理目标检测 → 报告 → 降落。
# 注意：当前是代理目标（橙色目标物），不是人员检测；报告会明确标注。
set -euo pipefail

_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${_dir}/env.sh" >/dev/null 2>&1
cd "${PX4_SAR_ROOT}"

task_text="${1:-起飞到 2 米，搜索东侧 5 米、南侧 6 米、半径 3 米的区域，原地扫描 30 秒，降落}"

if ! timeout 5 ros2 node list 2>/dev/null | grep -qx '/victim_detector'; then
    echo "[sar-demo] 未检测到 /victim_detector；请先启动感知：" >&2
    echo "  scripts/run_detector.sh            # 或 bringup with_detector:=true" >&2
    exit 2
fi

echo "[sar-demo] 目标：代理目标（橙色目标物，非人员检测）"
echo "[sar-demo] 指令：${task_text}"
export FLY_TASK_CONFIRM=1
exec scripts/fly_task.py "${task_text}" --proxy-target --execute
