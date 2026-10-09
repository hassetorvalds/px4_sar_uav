#!/usr/bin/env bash
# 为键盘手动接管准备 PX4 的专用 MAVLink 实例（幂等）。
#
# 背景：PX4 自带的 GCS 实例（UDP 18570，remote 14550）不适合作为客户端入口
# （只更新对端地址、固定发往 14550，实测收不到回传流）。因此专开一个实例：
#     端口 14700（PX4 监听） → 14701（发往键盘客户端）
# 该实例写在 PX4 的**构建产物** etc/init.d-posix/rcS 末尾：不修改 PX4 源码（submodule 保持干净），
# 但**每次重建 PX4 后需要重新执行本脚本**；scripts/run_px4_sitl.sh 会在启动前自动调用。
set -euo pipefail

_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="$(cd "${_dir}/../PX4-Autopilot" && pwd)"
RCS="${PX4_DIR}/build/px4_sitl_default/etc/init.d-posix/rcS"
LINE='mavlink start -u 14700 -o 14701 -r 1000000 -m onboard'

if [ ! -f "${RCS}" ]; then
    echo "[manual-link] 未找到 ${RCS}（PX4 尚未构建），跳过" >&2
    exit 0
fi

if grep -qF "${LINE}" "${RCS}"; then
    echo "[manual-link] 专用 MAVLink 实例已配置（14700 → 14701）"
else
    printf '\n# 键盘手动接管专用 MAVLink 实例（由 scripts/enable_manual_link.sh 注入）\n%s\n' \
        "${LINE}" >> "${RCS}"
    echo "[manual-link] 已注入：${LINE}"
fi
