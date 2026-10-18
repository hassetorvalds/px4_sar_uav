#!/usr/bin/env bash
# 启动感知节点（victim_detector）。
#
# 为什么单独一个脚本：本机系统 cv2 4.5.4 是针对 numpy 1.x 编译的，而 ~/.local 里装了
# numpy 2.2.6；直接 `ros2 run perception victim_detector` 在选 hog/haar 时会 import cv2 失败。
# `PYTHONNOUSERSITE=1` 让 python 只用系统 numpy（1.21.5），cv2 即可正常工作。
# color（颜色信标）后端不需要 cv2，但用同一入口保证行为一致。
#
# 用法：
#   scripts/run_detector.sh                      # 默认 color
#   scripts/run_detector.sh -p detector:=hog     # 行人 HOG
#   DETECTOR=hog scripts/run_detector.sh
set -uo pipefail

_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${_dir}/env.sh" >/dev/null 2>&1

export PYTHONNOUSERSITE=1
_detector="${DETECTOR:-color}"

exec ros2 run perception victim_detector --ros-args -p detector:="${_detector}" "$@"
