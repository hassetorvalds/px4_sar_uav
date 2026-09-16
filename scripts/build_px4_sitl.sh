#!/usr/bin/env bash
# 重建 PX4 SITL（换目录后必须执行）
#
# 说明：PX4 的可执行文件里编译进了构建目录的绝对路径，
#       且 build/px4_sitl_default/rootfs/etc 是指向绝对路径的软链接，
#       因此移动源码目录后旧构建产物不可用，必须重新构建。
#       首次构建会联网拉取 eProsima Micro-CDR / Micro-XRCE-DDS-Client。
set -euo pipefail

_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="$(cd "${_dir}/../PX4-Autopilot" && pwd)"

cd "${PX4_DIR}"
echo "[build] 清理 build/px4_sitl_default"
rm -rf build/px4_sitl_default
echo "[build] make px4_sitl"
make px4_sitl
echo "[build] 完成：${PX4_DIR}/build/px4_sitl_default/bin/px4"
