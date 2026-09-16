#!/usr/bin/env bash
# 重建 Micro XRCE-DDS Agent 工作区
#
# 说明：Agent 的 install/（含 MicroXRCEAgent 可执行文件）不入库，
#       新克隆的仓库必须执行本脚本才能得到 Agent。
#       首次构建需要通过 CMake ExternalProject 从 GitHub 拉取
#       eProsima Micro-CDR 与 Micro-XRCE-DDS-Client，因此需要联网。
set -euo pipefail

_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DDS_WS="$(cd "${_dir}/../px4_ros_uxrce_dds_ws" && pwd)"

cd "${DDS_WS}"
echo "[build] 清理 ${DDS_WS}/{build,install,log}"
rm -rf build install log

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
echo "[build] colcon build --packages-select microxrcedds_agent"
colcon build --packages-select microxrcedds_agent
echo "[build] 完成：source ${DDS_WS}/install/setup.bash（scripts/env.sh 已自动包含）"
