#!/usr/bin/env bash
# 重建 ROS 2 工作区（换目录、拖动工程后必须执行）
#
# 说明：colcon 的 install/ 目录里含有指向 build/ 的绝对路径软链接，
#       工作区一旦移动位置就必须清空 build/install/log 重新编译。
set -euo pipefail

_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS="$(cd "${_dir}/../ros2_px4_ws" && pwd)"

cd "${ROS_WS}"
echo "[build] 清理 ${ROS_WS}/{build,install,log}"
rm -rf build install log

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
echo "[build] colcon build --symlink-install"
colcon build --symlink-install
echo "[build] 完成：source ${ROS_WS}/install/setup.bash"
