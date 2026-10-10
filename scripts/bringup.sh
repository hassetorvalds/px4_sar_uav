#!/usr/bin/env bash
# 统一启动入口：一条命令拉起仿真栈（等价于逐个跑 run_*.sh）
#
# 用法：
#   scripts/bringup.sh                                  # 接口+传感器+地图+规划
#   scripts/bringup.sh with_mission:=true               # 追加航点任务
#   scripts/bringup.sh with_explorer:=true              # 追加自主探索
#   scripts/bringup.sh airsim_host:=172.21.192.1 camera_rate_hz:=2.5
#
# 前置（另开终端）：
#   scripts/run_agent.sh       # Micro XRCE-DDS Agent
#   scripts/run_px4_sitl.sh    # PX4 SITL（含手动接管 MAVLink 实例）
set -euo pipefail

_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${_dir}/env.sh"

exec ros2 launch bringup sim_stack.launch.py "$@"
