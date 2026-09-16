#!/usr/bin/env bash
# 运行当前的单文件 Offboard 验证节点（后续会拆分为 px4_interface 等模块）
set -euo pipefail

_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${_dir}/env.sh"

exec ros2 run px4_offboard offboard_control "$@"
