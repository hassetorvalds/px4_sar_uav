#!/usr/bin/env bash
# 启动 Micro XRCE-DDS Agent（ROS 2 与 PX4 之间的桥梁）
set -euo pipefail

_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${_dir}/env.sh"

exec MicroXRCEAgent udp4 -p 8888 "$@"
