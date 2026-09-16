#!/usr/bin/env bash
# 启动 PX4 SITL，并连接到 Windows 侧的 AirSim
set -euo pipefail

_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${_dir}/env.sh"

cd "${PX4_DIR}"
exec ./build/px4_sitl_default/bin/px4 "$@"
