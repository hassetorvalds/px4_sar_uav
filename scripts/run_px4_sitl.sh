#!/usr/bin/env bash
# 启动 PX4 SITL，并连接到 Windows 侧的 AirSim
set -euo pipefail

_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${_dir}/env.sh"

# 确保键盘手动接管所需的专用 MAVLink 实例存在（幂等；重建 PX4 后自动补上）
"${_dir}/enable_manual_link.sh"

cd "${PX4_DIR}"
exec ./build/px4_sitl_default/bin/px4 "$@"
