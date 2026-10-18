#!/usr/bin/env bash
# 启动 ExecuteTask Action 服务器（需要执行器已在线并发布 /mission/status）。
set -euo pipefail
_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${_dir}/env.sh" >/dev/null 2>&1
exec ros2 run mission task_action_server "$@"
