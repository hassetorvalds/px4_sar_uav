#!/usr/bin/env bash
# Qwen 自然语言导航入口：目标 grounding → TaskSpec → 可选 dynamic → fly_task。
# 前提：PX4/ROS 栈已启动；18081 隧道已建立；Qwen 服务 ready。
set -euo pipefail

_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${_dir}/env.sh" >/dev/null 2>&1
cd "${PX4_SAR_ROOT}"

dynamic=0
target_only=0
instruction=""
for arg in "$@"; do
  case "$arg" in
    --dynamic) dynamic=1 ;;
    --target-only) target_only=1 ;;
    -h|--help)
      echo "用法：scripts/run_qwen_nav.sh [--dynamic] [--target-only] [指令]"; exit 0 ;;
    *) instruction="$arg" ;;
  esac
done
instruction="${instruction:-飞到前方橙色目标附近}"

if ! curl -sf -m 5 http://127.0.0.1:18081/v1/models >/dev/null; then
  echo "[qwen-nav] 18081 不可达；先建立 Qwen 隧道（-L 18081:127.0.0.1:18081）" >&2
  exit 2
fi

stamp="$(date +%Y%m%d-%H%M%S)"
task="logs/tasks/qwen_nav_${stamp}.json"
echo "[qwen-nav] 指令：${instruction}"
python3 scripts/qwen_target.py --instruction "${instruction}" --out "${task}"

provider_pid=""
cleanup() {
  if [[ -n "${provider_pid}" ]]; then
    kill "${provider_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

if [[ "${target_only}" == "1" ]]; then
  echo "[qwen-nav] target-only：${task}"
  exit 0
fi

if [[ "${dynamic}" == "1" ]]; then
  python3 - "${task}" <<'PY'
import json, sys
path = sys.argv[1]
task = json.load(open(path, encoding='utf-8'))
for step in task['steps']:
    if step['type'] == 'goto':
        step['params']['dynamic_target'] = True
task['source_text'] += ' [dynamic Qwen]'
json.dump(task, open(path, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
PY
  python3 scripts/qwen_target_pub.py --ros-args \
    -p instruction:="${instruction}" -p rate_hz:=2.0 \
    > "logs/tasks/qwen_nav_${stamp}_provider.log" 2>&1 &
  provider_pid=$!
  echo "[qwen-nav] provider pid=${provider_pid}，等待 /qwen/target"
  if ! timeout 8 ros2 topic echo /qwen/target --once >/dev/null 2>&1; then
    echo "[qwen-nav] provider 未在 8 s 内发布目标" >&2
    exit 3
  fi
fi

echo "[qwen-nav] 执行 TaskSpec：${task}"
FLY_TASK_CONFIRM=1 scripts/fly_task.py --task-json "${task}" --execute
