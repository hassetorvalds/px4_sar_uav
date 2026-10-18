#!/usr/bin/env bash
# 等 AirSim 的 PX4 TCP 端口回到 Listen，然后**直接**启动 PX4。
#
# 背景（2026-10-18 实测）：AirSim 的 4560 是"单次 accept"——任何探测连接
# （包括 `echo > /dev/tcp/...`、`nc -z`）都会占掉 accept 槽，连接一关就变 CloseWait，
# 之后的 PX4 连接全部 SYN-SENT。所以这里**只读 Windows 的 socket 状态**，绝不 connect。
#
# 用法：先跑本脚本（后台），再在 UE 里 Stop → Play；检测到 Listen 后自动起 PX4。
set -uo pipefail

_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${_dir}/env.sh" >/dev/null 2>&1

_ps='/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe'
_timeout_s="${WAIT_TIMEOUT_S:-300}"
_interval="${WAIT_INTERVAL_S:-2}"

echo "[wait] 等待 AirSim 4560 进入 Listen（不连接端口）…"
for _ in $(seq 1 $((_timeout_s / _interval))); do
    state="$("${_ps}" -NoProfile -Command \
        "(Get-NetTCPConnection -LocalPort 4560 -State Listen -ErrorAction SilentlyContinue | Select-Object -First 1).State" \
        2>/dev/null | tr -d '\r\n[:space:]')"
    if [ "${state}" = "Listen" ]; then
        echo "[wait] 4560 已 Listen → 启动 PX4"
        cd "${PX4_SAR_ROOT}" || exit 1
        exec env PX4_SIM_HOST_ADDR="${PX4_SIM_HOST_ADDR:-172.21.192.1}" \
            bash "${_dir}/run_px4_sitl.sh"
    fi
    sleep "${_interval}"
done

echo "[wait] 超时（${_timeout_s}s）仍未见 Listen；请在 UE 里 Stop → Play 后重试" >&2
exit 1
