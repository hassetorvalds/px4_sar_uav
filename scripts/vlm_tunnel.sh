#!/usr/bin/env bash
# 一键管理到 LightNav-0 服务的 SSH 隧道：本机 127.0.0.1:18080 → 服务器 127.0.0.1:18080
#
# 用法：
#   scripts/vlm_tunnel.sh start     # 建立（幂等；已在运行则直接报健康状态）
#   scripts/vlm_tunnel.sh stop      # 关闭
#   scripts/vlm_tunnel.sh status    # 看进程 + GET /health
#
# 可用环境变量覆盖：VLM_TUNNEL_HOST / VLM_TUNNEL_KEY / VLM_TUNNEL_PORT /
#                  VLM_TUNNEL_REMOTE_PORT / VLM_TUNNEL_LOG
#
# 背景：隧道是长连接，跨天/断网/机器休眠后会消失（2026-10-12 实测掉过一次），
# 掉了以后客户端会报"远端服务不可达（连接被拒绝）"，先跑 status 再 start 即可。
set -uo pipefail

_host="${VLM_TUNNEL_HOST:-sol@10.255.248.205}"
_key="${VLM_TUNNEL_KEY:-${HOME}/.ssh/id_ed25519}"
_port="${VLM_TUNNEL_PORT:-18080}"
_remote_port="${VLM_TUNNEL_REMOTE_PORT:-18080}"
_log="${VLM_TUNNEL_LOG:-/tmp/vlm_tunnel.log}"
_pattern="ssh -N .*-L ${_port}:127.0.0.1:${_remote_port}"

health () {
    local body
    body="$(curl -s -m 5 "http://127.0.0.1:${_port}/health" 2>/dev/null)"
    if [ -n "$body" ]; then
        echo "[tunnel] /health → ${body}"
        return 0
    fi
    echo "[tunnel] /health 无响应（服务或隧道不在）"
    return 1
}

case "${1:-status}" in
    start)
        if pgrep -f "$_pattern" >/dev/null; then
            echo "[tunnel] 已有隧道进程，直接检查健康状态"
        else
            setsid nohup ssh -N \
                -o BatchMode=yes -o StrictHostKeyChecking=no \
                -o ServerAliveInterval=30 -o ServerAliveCountMax=3 \
                -o ExitOnForwardFailure=yes \
                -i "$_key" \
                -L "${_port}:127.0.0.1:${_remote_port}" "$_host" \
                >"$_log" 2>&1 </dev/null &
            sleep 3
        fi
        pgrep -f "$_pattern" >/dev/null && echo "[tunnel] 进程在线（日志 ${_log}）" \
            || { echo "[tunnel] 启动失败，见 ${_log}"; tail -3 "$_log" 2>/dev/null; exit 1; }
        health
        ;;
    stop)
        if pgrep -f "$_pattern" >/dev/null; then
            pkill -f "$_pattern" && echo "[tunnel] 已关闭"
        else
            echo "[tunnel] 没有在跑的隧道"
        fi
        ;;
    status)
        pgrep -af "$_pattern" || echo "[tunnel] 没有在跑的隧道"
        health
        ;;
    *)
        echo "用法：$0 {start|stop|status}" >&2
        exit 2
        ;;
esac
