#!/usr/bin/env bash
# 处理"Windows 保留端口段挡住了 AirSim 的 ApiServerPort"这一**本机环境问题**。
#
# 背景（2026-10-15）：Hyper-V/WSL 每次开机随机预留一段 TCP 端口；本轮 44986–45085
# 压到了 AirSim 的 `ApiServerPort=45000` → AirSim 启动即报
# `Error at startup: bind: <…权限不允许…>`（在 Windows 上直接 bind 45000 也会失败，
# 而换个空闲端口立刻成功 → 与 AirSim 本身无关）。
#
# 用法：
#   scripts/win_port_exclusion.sh status          # 只看：保留段 + 真实 bind 测试（默认）
#   scripts/win_port_exclusion.sh apply           # 修：把该端口设为**持久静态保留**（需 UAC 确认）
#   scripts/win_port_exclusion.sh revert          # 撤销：删除上面那条静态保留（需 UAC 确认）
#
# 端口可用环境变量覆盖：AIRSIM_PORT=45000 scripts/win_port_exclusion.sh status
#
# ⚠️ 这是**临时**环境改动：按项目要求，实验结束、用户确认后执行 `revert` 撤销。
set -uo pipefail

PORT="${AIRSIM_PORT:-45000}"
SYSROOT="/mnt/c/Windows/System32"
NETSH="${SYSROOT}/netsh.exe"
PS="${SYSROOT}/WindowsPowerShell/v1.0/powershell.exe"

if [ ! -x "$NETSH" ] || [ ! -x "$PS" ]; then
    echo "找不到 Windows 互操作入口（$NETSH / $PS）——本脚本只在 WSL 里用。" >&2
    exit 1
fi

show_ranges() {
    echo "== 当前 TCP 保留段（netsh int ipv4 show excludedportrange protocol=tcp）=="
    "$NETSH" int ipv4 show excludedportrange protocol=tcp 2>/dev/null | tr -d '\r'
    echo "== UDP 保留段 =="
    "$NETSH" int ipv4 show excludedportrange protocol=udp 2>/dev/null | tr -d '\r' | head -12
}

# 非破坏性的"真实绑定"测试：试完立刻释放（这是复现 AirSim 报错的等价实验）
bind_test() {
    # [Console]::OutputEncoding=UTF8：否则中文错误信息会变成 ???（Windows 控制台默认代码页）
    "$PS" -NoProfile -Command "[Console]::OutputEncoding=[System.Text.Encoding]::UTF8; try { \$l = [System.Net.Sockets.TcpListener]::new([System.Net.IPAddress]::Any, $PORT); \$l.Start(); Write-Output ('PORT $PORT = BIND OK 可绑定'); \$l.Stop() } catch { Write-Output ('PORT $PORT = BIND FAIL 仍被挡住: ' + \$_.Exception.Message) }" 2>&1 | tr -d '\r' | head -3
}

# 需要管理员：用 UAC 弹窗跑一条 cmd 复合命令（stop winnat → netsh → start winnat）
run_elevated() {
    local action="$1"   # add | delete
    local inner="net stop winnat & netsh int ipv4 ${action} excludedportrange protocol=tcp startport=${PORT} numberofports=1 & net start winnat"
    echo "== 申请管理员权限执行：${inner} =="
    echo "（Windows 会弹出 UAC 确认框，请点\"是\"）"
    "$PS" -NoProfile -Command "Start-Process -Verb RunAs -Wait -FilePath cmd.exe -ArgumentList '/c', '$inner'" 2>&1 | tr -d '\r'
}

case "${1:-status}" in
    status)
        show_ranges
        echo
        bind_test
        ;;
    apply)
        show_ranges
        echo
        bind_test
        echo
        run_elevated add
        sleep 2
        echo
        echo "== 修复后复验 =="
        bind_test
        echo
        echo "若上面是 BIND OK，就可以启动 AirSim 了。"
        echo "⚠️ 实验结束后请执行：scripts/win_port_exclusion.sh revert"
        ;;
    revert)
        run_elevated delete
        sleep 2
        echo
        echo "== 撤销后复验（此时若又被随机段压到，BIND FAIL 属正常）=="
        bind_test
        echo
        echo "已删除静态保留；下次开机 Hyper-V/WSL 会重新随机预留端口段。"
        ;;
    *)
        echo "用法：$0 [status|apply|revert]（AIRSIM_PORT 可覆盖端口，默认 45000）" >&2
        exit 2
        ;;
esac
