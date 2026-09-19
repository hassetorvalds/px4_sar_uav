#!/usr/bin/env bash
# 统一环境入口：source scripts/env.sh
#
# 约定：
#   PX4_SAR_ROOT  项目根目录
#   PX4_DIR       PX4 固件源码与 SITL 构建目录
#   ROS_WS        ROS 2 工作区（px4_msgs / px4_offboard 及后续模块）
#   DDS_WS        Micro XRCE-DDS Agent 工作区
#
# 注意：本文件必须被 source，不要直接执行。

_env_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_SAR_ROOT="$(cd "${_env_dir}/.." && pwd)"

# ROS 2 / colcon 的 setup.bash 会引用未定义变量，若调用方开启了 set -u 会直接报错，
# 这里在 source 期间临时关闭 -u，结束后恢复，使 env.sh 可被任何脚本安全引用。
_env_restore_u=0
case "$-" in
    *u*) _env_restore_u=1; set +u ;;
esac

export PX4_SAR_ROOT
export PX4_DIR="${PX4_SAR_ROOT}/PX4-Autopilot"
export ROS_WS="${PX4_SAR_ROOT}/ros2_px4_ws"
export DDS_WS="${PX4_SAR_ROOT}/px4_ros_uxrce_dds_ws"

# 1) ROS 2 基础环境
if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
else
    echo "[env] 警告：未找到 /opt/ros/humble/setup.bash" >&2
fi

# 2) Micro XRCE-DDS Agent（提供 MicroXRCEAgent 及其 .so 的搜索路径）
if [ -f "${DDS_WS}/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "${DDS_WS}/install/setup.bash"
else
    echo "[env] 警告：${DDS_WS}/install 未构建，先运行 scripts/build_ros_ws.sh" >&2
fi

# 3) 本项目 ROS 2 工作区（px4_msgs / px4_offboard）
if [ -f "${ROS_WS}/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "${ROS_WS}/install/setup.bash"
else
    echo "[env] 警告：${ROS_WS}/install 未构建，先运行 scripts/build_ros_ws.sh" >&2
fi

# 4) SITL 启动参数：AirSim 跑在 Windows 侧，PX4 需要知道对端地址
export PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-10016}"   # 10016 = none_iris
if [ -z "${PX4_SIM_HOST_ADDR:-}" ]; then
    # 优先用默认网关（WSL2 NAT 模式下即 Windows 主机），退回 DNS 配置里的主机地址
    # 注意：沙箱/受限环境里 ip 命令可能返回非零，用 || true 保证在 set -e -o pipefail 下不中断
    PX4_SIM_HOST_ADDR="$({ ip route show 2>/dev/null || true; } | awk '/default/ {print $3; exit}')"
    if [ -z "${PX4_SIM_HOST_ADDR}" ] && [ -r /etc/resolv.conf ]; then
        PX4_SIM_HOST_ADDR="$(awk '/^nameserver/ {print $2; exit}' /etc/resolv.conf 2>/dev/null)"
    fi
    export PX4_SIM_HOST_ADDR
fi

echo "[env] PX4_SAR_ROOT=${PX4_SAR_ROOT}"
if [ -z "${PX4_SIM_HOST_ADDR}" ]; then
    echo "[env] 警告：未能自动探测 Windows 主机地址，请手动 export PX4_SIM_HOST_ADDR=<Windows IP>" >&2
else
    echo "[env] PX4_SIM_HOST_ADDR=${PX4_SIM_HOST_ADDR}  PX4_SYS_AUTOSTART=${PX4_SYS_AUTOSTART}"
fi

if [ "${_env_restore_u}" = "1" ]; then
    set -u
fi
unset _env_restore_u
