#!/usr/bin/env bash
# 旧驱动专用：前视单相机（preflight 要求 airsim_camera 恰好 1 个）
exec bash "$(dirname "$0")/bringup.sh" with_arbiter:=false with_down_camera:=false camera_rate_hz:=2.5 "$@"
