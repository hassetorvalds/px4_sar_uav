#!/usr/bin/env bash
# 新执行器/覆盖专用：前视 + 下视双相机
exec bash "$(dirname "$0")/bringup.sh" with_arbiter:=false with_down_camera:=true camera_rate_hz:=2.5 "$@"
