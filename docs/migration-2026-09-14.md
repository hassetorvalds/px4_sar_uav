# 目录整合迁移记录（2026-09-14）

## 1. 迁移目标与结果

把散落在 `/home/sol/` 下的 PX4、Micro XRCE-DDS Agent、ROS 2 工作区与主文档，
整合进单一项目根目录 `/home/sol/px4_sar_uav/`，并完成重建与端到端验证。

| 迁移前 | 迁移后 |
|---|---|
| `/home/sol/PX4-Autopilot/` | `/home/sol/px4_sar_uav/PX4-Autopilot/` |
| `/home/sol/px4_ros_uxrce_dds_ws/` | `/home/sol/px4_sar_uav/px4_ros_uxrce_dds_ws/` |
| `/home/sol/ros2_px4_ws/` | `/home/sol/px4_sar_uav/ros2_px4_ws/` |
| `/home/sol/README.md` | `/home/sol/px4_sar_uav/README.md` |

PX4 分支与工作区均保持原状：`main`，`v1.18.0-beta1-442-gd0c6c0df7f`，未做任何分支切换或提交。

## 2. 为什么可以整体搬迁

1. **源码无绝对路径依赖**：三个目录中 `/home/sol` 的引用共 2397 处，全部位于
   `build/`、`install/`、`log/` 等可再生成的构建产物中，`src/` 与 PX4 源码内为 0 处。
2. **Git 元数据可搬迁**：PX4 的子模块 `.git` 文件使用相对 `gitdir: ../../.git/modules/...`，
   仓库配置中无绝对路径。
3. **同一文件系统**：`/home/sol` 与目标目录同在 `/dev/sdd`，搬迁是瞬时重命名（rename），
   不产生数据拷贝，出问题可秒级搬回。

## 3. 必须重建的原因

构建产物中存在**编译期烧入的绝对路径**，搬目录后旧产物不可用：

- PX4：可执行文件内编译进 `PX4_BINARY_DIR`（旧路径），首次启动时会因找不到
  `.../build/px4_sitl_default/etc` 而报 `Error opening startup file`；
  且 `build/px4_sitl_default/rootfs/etc` 是指向绝对路径的软链接。
- ROS 2：colcon 使用 `--symlink-install`，`install/` 内的头文件是指向
  `/home/sol/ros2_px4_ws/build/...` 的**绝对软链接**，迁移后全部失效。
- Micro XRCE-DDS Agent：可执行文件未记录 RPATH，依赖工作区 `setup.bash` 设置的
  `LD_LIBRARY_PATH`，**无需重编**，仅需修正 3 个 POSIX `sh` 变体脚本中的绝对前缀。

## 4. 实际执行步骤

```bash
# 1) 搬迁（瞬时）
mkdir -p /home/sol/px4_sar_uav
mv PX4-Autopilot ros2_px4_ws px4_ros_uxrce_dds_ws README.md /home/sol/px4_sar_uav/

# 2) 重建 PX4 SITL（会自动重新生成 rootfs 软链接）
cd /home/sol/px4_sar_uav/PX4-Autopilot && make px4_sitl

# 3) 重建 ROS 2 工作区
cd /home/sol/px4_sar_uav/ros2_px4_ws
rm -rf build install log && colcon build --symlink-install
```

以上 2、3 步已固化到 `scripts/build_px4_sitl.sh` 与 `scripts/build_ros_ws.sh`。

## 5. 验证结果

### 5.1 构建

| 项目 | 结果 |
|---|---|
| PX4 `make px4_sitl` | 成功，产出 `build/px4_sitl_default/bin/px4`，二进制内路径已更新为新目录 |
| PX4 rootfs 软链接 | 已重建为 `rootfs/etc -> /home/sol/px4_sar_uav/PX4-Autopilot/build/px4_sitl_default/etc` |
| ROS 2 `colcon build --symlink-install` | 成功，2 个包，耗时 5 分 31 秒（4 核并行受限下 px4_msgs 占 4 分 30 秒） |
| `ros2 pkg prefix px4_msgs / px4_offboard` | 均指向新路径 |
| 旧路径残留引用 | ROS 工作区 0 处、PX4 构建产物 0 处、Agent 工作区 0 处 |

### 5.2 运行链路

环境：Windows 侧 AirSim 在线，`PX4_SIM_HOST_ADDR=172.21.192.1`，`PX4_SYS_AUTOSTART=10016`（none_iris）。

```text
MicroXRCEAgent udp4 -p 8888              ✅ 运行
PX4 SITL 启动                            ✅
Simulator connected on TCP port 4560     ✅ AirSim 已接通
uxrce_dds_client synchronized            ✅ DDS 建链，全部 /fmu/out 数据写入器创建成功
Ready for takeoff!                       ✅
OFFBOARD (nav_state=14)                  ✅
ARM (arming_state=2)                     ✅
爬升并稳定在 3 m                          ✅ 位置 (0.00, 0.04, -2.99)（NED 单位为米）
trajectory_setpoint 频率                  ✅ 20.000 Hz，min=0.050s max=0.050s
accepts_offboard_setpoints / preflight    ✅ true / true，failsafe=false
```

与迁移前 README 中记录的验证状态一致，说明整合未破坏已跑通的链路。

### 5.3 迁移过程中遇到的问题

1. **PX4 重建需要联网**：编译 `uxrce_dds_client` 依赖 ExternalProject 从 GitHub 拉取
   eProsima Micro-CDR 与 Micro-XRCE-DDS-Client。离线环境需预先缓存这两个仓库，
   或允许首次构建联网。
2. **Agent 的 `sh` 变体脚本**：`install/setup.sh`、`install/local_setup.sh`、
   `install/microxrcedds_agent/share/microxrcedds_agent/package.sh` 内写死旧前缀，
   已就地修正；bash 变体（`setup.bash`）本身是相对路径解析，不受影响。

## 6. 已知问题（遗留项，建议下一阶段处理）

- 本次仅验证到 OFFBOARD / ARM / 3 m 悬停与手动方式收尾（`commander disarm -f`）。
  `commander land` 已下发并进入 `AUTO_LAND`（nav_state=18），但落地后估计器出现
  `Preflight Fail: High Accelerometer Bias`、`vertical velocity unstable`，
  `vehicle_land_detected` 未置位，local position 的 z 也出现漂移。
  这表明**自动降落与低速段估计器行为需要专项调试**，与 README 中将“自动降落”
  列为待办项相互印证。该次飞行的完整日志已另存到
  `logs/2026-09-14/10_54_06.ulg`（约 71 MB，PX4 同时写在
  `PX4-Autopilot/build/px4_sitl_default/rootfs/log/` 下，但重建会清空该目录）。
- 坐标系约定：ROS 2 侧的 `/fmu/out/vehicle_local_position_v1` 与
  `/fmu/out/vehicle_odometry` 与 PX4 uORB 数值一致（未做 NED→ENU 转换），
  进入感知/规划阶段前必须统一约定并写成单元测试。

## 7. 回退方式

```bash
cd /home/sol/px4_sar_uav
mv PX4-Autopilot ros2_px4_ws px4_ros_uxrce_dds_ws README.md /home/sol/
```

搬回后同样需要按第 4 节重建，因为构建产物中的绝对路径又变了。

## 8. 日常约定（重要）

> **移动即重编译。** 只要这三个目录换了路径，就必须重新执行
> `scripts/build_px4_sitl.sh` 与 `scripts/build_ros_ws.sh`，
> 不要尝试直接复用旧的 `build/` 与 `install/`。
