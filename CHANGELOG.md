# 变更日志

本文件记录本项目**所有**值得留痕的变更：新增功能、结构性调整、验证结果、
已知问题与移除内容。格式参考 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，
按日期倒序排列，最新在最上方。

## 记录规范（后续所有改动都按此写）

1. **每次提交/合入前**：在顶部新增或补充当天条目，不要事后补记。
2. 分类固定为：`新增 / 变更 / 修复 / 移除 / 验证 / 已知问题`，没有内容的小节可省略。
3. 每条都要**可验证**：写清命令、实测数值或文件路径，避免“已优化”“已完善”这类空话。
4. 涉及飞控、接管、failsafe 的改动，必须在 `验证` 中给出触发条件与结果；只跑通正常流程不算验证。
5. 涉及目录搬迁、依赖升级的改动，必须写明**是否需要重建**以及重建命令。
6. 版本基线变化（PX4 / px4_msgs / Agent / ROS 2）单独在 `变更` 中列出前后 commit 或版本号。

---

## [2026-09-16]

### 新增

- 版本库初始化：项目根目录建立 Git 仓库，主分支 `main`，首个提交 `52daa86`
  （25 个文件，仓库对象 144 KiB，提交树 0.07 MB）。
  - `.gitignore`：排除构建产物（`build/`、`install/`、`log/`）、飞行与仿真数据
    （`logs/`、`*.ulg`、`*.bag`、`*.db3`、`*.mcap`）、Python 缓存与编辑器目录。
  - `.gitmodules`：三个第三方依赖以 submodule 固定版本，源码不入库：
    - `PX4-Autopilot` → `PX4/PX4-Autopilot` @ `d0c6c0df7f`（main 分支）
    - `ros2_px4_ws/src/px4_msgs` → `PX4/px4_msgs` @ `598c7aa`（release/1.18 分支）
    - `px4_ros_uxrce_dds_ws/src/Micro-XRCE-DDS-Agent` → `eProsima/Micro-XRCE-DDS-Agent` @ `57d0862`（v2.4.2）
- `scripts/build_dds_ws.sh`：重建 Micro XRCE-DDS Agent 工作区。
  Agent 的 `install/`（含 `MicroXRCEAgent` 可执行文件）不入库，
  新克隆的仓库必须执行本脚本才能得到 Agent；首次构建需要联网。
- README 新增「5.1 从远程仓库克隆与重建」：从零克隆 → 初始化 submodule →
  重建三套构建 → 启动链路，并说明 `--recursive` 的必要性。

### 变更

- 推送 `main` 到远程仓库 `https://github.com/hassetorvalds/px4_sar_uav.git`，
  本地 `main` 已跟踪 `origin/main`。
- 建立开发分支 `dev`（自 `main` 牵出）：后续功能开发在 `dev` 进行，稳定后合回 `main`。
- README 第 5 节目录树补充 `CHANGELOG.md`、`logs/` 与新增脚本条目。
- README 新增「5.2 分支约定」：`main`（稳定基线，只接受合并）/ `dev`（集成）/
  `feature/*` / `hotfix/*` 的分工，分支命名、提交信息前缀、CHANGELOG 同步要求，
  以及 submodule 指针变更与禁入库内容的约束。

### 验证

- `git ls-remote origin`：`refs/heads/main` 与本地 `HEAD` 同为 `52daa86`，工作区干净。
- 入库体积核对：提交树 0.07 MB、仓库对象 144 KiB；3.5 GB 构建产物与 71 MB 飞行日志均未入库。

### 已知问题

- README 5.1 的「全新克隆 → 重建」路径尚未在干净环境完整跑通。
  `build_dds_ws.sh` 的构建逻辑与已实际执行过的命令一致（`colcon build --packages-select
  microxrcedds_agent`），但本机 Agent 工作区是既有构建，未做从零重跑验证；
  首次在新机器使用若报错，请在本文件补充记录。

---

## [2026-09-14]

### 新增

- `scripts/`：项目统一脚本集
  - `env.sh`：统一环境入口，导出 `PX4_SAR_ROOT` / `PX4_DIR` / `ROS_WS` / `DDS_WS`，
    按顺序 source ROS 2、Agent、本工作区，并自动探测 `PX4_SIM_HOST_ADDR`（默认网关，失败时回退 `/etc/resolv.conf`）。
  - `run_agent.sh`、`run_px4_sitl.sh`、`run_offboard.sh`：启动 Agent / PX4 SITL / Offboard 验证节点。
  - `check_stack.sh`：链路冒烟检查（`/fmu/out` 话题存在性 + 频率）。
  - `build_ros_ws.sh`、`build_px4_sitl.sh`：换目录或改依赖后的重建脚本。
- `docs/roadmap.md`：后续开发路线图（工程前置 + 阶段 A–H，含依赖、工期、验收标准、风险）。
- `docs/migration-2026-09-14.md`：目录整合迁移记录（原因、步骤、验证、回退）。
- `logs/2026-09-14/10_54_06.ulg`：本次端到端验证的 PX4 飞行日志（约 71 MB），
  与 `build/` 解耦保存，避免重建时丢失。
- `CHANGELOG.md`：本文件。

### 变更

- **项目整合为单一目录**：`PX4-Autopilot/`、`px4_ros_uxrce_dds_ws/`、`ros2_px4_ws/`、`README.md`
  由 `/home/sol/` 根下搬迁至 `/home/sol/px4_sar_uav/`。
  - 搬迁方式：同一文件系统内的 `mv`（瞬时 rename，无数据拷贝），可秒级回退。
  - PX4 分支与提交**未改动**：`main`，`v1.18.0-beta1-442-gd0c6c0df7f`。
  - 版本基线：PX4 `main@d0c6c0df7f`、px4_msgs `598c7aa`（对应固件 `993f542f`，release/1.18 线）、
    Micro XRCE-DDS Agent `v2.4.2`、ROS 2 Humble。
- `README.md`：第 4 节改为准确的「已实现 / 未实现 / 已知问题」清单；
  第 5 节目录结构更新为整合后的真实结构，并写明“移动即重编译”的约定。
- `px4_ros_uxrce_dds_ws/install/setup.sh`、`install/local_setup.sh`、
  `install/microxrcedds_agent/share/microxrcedds_agent/package.sh`：
  修正写死的旧工作区绝对前缀（仅 POSIX `sh` 变体受影响，`setup.bash` 本身为相对解析）。

### 移除

- `/tmp/px4_old_build/`（约 1.7 GB）：搬迁前的 PX4 构建产物，含旧路径的绝对引用，迁移后已失效。

### 验证

重建（换目录后必须执行）：

| 项目 | 结果 |
|---|---|
| `make px4_sitl`（PX4-Autopilot） | 成功；二进制内路径更新为新目录；`rootfs/etc` 软链接重建 |
| `colcon build --symlink-install`（ros2_px4_ws） | 成功，2 个包，5 分 31 秒 |
| 旧路径残留引用 | ROS 工作区 0 处、PX4 构建产物 0 处、Agent 工作区 0 处 |

端到端飞行（Windows 侧 AirSim 在线，`PX4_SIM_HOST_ADDR=172.21.192.1`，`PX4_SYS_AUTOSTART=10016`）：

```text
Simulator connected on TCP port 4560      ✅
uxrce_dds_client synchronized             ✅ 全部 /fmu/out 数据写入器创建成功
Ready for takeoff!                        ✅
OFFBOARD                                  ✅ nav_state=14
ARM                                       ✅ arming_state=2
3 m 悬停                                  ✅ 实测 (0.00, 0.04, -2.99)
trajectory_setpoint 频率                   ✅ 20.000 Hz（min=max=0.050 s）
accepts_offboard_setpoints / preflight     ✅ true / true，failsafe=false
```

结论：整合未破坏既有链路，验证状态与搬迁前一致。

### 已知问题

- **自动降落未通过验证**：下发 `commander land` 后进入 `AUTO_LAND`（`nav_state=18`），
  但落地后出现 `Preflight Fail: High Accelerometer Bias`、`Preflight Fail: vertical velocity unstable`，
  `vehicle_land_detected.landed` 始终未置位，local position 的 z 由 -3 漂移至约 +2.2，
  最终以 `commander disarm -f` 强制收尾。取证日志见 `logs/2026-09-14/10_54_06.ulg`，
  安排为下一阶段（路线图阶段 A）的首要调试项。
- **坐标系约定待固化**：`/fmu/out/vehicle_local_position_v1` 与 `/fmu/out/vehicle_odometry`
  的数值与 PX4 uORB 一致（DDS 侧未做 NED→ENU 转换），进入感知/规划前需统一定义并配单元测试。

### 回退方式

```bash
cd /home/sol/px4_sar_uav
mv PX4-Autopilot ros2_px4_ws px4_ros_uxrce_dds_ws README.md /home/sol/
```

搬回后同样需要重新构建（`scripts/build_px4_sitl.sh` 与 `scripts/build_ros_ws.sh`），
因为构建产物中烧入了绝对路径。
