# 基于 ROS 2 + PX4 + AirSim + VLM 的灾害搜救无人机系统

> 面向灾害搜救场景的语言驱动、人机协同、安全自主四旋翼无人机系统规划文档  
> 当前仿真平台：**Windows 11 + UE4.27 + AirSim 1.8.1 + WSL2 Ubuntu 22.04 + ROS 2 Humble + PX4 1.18 + Micro XRCE-DDS Agent 2.4.2**
>
> 进度速览：第 4 节「已实现 / 未实现」· 变更记录：[CHANGELOG.md](CHANGELOG.md) · 开发排期：[docs/roadmap.md](docs/roadmap.md)

---

## 1. 项目最终目标

本项目的最终目标是构建一个面向**灾害搜救**的自主无人机系统，具备以下能力：

1. **自然语言任务输入**
   - 操作人员使用自然语言描述任务，例如：
     - “搜索前方倒塌建筑区域。”
   - 使用视觉语言模型（VLM）理解任务、环境和语义目标。

2. **自然语言驱动的任务规划**
   - VLM 不直接控制电机，也不直接产生高频飞控指令。
   - VLM 输出结构化任务、搜索区域、目标类别、优先级和行为约束。
   - Mission Planner 将其转换为可执行的任务状态机和导航目标。

3. **自主飞行**
   - ROS 2 通过 PX4 Offboard 接口发送轨迹/位置目标。
   - Global Planner 负责大尺度航线和航点规划。
   - Local Planner 负责局部轨迹生成。

4. **实时避障，而且不经过 VLM**
   - RGB-D / 深度相机 / LiDAR 等高频传感器数据进入局部地图。
   - 局部规划器在高频循环中实时检测障碍并重新规划。
   - VLM 不进入实时避障控制回路。

5. **人工随时接管**
   - 自主飞行过程中，操作手可以随时通过 RC/遥控器接管。
   - 人工控制权优先级高于自主规划和 VLM。
   - 即使 ROS 2、VLM、规划器出现异常，操作手仍应尽可能通过 PX4 原生手动控制机制接管。

6. **灾害搜救**
   - 在自主探索、避障、语义理解和人工接管全部稳定后，再加入被困人员识别。
   - 目标是形成：
     - 区域搜索
     - 目标发现
     - 语义确认
     - 靠近/悬停
     - 信息记录
     - 结果报告

---

# 2. 总体系统架构

最终系统推荐采用“**慢速语义决策 + 快速几何规划 + PX4 安全控制 + 人工高优先级接管**”的分层架构。

```text
                         ┌─────────────────────┐
                         │      操作人员        │
                         │  自然语言 / 遥控器   │
                         └─────────┬───────────┘
                                   │
                       ┌───────────┴───────────┐
                       │                       │
                       ▼                       ▼
                ┌──────────────┐       ┌──────────────┐
                │ VLM / LLM    │       │ RC / Manual  │
                │ 任务理解      │       │ 人工接管      │
                └──────┬───────┘       └──────┬───────┘
                       │                      │
                       ▼                      │
                ┌──────────────┐              │
                │ Mission      │              │
                │ Planner      │              │
                └──────┬───────┘              │
                       │                      │
                       ▼                      │
                ┌──────────────┐              │
                │ Global       │              │
                │ Planner      │              │
                └──────┬───────┘              │
                       │                      │
                       ▼                      │
             ┌────────────────────┐            │
             │ Local Planner      │◄───────────┤
             │ 实时避障 / 重规划   │            │
             └─────────┬──────────┘            │
                       │                       │
                       ▼                       │
                ┌──────────────┐               │
                │ Safety       │◄──────────────┘
                │ Supervisor   │
                └──────┬───────┘
                       │
                       ▼
                ┌──────────────┐
                │ PX4          │
                │ Flight Stack │
                └──────┬───────┘
                       │
                       ▼
                ┌──────────────┐
                │ AirSim 1.8.1 │
                │ UE4.27       │
                └──────────────┘
```

核心原则：

> **VLM 决定“做什么、去哪里、为什么去”；规划器决定“怎么走”；PX4 决定“怎么稳定、安全地飞”；人工操作手拥有最高接管优先级。**

---

# 3. 三条独立控制回路

最终系统建议拆成三条回路。

## 3.1 慢速语义回路：VLM

```text
自然语言
   ↓
VLM
   ↓
场景理解 / 语义目标 / 任务分解
   ↓
Mission Planner
```

特点：

- 低频运行
- 不直接进入电机控制
- 可以承受一定延迟
- 可以失败并重启
- 不应该承担实时避障职责

主要任务：

- 理解自然语言
- 判断任务目标
- 识别语义目标
- 调整搜索策略
- 解释视觉结果
- 触发任务状态变化

---

## 3.2 快速几何回路：实时避障

```text
RGB-D / Depth / LiDAR
          ↓
      Local Map
          ↓
     Local Planner
          ↓
   Trajectory Setpoint
          ↓
          PX4
```

特点：

- 高频运行
- 不经过 VLM
- 对障碍物变化快速响应
- 负责局部重规划

目标：

> 即使 VLM 正在思考，前方出现墙、建筑、树木或其他动态障碍时，飞机仍然能够自主避开。

---

## 3.3 高优先级安全回路：人工接管

```text
遥控器 / RC
     ↓
   PX4
     ↓
Manual Control
```

人工控制不应依赖：

```text
RC → ROS 2 → VLM → Planner → PX4
```

而应尽可能走：

```text
RC → PX4
```

这样即使：

- ROS 2 崩溃
- VLM 崩溃
- GPU 崩溃
- 规划器崩溃
- 网络通信中断

也不应阻断人工控制。

最终优先级建议：

```text
人工接管
   >
安全约束
   >
局部避障
   >
全局规划
   >
任务规划
   >
VLM
```

---

# 4. 当前已经完成的基础平台

> 变更记录见 [CHANGELOG.md](CHANGELOG.md)；后续开发排期见 [docs/roadmap.md](docs/roadmap.md)。

## 4.1 运行环境（已就绪）

```text
Windows 11（宿主）
│
├── UE4.27
├── AirSim 1.8.1
│
└── WSL2 Ubuntu 22.04（开发侧）
    │
    ├── ROS 2 Humble
    ├── PX4 1.18（main 分支，v1.18.0-beta1-442-gd0c6c0df7f）
    ├── Micro XRCE-DDS Agent 2.4.2
    └── px4_msgs（commit 598c7aa，对应固件 993f542f）
```

## 4.2 项目结构（已整合到单一目录）

全部内容位于 `/home/sol/px4_sar_uav/`，详见第 5 节。
迁移过程与注意事项记录在 `docs/migration-2026-09-14.md`。

## 4.3 已实现并验证的能力

| 能力 | 状态 | 实测证据 |
|---|---|---|
| AirSim ↔ PX4 SITL 仿真链路 | ✅ | `Simulator connected on TCP port 4560` |
| PX4 ↔ uXRCE-DDS ↔ ROS 2 桥接 | ✅ | `uxrce_dds_client synchronized`，全部 `/fmu/out` 数据写入器创建成功 |
| ROS 2 读取 PX4 状态 | ✅ | `/fmu/out/vehicle_odometry`、`vehicle_attitude`、`vehicle_status_v4`、`vehicle_local_position_v1` |
| ROS 2 向 PX4 下发设定点与指令 | ✅ | `/fmu/in/offboard_control_mode`、`trajectory_setpoint`（20.000 Hz）、`vehicle_command` |
| OFFBOARD 模式 | ✅ | `nav_state=14` |
| ARM | ✅ | `arming_state=2`，`accepts_offboard_setpoints=true` |
| 3 m 悬停 | ✅ | 实测位置 `(0.00, 0.04, -2.99)`，`failsafe=false`，`pre_flight_checks_pass=true` |
| 集成脚本（环境/启动/巡检/重建） | ✅ | `scripts/env.sh`、`run_agent.sh`、`run_px4_sitl.sh`、`run_offboard.sh`、`check_stack.sh`、`build_px4_sitl.sh`、`build_ros_ws.sh` |
| 飞行日志留存 | ✅ | `logs/2026-09-14/10_54_06.ulg` |
| 航点任务状态机 | ✅ | `mission/waypoint_mission.py`：WAIT_PX4 → ARM → OFFBOARD → TAKEOFF → CRUISE → LAND → WAIT_DISARM → DONE，含逐阶段超时与中止处理 |
| 5 m 方形航迹 | ✅ | 实测航点误差 0.15 / 0.26 / 0.44 / 0.45 / 0.46 m，均在 0.5 m 判定半径内；起飞高度误差 0.03 m |
| Offboard 桥接与看门狗 | ✅ | `px4_interface/offboard_bridge.py`：20 Hz 设定点流；目标 2 s 未刷新即保持当前位置；PX4 状态丢失即停止发设定点交还 failsafe |
| 坐标系转换与单元测试 | ✅ | `px4_interface/frames.py`（NED↔ENU、偏航换算）+ 8 个 pytest 用例 |

完整链路已打通：

```text
AirSim → PX4 → uXRCE-DDS → ROS 2 → PX4 Offboard

arming_state: 2
nav_state: 14
accepts_offboard_setpoints: True
pre_flight_checks_pass: True
failsafe: False
position z ≈ -3 m
```

当前 ROS 2 工作区结构：

```text
ros2_px4_ws/src/
├── px4_msgs/            （submodule）
├── px4_offboard/        早期单文件验证节点（保留为对照）
├── px4_interface/       接口层：QoS、坐标系、状态跟踪、Offboard 桥接
└── mission/             任务层：航点状态机 + 启动文件
```

## 4.4 尚未实现

```text
□ 自动降落（已尝试，未通过，见 4.5）
□ 人工接管（RC → PX4 优先通道）与 failsafe 故障注入验证
□ 传感器接入（AirSim RGB / Depth / LiDAR → ROS 2）
□ 实时避障与局部重规划
□ 自主探索 / 搜索区域覆盖
□ VLM 自然语言任务解析
□ 语义地图与语言—空间定位
□ 人员检测、跟踪、疑似被困人员确认
```

## 4.5 已知问题

```text
自动降落未通过：下发 land 后进入 AUTO_LAND，但 40 s 内未落地（任务按超时中止）。
2026-09-16 现场读到的 vehicle_land_detected：
  in_ground_effect=true, in_descend=true, close_to_ground_or_skipped_check=true,
  has_low_throttle=false, ground_contact=false, at_rest=false, landed=false
即飞机在近地面悬停而非触地，落地检测链路无法置位。
取证日志：logs/2026-09-14/10_54_06.ulg、logs/2026-09-16/05_45_12.ulg
处理计划：路线图阶段 A 首要调试项（分析 .ulg，必要时改用近地判定 + 显式 disarm 收尾）

航段速度偏慢：5 m 直线航段耗时约 15–19 s（约 0.3 m/s），
远低于 PX4 默认 MPC_XY_VEL_MAX，需确认限制来自轨迹生成还是 AirSim 锁步时间。
```

---

# 5. 当前软件/目录结构

现状（2026-09-14 已完成整合，全部内容收进单一项目根目录）：

```text
/home/sol/
│
└── px4_sar_uav/                    ← 项目根目录
    │
    ├── README.md                   本规划文档
    ├── CHANGELOG.md                变更日志（所有改动留痕）
    │
    ├── PX4-Autopilot/              PX4 1.18（main 分支不变，submodule 固定版本）
    │
    ├── px4_ros_uxrce_dds_ws/       Micro-XRCE-DDS-Agent 2.4.2
    │
    ├── ros2_px4_ws/                ROS 2 工作区
    │   ├── px4_msgs/               submodule，与固件匹配的消息定义
    │   ├── px4_offboard/           早期单文件验证节点（保留作对照）
    │   ├── px4_interface/          接口层：QoS / 坐标系 / 状态跟踪 / Offboard 桥接
    │   └── mission/                任务层：航点状态机 + launch
    │
    ├── scripts/                    环境与启动脚本
    │   ├── env.sh                  统一 source 入口（推荐每个终端先执行）
    │   ├── run_agent.sh            启动 Micro XRCE-DDS Agent
    │   ├── run_px4_sitl.sh         启动 PX4 SITL（连接 Windows 侧 AirSim）
    │   ├── run_offboard.sh         运行当前 Offboard 验证节点
    │   ├── check_stack.sh          链路冒烟检查
    │   ├── build_dds_ws.sh         重建 Micro XRCE-DDS Agent 工作区
    │   ├── build_ros_ws.sh         重建 ROS 2 工作区
    │   └── build_px4_sitl.sh       重建 PX4 SITL
    │
    ├── docs/                       迁移记录与开发路线图
    │
    └── logs/                       飞行日志（不入库）
```

迁移结论与注意事项：

- 三个目录的**源码本身不含任何绝对路径**，因此可以整体搬迁；
- 但 **PX4 与 ROS 2 的构建产物里有绝对路径**（PX4 可执行文件内编译进了构建目录路径，
  `build/px4_sitl_default/rootfs/etc` 是指向绝对路径的软链接；colcon 的 `install/` 是
  指向 `build/` 的绝对软链接），所以**搬完必须重建**，不能沿用旧产物；
- 日常使用方式改为：

```text
cd ~/px4_sar_uav
source scripts/env.sh          # 每个新终端执行一次
scripts/run_agent.sh           # 终端 1
scripts/run_px4_sitl.sh        # 终端 2
scripts/run_offboard.sh        # 终端 3
scripts/check_stack.sh         # 终端 4：链路检查
```

## 5.1 从远程仓库克隆与重建

远程仓库：`https://github.com/hassetorvalds/px4_sar_uav.git`

仓库内只包含**自研源码、脚本与文档**；三个第三方依赖以 submodule 形式固定版本，
构建产物（`build/`、`install/`、`log/`）与飞行日志（`logs/`、`*.ulg`）不入库。

在新机器上从零建立可运行环境：

```bash
git clone https://github.com/hassetorvalds/px4_sar_uav.git
cd px4_sar_uav

# 1) 拉取三个外部依赖（PX4 自带二级子模块，耗时较长；只需执行一次）
git submodule update --init --recursive

# 2) 重建三套构建（Agent 与 PX4 首次构建需要联网）
scripts/build_dds_ws.sh        # Micro XRCE-DDS Agent
scripts/build_px4_sitl.sh      # PX4 SITL
scripts/build_ros_ws.sh        # px4_msgs + px4_offboard

# 3) 启动链路（Windows 侧 AirSim 需先运行）
source scripts/env.sh
scripts/run_agent.sh           # 终端 1
scripts/run_px4_sitl.sh        # 终端 2
scripts/run_offboard.sh        # 终端 3
scripts/check_stack.sh         # 终端 4：链路检查
```

注意：

- `git submodule update --init`（不带 `--recursive`）只拉取三个外部依赖本身，
  PX4 构建所需的二级子模块仍然缺失，`make px4_sitl` 会报错；
- 三个 `build_*` 脚本都会**清空并重建**对应工作区，因为它们内部含绝对路径，
  这也是换目录后必须重建的原因（详见 `docs/migration-2026-09-14.md`）；
- 飞行日志不入库，如需留存请从
  `PX4-Autopilot/build/px4_sitl_default/rootfs/log/` 自行取出保存。

## 5.2 分支约定

分支模型（保持简单，够用即可）：

```text
main        稳定基线，始终可构建、可复现，只接受合并，不直接提交
 │
 ├── dev    集成分支，日常开发都在这里提交
 │    │
 │    └── feature/<模块>-<简述>   单个功能，从 dev 牵出，完成后合回 dev
 │
 └── hotfix/<简述>               紧急修复，从 main 牵出，修完合回 main 并同步到 dev
```

约定：

1. **`main` 不直接提交**：所有改动经 `dev`（功能分支）合入，保持 `main` 与
   远程一致、随时可作为可复现基线。
2. **分支命名**：`feature/<模块>-<简述>`，模块用第 6 节的包名（如 `feature/px4_interface-waypoints`），
   修复用 `hotfix/<简述>`，纯文档用 `docs/<简述>`。
3. **提交信息**：使用 `feat:` / `fix:` / `docs:` / `chore:` / `refactor:` / `test:` 前缀 + 中文描述，
   一次提交只做一件事。
4. **CHANGELOG 必须同步**：任何提交都要按 `CHANGELOG.md` 顶部「记录规范」补充当天条目，
   含命令、实测数值或文件路径。
5. **涉及飞控、接管、failsafe 的改动**：必须附触发条件与结果，只跑通正常流程不算验证。
6. **submodule 指针变更**：在 CHANGELOG 中记录变更前后的 commit。
7. **阶段里程碑**打标签，例如 `v0.1-multipoint`、`v0.2-takeover`。
8. **禁止入库**：`build/`、`install/`、`log/`、`logs/`、`*.ulg`（已由 `.gitignore` 兜底，
   不要用 `git add -f` 绕过）。

未来建议逐步扩展：

```text
ros2_px4_ws/src/
│
├── px4_msgs/
│
├── px4_interface/
│
├── sensor_bridge/
│
├── mapping/
│
├── planner/
│
├── mission/
│
├── vlm/
│
├── perception/
│
└── safety/
```

---

# 6. 推荐的最终 ROS 2 软件模块

模块实现状态（2026-09-16）：

```text
✅ px4_interface     QoS、坐标系转换、状态跟踪、Offboard 桥接（offboard_bridge）
✅ mission           航点状态机（phase A 主体已实现，自动降落待修）
□ sensor_bridge □ mapping □ planner □ vlm □ perception □ safety
```

## 6.1 `px4_interface`

负责所有 PX4 接口：

```text
px4_state_bridge
px4_command
px4_offboard
```

当前实现（2026-09-16）：`offboard_bridge` 节点 + `frames` / `qos` / `state` 三个库模块。
其中状态跟踪做成**共享库**（`Px4StateMonitor`）而不是独立节点，避免重复订阅；
`px4_command` 的指令入口目前是 `std_msgs/String` 文本话题，后续需要结构化接口时再升级为服务/动作。

输入：

```text
/fmu/out/*
```

输出：

```text
/fmu/in/*
```

职责：

- PX4 状态读取
- Offboard 模式
- ARM / DISARM
- TrajectorySetpoint
- VehicleCommand
- PX4 状态监控

---

## 6.2 `sensor_bridge`

负责 AirSim / 相机 / LiDAR 数据：

```text
camera_node
depth_node
lidar_node
imu_node
gps_node
```

最终统一输出 ROS 2 sensor topics。

---

## 6.3 `mapping`

负责：

```text
local_map
semantic_map
```

局部地图：

- 障碍物
- 深度
- 点云
- 可通行区域

语义地图：

- 建筑
- 道路
- 车辆
- 窗户
- 人员候选区域
- 其他灾害场景语义

---

## 6.4 `planner`

建议包含：

```text
global_planner
local_planner
trajectory_generator
explorer
```

### Global Planner

负责：

```text
起点 → 区域 → 目标区域 → 航点
```

### Local Planner

负责：

```text
当前位姿 + 局部地图
        ↓
实时安全轨迹
```

### Explorer

负责：

```text
未知区域
 ↓
Frontier
 ↓
下一探索目标
```

---

## 6.5 `mission`

负责：

```text
任务树
状态机
行为管理
航点管理
任务恢复
```

例如：

```text
IDLE
 ↓
RECEIVE_COMMAND
 ↓
INTERPRET_COMMAND
 ↓
GENERATE_MISSION
 ↓
PLAN_GLOBAL
 ↓
EXECUTE
 ↓
SEARCH
 ↓
VERIFY
 ↓
REPORT
```

异常支路：

```text
ANY STATE
 ├── MANUAL TAKEOVER
 ├── EMERGENCY
 └── FAILSAFE
```

---

## 6.6 `vlm`

建议拆成：

```text
command_parser
scene_understanding
semantic_grounding
```

### `command_parser`

自然语言：

> “搜索前方倒塌建筑区域。”

转换：

```json
{
  "mission": "search",
  "region": "collapsed_buildings",
  "priority": "high"
}
```

### `scene_understanding`

负责：

- 场景理解
- 视觉问答
- 语义解释

### `semantic_grounding`

负责：

```text
自然语言
 ↓
视觉语义
 ↓
空间目标
```

例如：

> “去红色建筑后面。”

转换成：

```text
目标类别：建筑
属性：红色
空间关系：后方
→ 3D/地图坐标区域
```

---

## 6.7 `perception`

后期人员搜救：

```text
person_detector
person_tracker
victim_verifier
```

建议：

```text
检测模型：
“有没有人？”

VLM：
“这个人是不是疑似被困人员？”
```

不要让 VLM 承担所有高频目标检测。

---

## 6.8 `safety`

建议：

```text
collision_monitor
geofence_monitor
manual_override_monitor
emergency_manager
```

这是最终系统的高优先级安全层。

---

# 7. 自然语言到无人机行动的完整链路

例如操作员输入：

> “搜索前方 200 米范围内的倒塌建筑，优先检查可能有人躲藏的窗口，发现疑似被困人员后靠近到 15 米并悬停。”

系统应转换为：

```text
自然语言
   ↓
VLM
   ↓
结构化任务
   ↓
Mission Planner
   ↓
Region / Search Behavior
   ↓
Global Planner
   ↓
Local Planner
   ↓
实时避障
   ↓
PX4
```

可能生成：

```json
{
  "mission": "search_and_inspect",
  "region": {
    "type": "radius",
    "radius_m": 200
  },
  "targets": [
    "collapsed_building",
    "possible_survivor_location"
  ],
  "priority": [
    "window",
    "entrance",
    "open_area"
  ],
  "victim_inspection_distance_m": 15,
  "behaviors": {
    "search": true,
    "approach": true,
    "hover": true,
    "capture": true
  }
}
```

注意：

> VLM 输出的是任务级语义，而不是 roll/pitch/yaw 或电机控制量。

---

# 8. 实时避障设计

推荐链路：

```text
AirSim RGB
AirSim Depth
AirSim LiDAR
       ↓
     ROS 2
       ↓
 Local Obstacle Map
       ↓
 Local Planner
       ↓
 Safe Trajectory
       ↓
 PX4 Offboard
```

VLM 不在这里。

例如：

```text
VLM：
“去建筑 B”

Global Planner：
A → B

Local Planner：
发现前方 3 m 墙体

→ 修改局部轨迹

无人机：
绕障

→ 恢复原始目标
```

因此：

```text
VLM 决定：
“去哪”

Local Planner 决定：
“当前怎么绕过去”
```

---

# 9. 自主探索设计

灾害搜救往往不是简单的：

```text
A → B
```

而是：

```text
进入未知区域
 ↓
建立地图
 ↓
寻找 frontier
 ↓
选择下一个观察点
 ↓
覆盖搜索
 ↓
发现新目标
 ↓
改变任务优先级
```

因此建议研究两类方法：

## Fast-Planner

适合：

- 在线轨迹生成
- 四旋翼高速规划
- 复杂未知环境
- 局部轨迹优化

## FUEL

适合：

- 未知环境探索
- Frontier-based exploration
- 搜索区域覆盖
- 探索任务规划

在本项目中可以采用：

```text
Mission / Explorer
       ↓
FUEL 风格探索
       ↓
Fast-Planner 风格局部轨迹
       ↓
PX4
```

---

# 10. VLM 与空间语义

推荐重点研究：

## VLMaps

核心思想：

```text
视觉特征
+
语言特征
+
3D空间
```

将自然语言目标映射到空间地图。

例如：

```text
“红色建筑”
“车辆旁边”
“窗口附近”
“建筑后方”
```

最终都需要变成：

```text
semantic target
      ↓
3D spatial target
      ↓
planner
```

这一步是自然语言驱动导航的重要桥梁。

---

# 11. VLM-UAV 概念验证

可以参考：

## VLM-Nav

其思路与本项目比较接近：

```text
AirSim
+
UAV
+
RGB
+
Depth
+
VLM
+
Navigation
```

适合研究：

- VLM 如何介入 UAV 导航
- 图像 → 语义 → 导航决策
- AirSim 中的 VLM 实验

但不建议直接将其作为最终系统架构。

原因：

```text
VLM-Nav
    ↓
适合作为研究原型

本项目
    ↓
需要额外加入
实时局部避障
人工接管
PX4安全层
任务状态机
灾害搜索
```

---

# 12. 人工接管设计

这是系统的核心安全要求之一。

目标：

```text
自主飞行
   ↓
操作手打杆
   ↓
立即接管
   ↓
手动飞行
```

必须优先依赖 PX4 原生手动控制能力，而不是让 ROS 2 自己模拟人工接管。

推荐结构：

```text
            RC
             │
             ▼
           PX4
             ▲
             │
      ROS 2 Offboard
```

而不是：

```text
RC
 ↓
ROS2
 ↓
VLM
 ↓
Planner
 ↓
PX4
```

安全优先级：

```text
Manual Control
      >
Safety Supervisor
      >
Local Planner
      >
Global Planner
      >
Mission Planner
      >
VLM
```

需要重点研究 PX4 的：

- manual control
- RC override
- Offboard loss failsafe
- failsafe configuration
- mode switching
- commander

---

# 13. 人员识别最终方案

人员识别应该放在所有飞控和导航系统稳定之后。

推荐：

```text
RGB
 ↓
Person Detector
 ↓
Person Candidates
 ↓
Tracker
 ↓
VLM Verification
 ↓
Possible Victim
 ↓
Approach / Hover / Observe
 ↓
Record Position
 ↓
Report
```

而不是：

```text
VLM直接负责所有目标检测
```

一个合理的分工是：

```text
检测器：
“这里有一个人。”

Tracker：
“这是同一个人。”

VLM：
“这个人是否可能是受困人员？
是否位于废墟/窗户/危险位置？
是否需要进一步确认？”
```

---

# 14. 开发阶段规划

## Phase 1：基础自主飞行

当前基本完成（详见第 4.3 节的能力清单）。

```text
✅ AirSim + UE4
✅ PX4
✅ ROS 2
✅ uXRCE-DDS
✅ px4_msgs
✅ Offboard
✅ ARM
✅ 3 m 悬停
✅ 航点状态机
✅ 5 m 方形航迹（实测误差 0.15–0.46 m）
```

下一步：

```text
⏳ 自动降落（流程已实现，未通过：AUTO_LAND 后近地悬停不触地，见 4.5）
□ 降落后的自动上锁与任务收尾
```

---

## Phase 2：人工接管

目标：

```text
AUTO
 ↓
RC 接管
 ↓
MANUAL
```

需要验证：

```text
□ 自主 → 手动切换
□ 手动飞行
□ 手动 → 自主恢复
□ ROS 2 节点异常时人工仍可接管
□ Offboard 丢失时 failsafe
```

这是最终系统的安全基础。

---

## Phase 3：深度感知

加入：

```text
□ AirSim RGB
□ AirSim Depth
□ LiDAR
□ ROS 2 sensor bridge
□ 时间同步
□ 坐标变换
```

目标：

```text
传感器 → ROS 2
```

数据稳定后再做规划。

---

## Phase 4：实时避障

```text
□ Local obstacle map
□ Local planner
□ 在线轨迹更新
□ 静态障碍物
□ 动态障碍物
□ 避障后重新回到原任务
```

核心要求：

> 避障过程不经过 VLM。

---

## Phase 5：多航点和自主探索

```text
□ Global Planner
□ Local Planner
□ Frontier Exploration
□ 搜索区域覆盖
□ 自动重规划
□ 任务中断恢复
```

此阶段可以重点研究：

- Fast-Planner
- FUEL

---

## Phase 6：自然语言任务规划

```text
□ 自然语言输入
□ VLM command parser
□ 结构化任务
□ Mission Planner
□ Task State Machine
```

例：

```text
“搜索前方废墟区域”
```

转换为：

```text
SEARCH_REGION
```

---

## Phase 7：语言与空间语义结合

```text
□ 语义目标
□ 语义地图
□ 自然语言空间定位
□ 语言→3D目标
```

重点研究：

- VLMaps 思路

---

## Phase 8：灾害搜救视觉

```text
□ Person Detector
□ Person Tracking
□ VLM Victim Verification
□ 目标靠近
□ 观察/悬停
□ 记录位置
□ 搜救任务报告
```

---

# 15. 推荐 GitHub / 开源项目

| 项目 | 用途 | 推荐程度 |
|---|---|---|
| `PX4/PX4-user_guide` / `px4_ros_com` | PX4 + ROS 2 + Offboard 官方参考 | ★★★★★ |
| `microsoft/AirSim` | AirSim + UE4 仿真平台 | ★★★★★ |
| `HKUST-Aerial-Robotics/Fast-Planner` | 无人机快速轨迹规划 | ★★★★★ |
| `HKUST-Aerial-Robotics/FUEL` | 未知区域自主探索 | ★★★★★ |
| `vlmaps/vlmaps` | 语言→视觉/空间语义 | ★★★★★ |
| `gcsarker/vlm_nav` | AirSim + VLM UAV 概念验证 | ★★★★☆ |
| `ethz-asl/mav_trajectory_generation` | UAV 轨迹生成 | ★★★★☆ |
| `ethz-asl/mav_control_rw` | 控制/MPC研究 | ★★★☆☆ |
| `ros-navigation/navigation2` | ROS 2 导航架构参考 | ★★★☆☆ |
| `PX4/PX4-Avoidance` | 历史避障项目 | ★★☆☆☆ |

### 特别注意：PX4-Avoidance

`PX4/PX4-Avoidance` 很知名，但目前已经不维护，不建议作为新项目的核心依赖。

可以：

```text
学习其设计
```

但不要：

```text
直接把它作为本项目的核心规划系统
```

---

# 16. 项目最终推荐技术栈

```text
Windows 11
│
├── UE4.27
├── AirSim 1.8.1
│
└── WSL2 Ubuntu 22.04
    │
    ├── ROS 2 Humble
    │
    ├── PX4 1.18
    │
    ├── Micro XRCE-DDS Agent
    │
    ├── px4_msgs
    │
    ├── Sensor Bridge
    │
    ├── Local Mapping
    │
    ├── Fast-Planner
    │
    ├── FUEL-style Explorer
    │
    ├── Mission Planner
    │
    ├── VLM
    │
    ├── VLMaps-style semantic grounding
    │
    ├── Person Detector
    │
    ├── Person Tracker
    │
    └── Safety Supervisor
```

---

# 17. 最终运行架构

```text
                    人类
                     │
             ┌───────┴────────┐
             │                │
         自然语言             RC
             │                │
             ▼                │
           VLM                │
             │                │
             ▼                │
      Mission Planner         │
             │                │
             ▼                │
      Global Planner          │
             │                │
             ▼                │
       Local Planner ◄── Depth/LiDAR
             │
             ▼
       Safety Supervisor ◄────┘
             │
             ▼
            PX4
             │
             ▼
        AirSim / UE4
```

---

# 18. 速度层级建议

推荐：

```text
VLM / LLM
    ↓
低频
≈ 1 Hz 量级或更低
```

```text
Mission Planner
    ↓
低频
≈ 1–5 Hz
```

```text
Global Planner
    ↓
≈ 1–10 Hz
```

```text
Local Planner
    ↓
≈ 10–50 Hz
```

```text
PX4 control loops
    ↓
更高频
```

实际频率应根据具体算法和硬件进一步测试，不应把这些数字当成固定标准。

核心原则：

> **越靠近飞控，越高频、越确定、越少依赖大模型。**

---

# 19. 灾害搜救最终工作流

最终希望实现：

```text
操作员：

“搜索前方废墟，
优先寻找可能被困人员。”

        ↓

VLM
        ↓

Mission Planner
        ↓

搜索区域定义
        ↓

Global Planner
        ↓

Explorer
        ↓

Local Planner
        ↓
实时避障
        ↓
PX4
        ↓
AirSim / UE4
```

发现疑似人员：

```text
Person Detector
        ↓
Tracking
        ↓
VLM Verification
        ↓
Mission Planner
        ↓
靠近到 15m
        ↓
悬停观察
        ↓
记录位置
        ↓
报告
```

如果操作员此时接管：

```text
任意状态
   ↓
RC
   ↓
PX4 Manual
   ↓
手动飞行
```

---

# 20. 项目验收指标

## 飞控层

```text
✅ PX4 正常运行
✅ AirSim 正常连接
✅ ROS 2 正常通信
✅ Offboard 稳定（20 Hz 设定点流，含看门狗）
✅ ARM 正常
⏳ 自动降落（未通过，见 4.5）
```

## 安全层

```text
□ RC 随时接管
□ Offboard loss
□ ROS 2 节点故障
□ VLM 节点故障
□ Planner 故障
□ failsafe
```

## 规划层

```text
✅ 多航点（5 m 方形航迹，误差 0.15–0.46 m）
□ 在线重规划
□ 静态避障
□ 动态避障
□ 搜索区域覆盖
```

## VLM 层

```text
□ 自然语言任务理解
□ 结构化任务生成
□ 场景理解
□ 语言→空间目标
```

## 搜救层

```text
□ 区域搜索
□ 人员检测
□ 人员跟踪
□ 疑似被困人员确认
□ 靠近
□ 悬停观察
□ 定位/报告
```

---

# 21. 开发原则

## 原则 1：VLM 不直接控制飞控

禁止把：

```text
VLM → 电机/姿态指令
```

作为系统主架构。

采用：

```text
VLM → Mission → Planner → PX4
```

---

## 原则 2：实时避障不经过 VLM

采用：

```text
Depth/LiDAR → Local Map → Local Planner → PX4
```

确保：

```text
VLM 延迟 ≠ 避障延迟
```

---

## 原则 3：人工接管优先

人工接管不能依赖 VLM 或 ROS 2 的健康状态。

---

## 原则 4：模块化

不要把全部逻辑继续写在单一 `offboard_control.py`。

当前单文件程序用于：

```text
实验验证
```

最终应拆成：

```text
Mission
Planner
Safety
Perception
VLM
PX4 Interface
```

---

## 原则 5：逐层验证

不要一次接入：

```text
VLM + 避障 + 人员检测 + 自动驾驶
```

应该：

```text
飞控
 ↓
多航点
 ↓
人工接管
 ↓
避障
 ↓
探索
 ↓
VLM
 ↓
人员识别
```

---

# 22. 当前下一步

当前已经完成：

```text
✅ PX4 + AirSim
✅ ROS 2
✅ uXRCE-DDS
✅ Offboard
✅ ARM
✅ 3 m Hover
✅ 航点状态机 + 5 m 方形航迹
```

当前进行中：

```text
⏳ 自动降落（AUTO_LAND 后近地悬停不触地，见 4.5 与 docs/roadmap.md）
```

后续步骤：

### Step 1（已完成）

实现：

```text
5 m 方形航迹
```

验证：

```text
多航点 + waypoint state machine
```

### Step 2

状态：进行中。

实现：

```text
自动降落
```

### Step 3

实现：

```text
自主飞行 ↔ 人工接管
```

### Step 4

加入：

```text
RGB-D / Depth / LiDAR
```

### Step 5

加入：

```text
Local Planner + 实时避障
```

### Step 6

加入：

```text
自主探索 / FUEL 思路
```

### Step 7

加入：

```text
VLM Natural Language Mission
```

### Step 8

加入：

```text
Semantic Mapping / VLMaps 思路
```

### Step 9

最后加入：

```text
Person Detection
+
Tracking
+
VLM Verification
```

---

# 23. 参考资料

## PX4 / ROS 2

- PX4 ROS 2 / uXRCE-DDS 文档  
  https://docs.px4.io/main/en/middleware/uxrce_dds

- PX4 ROS 2 Offboard Control  
  https://docs.px4.io/main/en/ros/ros2_offboard_control.html

- PX4 用户指南 / ROS 2  
  https://github.com/PX4/PX4-user_guide

## AirSim

- Microsoft AirSim  
  https://github.com/microsoft/AirSim

## UAV Planning

- Fast-Planner  
  https://github.com/HKUST-Aerial-Robotics/Fast-Planner

- FUEL  
  https://github.com/HKUST-Aerial-Robotics/FUEL

- MAV Trajectory Generation  
  https://github.com/ethz-asl/mav_trajectory_generation

- MAV Control RW  
  https://github.com/ethz-asl/mav_control_rw

## Language / Spatial Semantics

- VLMaps  
  https://github.com/vlmaps/vlmaps

## VLM + UAV

- VLM-Nav  
  https://github.com/gcsarker/vlm_nav

## ROS 2 Navigation

- Navigation2  
  https://github.com/ros-navigation/navigation2

## Legacy Avoidance Reference

- PX4-Avoidance  
  https://github.com/PX4/PX4-Avoidance

> 注意：PX4-Avoidance 当前不维护，仅建议作为历史设计参考。

---

# 24. 结论

本项目最终不是单纯的：

> “VLM 控制无人机”。

而应构建为：

> **一个面向灾害搜救的语言驱动、人机协同、安全自主无人机系统。**

核心架构：

```text
自然语言
   ↓
VLM
   ↓
Mission Planner
   ↓
Global Planner
   ↓
Local Planner ← Depth/LiDAR
   ↓
Safety Supervisor ← RC
   ↓
PX4
   ↓
AirSim / UE4
```

其中：

```text
VLM = 理解任务
Mission = 决定任务流程
Global Planner = 决定去哪里
Local Planner = 决定怎么绕
PX4 = 决定怎么稳定、安全地飞
RC = 随时拥有人工接管权
```

最终形成：

```text
自然语言任务
    +
视觉理解
    +
自主规划
    +
实时避障
    +
人工接管
    +
人员搜救
```

这是本项目的总体技术路线和开发路线。
