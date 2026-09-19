# 后续开发路线图

> 配套文档：`README.md`（总体架构）、`docs/migration-2026-09-14.md`（目录整合记录）
> 起点：2026-09-14，底座已验证可用（OFFBOARD / ARM / 3 m 悬停 / 20 Hz 设定点流）

## 0. 工程前置（1–2 天，先做）

这些不是“飞”的功能，但决定后面每一步的调试成本。

| 事项 | 具体动作 | 完成判定 |
|---|---|---|
| 版本固化 | 记录 PX4 `d0c6c0df7f`（main）、px4_msgs `598c7aa`、Agent `v2.4.2`、ROS 2 Humble；**不改 PX4 分支** | `docs/VERSIONS.md` 落盘，含各仓库 commit |
| 消息错位监控 | PX4 在 main、px4_msgs 锁在 release/1.18（固件 `993f542f`）形成版本错位；每次升级前后跑一次话题冒烟 | `scripts/check_stack.sh` 全绿，异常时记录到 VERSIONS |
| 参数基线 | 导出当前 SITL 参数（`param save`）并纳入版本管理，后续调避障/接管参数有对照 | `params/px4_sitl_baseline.params` |
| 日志与回放 | 约定每次飞行保存 `.ulg` 与 `rosbag2`，并把关键结论写回文档 | `logs/` 目录结构 + 命名规范 |
| 环境脚本 | 统一 `scripts/env.sh`，杜绝各终端环境不一致 | 已在本次迁移中完成 |
| 坐标系约定 | 明确 ROS 侧使用 ENU/FLU、PX4 侧 NED/FRD，写转换工具与单元测试 | `px4_interface/frames.py` + 测试用例 |

## 1. 阶段 A：多航点、状态机与自动降落（1–2 周）

**目标**：把单文件 `offboard_control.py` 的验证能力，升级为可复用的飞行控制层。

交付物：

```text
ros2_px4_ws/src/
├── px4_interface/          状态桥 + 指令 + Offboard 设定点流
│   ├── px4_state_bridge     /fmu/out/* → 内部状态（含超时判定）
│   ├── px4_command          ARM / DISARM / 模式 / LAND 的服务封装
│   └── px4_offboard         20–50 Hz 设定点流 + 看门狗
└── mission/                IDLE→ARM→TAKEOFF→WAYPOINTS→LAND 状态机
```

验收标准：

- 5 m 方形航迹，四个航点位置误差 < 0.5 m，全程 `failsafe=false`；
- 航点状态机可中断、可重入，单点失败能回到上一个安全航点；
- 自动降落：落地检测置位（`vehicle_land_detected.landed=true`）→ 自动 disarm；
- 设定点流中断 > 0.5 s 触发看门狗动作（停机或降落），有日志证据。

风险与对策：

- **本次已验证的遗留问题**：`commander land` 后出现 `High Accelerometer Bias`、
  `vertical velocity unstable`、落地检测不置位 → 阶段 A 一开始就用已有的 `.ulg`
  复盘，先解决低速/地面段估计器问题，再谈自动降落验收。
- 位置控制精度受 AirSim 传感器与 EKF 影响，验收前先确认悬停段 `eph/epv` 量级。

进度（2026-09-16）：

- ✅ 航点状态机与 5 m 方形航迹：实测误差 0.15 / 0.26 / 0.44 / 0.45 / 0.46 m（判定半径 0.5 m），
  起飞高度误差 0.03 m；
- ✅ 设定点看门狗（目标超时保持位置、PX4 状态丢失交还 failsafe）与逐阶段超时中止；
- ⏳ 自动降落未通过：`AUTO_LAND` 后 40 s 未落地。现场读数为近地悬停
  （`has_low_throttle=false`、`ground_contact=false`、`at_rest=false`，而
  `in_ground_effect=true`、`in_descend=true`），下一步分析 `.ulg` 并确定收尾策略；
- ⏳ 航段速度约 0.3 m/s（5 m 用 15–19 s），待确认是否受轨迹生成参数或 AirSim 锁步影响；
- 实现落点：`px4_interface`（`offboard_bridge` / `frames` / `qos` / `state`）与
  `mission`（`waypoint_mission` + `square_5m.launch.py`）。
  其中状态桥接实现为**共享库**（`Px4StateMonitor`）而非独立节点，避免重复订阅与多余进程。

## 2. 阶段 B：人工接管与安全层（1–2 周）

**目标**：让“随时接管”成为可验证能力，而不是架构口号。

交付物：`safety/`（collision_monitor、geofence_monitor、manual_override_monitor、emergency_manager）。

验证矩阵（每项都要有记录）：

| 场景 | 期望 |
|---|---|
| OFFBOARD 飞行中 RC 打杆 | 立即切 POSCTL/MANUAL，人工优先 |
| 恢复自主 | 重新进入 OFFBOARD 并接回航点 |
| kill 掉 offboard 节点 | 触发 Offboard 丢失 failsafe（按 `COM_OBL_RC_ACT` 配置动作） |
| kill 掉 uXRCE-DDS Agent | PX4 不受影响，保底仍可 RC 接管 |
| 地理围栏越界 | 触发 `geofence_monitor` 阻止越界 |

实现要点：

- 人工接管走 **RC → PX4** 原生通道，ROS 2 只做监测（`manual_control_setpoint`、`input_rc`、
  `vehicle_status.nav_state`），不参与接管链路；
- SITL 下的“RC”用 QGroundControl 手柄映射（MAVLink manual control）注入，或由 AirSim
  侧注入遥控输入，两种方式都先在文档中定好；
- 相关参数（`COM_OBL_RC_ACT`、`COM_RC_IN_MODE`、`GF_*`）纳入参数基线。

## 3. 阶段 C：感知接入（2–3 周）

**目标**：把 AirSim 的 RGB / Depth / LiDAR 变成 ROS 2 时间戳正确、坐标系正确的数据。

交付物：`sensor_bridge/`（camera_node、depth_node、lidar_node、imu_node、tf 发布）。

要点与风险：

- AirSim 1.8.1 官方 ROS 封装为 ROS 1，ROS 2 需自建 bridge（Python/C++ AirSim client →
  `sensor_msgs/Image`、`CameraInfo`、`PointCloud2`）；选型要在本阶段开头定案并记录；
- 时间同步：AirSim 仿真时间 → ROS `use_sim_time`；PX4 时间戳来自 boot time，
  跨域时间不得混用；
- 坐标变换：以阶段 0 的约定为准，`map→odom→base_link→camera_link` 全链路可查，
  并在 RViz2 中可视化验收。

验收：RViz2 中深度图/点云与 AirSim 场景一致，时间戳偏移 < 10 ms，TF 无断链。

## 4. 阶段 D：实时避障（3–4 周）

**目标**：VLM 不在回路的前提下，实现“障碍出现 → 自主绕开 → 回到原任务”。

交付物：`mapping/local_map`（voxel/ESDF）+ `planner/local_planner`。

建议路线（由轻到重）：

1. 2.5D 栅格 + 膨胀 + 局部航点重规划（最快拿到可用避障）；
2. ESDF/占据栅格 + 多项式局部轨迹（Fast-Planner 思路）；
3. 再考虑完整 Fast-Planner 移植（注意上游为 ROS 1，需 ROS 2 移植工作）。

验收：静态障碍 100% 绕开；动态障碍（移动障碍）不碰撞且有重规划日志；
避障结束后回到原全局航点；全过程 VLM 节点可离线。

## 5. 阶段 E：自主探索（3–4 周）

交付物：`planner/global_planner` + `planner/explorer`（frontier 选择、覆盖策略）。

验收：未知区域内自主覆盖设定区域，覆盖率与重复率可量化（如覆盖 > 80%、重复 < 25%），
任务中断后可恢复。

## 6. 阶段 F：自然语言任务规划（2–3 周）

交付物：`vlm/command_parser`（自然语言 → 结构化任务 JSON）+ `mission` 对接。

要点：

- VLM 只输出任务级语义（区域、目标类别、优先级、行为约束），**严禁输出姿态/电机量**；
- 定义 JSON schema 与校验层，非法输出直接拒绝并回退到上一任务；
- 语言模型部署方式（Windows 侧 GPU / WSL2 GPU / 远程 API）先做延迟与带宽预算再定；
- 注入测试集：至少 20 条中文指令，覆盖模糊/矛盾/超范围指令。

验收：指令解析成功率与错误拒绝率可量化；单条指令端到端（指令 → 起飞 → 搜索）可复现。

## 7. 阶段 G：语言—空间语义（3–4 周）

交付物：`mapping/semantic_map` + `vlm/semantic_grounding`（VLMaps 思路）。

验收：“红色建筑后面”“窗户附近”等空间指代能落到地图区域坐标，并在仿真中飞到该区域。

进度（2026-09-19，前置部分提前完成）：已借鉴 See, Point, Fly 的“指向点 + 粗略深度”
思路实现 `vlm` 包（`pointing.py` / `vlm_client.py` / `vlm_navigator.py`），
链路“图像 → 指向 → 机体系反投影 → 机体速度 → PX4”已用 mock provider 飞通；
真实 VLM 与真实相机图像尚未接入。参考实现为 Proprietary 许可，仅借鉴思路
（见 `docs/reference-see-point-fly.md`）。

## 8. 阶段 H：人员搜救视觉（3–5 周）

交付物：`perception/`（person_detector、person_tracker、victim_verifier）。

分工原则：检测器回答“这里有人”，跟踪器回答“是同一个人”，
VLM 回答“是否疑似被困、是否需要进一步确认”。

验收：检测率/误报率、跟踪 ID 稳定性、疑似目标 → 靠近至指定距离 → 悬停观察 →
记录坐标 → 生成报告，全流程可回放。

## 9. 优先级与排期建议

| 阶段 | 依赖 | 建议工期 | 风险等级 |
|---|---|---|---|
| 0 工程前置 | — | 1–2 天 | 低 |
| A 多航点/状态机/降落 | 0 | 1–2 周 | 中（含估计器遗留问题） |
| B 人工接管/安全 | A | 1–2 周 | 中 |
| C 感知接入 | 0 | 2–3 周 | 中高（ROS 2 bridge 选型） |
| D 实时避障 | C | 3–4 周 | 高 |
| E 自主探索 | D | 3–4 周 | 高 |
| F 自然语言任务 | A | 2–3 周 | 中 |
| G 语言—空间语义 | C、F | 3–4 周 | 高 |
| H 人员搜救 | D、G | 3–5 周 | 高 |

并行建议：C（感知）与 F（VLM 解析）可并行推进；D 必须等 C 的数据质量达标后再开始。

## 10. 开发纪律（沿用 README 第 21 节并补充）

1. 每阶段只引入一个新变量，先跑通再叠加；
2. 每次飞行保留 `.ulg` + `rosbag2`，结论写回 `docs/`；
3. 涉及飞控的功能（接管、failsafe、围栏）必须先做“故障注入”测试，再谈功能验收；
4. 任何新增节点的 QoS 必须与 PX4 话题一致（BEST_EFFORT + TRANSIENT_LOCAL），
   否则会出现“话题在但收不到数据”的假故障。
