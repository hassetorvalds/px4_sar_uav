# 参考仓库对照与"语言→飞行"设计结论

> 日期：2026-10-18
> 目的：在继续扩展"自然语言指挥飞行"之前，先对照成熟项目的**接口设计**，
> 避免重复造轮子，也避免把已经验证过的飞控栈推倒重来。
>
> 本地参考目录：`reference/`（已加入 `.gitignore`，不入库）。
> 复现：`git clone --depth 1 <url> reference/<name>`。

## 0. 先对齐一个事实

项目里的基础能力**早就验证过**：Offboard / ARM / 悬停 / 5 m 方形航迹 / 人工接管 /
Offboard 丢失策略 / 四层落地兜底，证据在 [RESULTS.md](RESULTS.md) 与 CHANGELOG。
2026-10-18 这次真飞只是**复验链路**（起飞 1.9 m → 悬停 5 s → 降落；向前 3 m → 返航降落），
不是新能力。

真正的新东西是"自然语言 → 可校验任务 → 安全执行"这一层。它应该先对齐设计，
再决定写什么。下面是对照结论。

## 1. 拉了什么、为什么

| 仓库 | 大小 | 许可证 | 对照重点 | 结论 |
|---|---|---|---|---|
| `Auterion/px4-ros2-interface-lib` | 3.5 MB | **BSD-3-Clause** | 外部模式（ModeBase）、任务执行器（MissionExecutor）、健康/解锁检查、failsafe 延迟 | **重点借鉴设计**，暂不换库 |
| `PX4/px4_ros_com` | 428 KB | **BSD-3-Clause** | 官方 Offboard 示例、QoS/时间戳/话题命名基线 | 作为回归对照，不引入 |
| `alireza787b/mavsdk_drone_show` | 92 MB | **PolyForm 多许可（非宽松）** | Simurgh 的 typed intent / 确认门 / 命令生命周期 / 证据链；QuickScout 的 SAR 覆盖规划与发现物模型 | **只借概念，不抄代码** |
| `Jaeyoung-Lim/px4-offboard` | 320 KB | **BSD-3-Clause** | 最小 Offboard 示例 + RViz visualizer | 参考 visualizer；控制层我们已更强 |
| `AhmedHisham1/UAV-Search` | 2.3 MB | **无 LICENSE（默认保留所有权利）** | 学生 SAR 项目：A* 离线规划 + MAVSDK 任务；YOLO/SSD 检测；**正下视相机**像素→全局定位 | **只借思路**，不复制 |
| `JacopoPan/aerial-autonomy-stack` | 3.1 MB（partial） | **MIT** | **ROS 2 Action 飞行原语 + YAML 行为树任务 + 黑板上条件 + 弓字搜索/锁定** | **首选架构参考**；可借鉴代码（保留版权） |
| `casy-lab/PX4_Firmware` | 336 KB（partial，未 checkout） | PX4 原 BSD-3 派生 | 雪崩 ARVA 的 extremum seeking + IMB 控制（学术分支 `airborne`） | 领域不同，**仅概念参考** |

> `PX4_Firmware` 的默认分支只是 PX4 1.12 派生文档；真正的 ARVA/ES 代码在 `airborne` 分支的
> `src/modules/extremum_seeking` 与 `mc_att_control/IMBControl`，需要单独 fetch 该分支才有内容。
> 与当前"视觉/语言指挥"方向不同，未继续深入。

未拉（下一阶段按需）：`HKUST-Aerial-Robotics/Fast-Planner`、`FUEL`（规划/探索，ROS 1 为主，
注意 GPL 类许可）、`vlmaps`、`vlm_nav`（语义/VLM 导航）。

## 2. px4-ros2-interface-lib：最值得借鉴的是"模式 + 任务"分层

关键文件：

- `px4_ros2_cpp/include/px4_ros2/components/mode.hpp`
- `px4_ros2_cpp/include/px4_ros2/mission/mission_executor.hpp`
- `px4_ros2_cpp/include/px4_ros2/mission/mission.hpp`
- `px4_ros2_cpp/src/mission/mission_executor.cpp`
- `px4_ros2_cpp/include/px4_ros2/components/health_and_arming_checks.hpp`
- `examples/cpp/modes/mission/include/mission.hpp`

它做了什么：

1. **ModeBase**：把"外部控制"注册成 PX4 的一个模式（不是裸发 Offboard 设定点）。
   - `checkArmingAndRunConditions()`：周期检查"能不能解锁/继续跑"；
   - `onActivate()` / `onDeactivate()`：控制权进入/退出；
   - `updateSetpoint(dt_s)`：设定点更新；
   - `completed(Result)`：模式结束语义（`Rejected/Timeout/Deactivated/...`）；
   - `deferFailsafes(enabled, timeout)` + `onFailsafeDeferred()`：主动接管 failsafe 时机；
   - 看门狗、RequirementFlags、消息兼容性检查。
2. **MissionExecutor**：任务是一个 JSON：
   ```json
   {
     "version": 1,
     "mission": {
       "defaults": {"horizontalVelocity": 5, "maxHeadingRate": 60},
       "items": [
         {"type": "takeoff"},
         {"type": "navigation", "navigationType": "waypoint", "x": 0, "y": 0, "z": 2},
         {"type": "basicCustomAction", "customArgument": "..."},
         {"type": "rtl"}
       ]
     }
   }
   ```
   - 内置动作：`takeoff / rtl / land / hold / changeSettings / onResume / onFailure`；
   - 自定义动作插件（`addCustomAction`）；
   - `onProgressUpdate` / `onCompleted` / `setActivityInfo`；
   - `MissionFileMonitor` + `persistence_filename`：任务文件热更新、中断后续跑；
   - 轨迹执行与任务执行分开（`TrajectoryExecutorInterface`）。

**对照我们的 `mission/task_spec.py` + `task_executor.py`**：

| 维度 | 我们现在 | 参考做法 | 判断 |
|---|---|---|---|
| 任务表示 | `TaskSpec.steps`（8 种 flat step） | `version + defaults + items`（动作可扩展） | **可以对齐**，但保持 backend 不变 |
| 控制权 | 发 `arm/offboard` 命令，自己管阶段 | 注册成 PX4 模式，PX4 知道"谁在控" | 借鉴语义，暂不移植（C++/版本成本） |
| 解锁前检查 | `preflight.sh` + 飞控自身检查 | `checkArmingAndRunConditions` + RequirementFlags | **应补**：执行器内做一次显式健康检查 |
| 命令结果 | bridge 日志里有 ACK，执行器不跟踪 | `Result` 贯穿模式完成语义 | **应补**：命令生命周期 |
| failsafe | 外部监督层 + 自己的超时 | `deferFailsafes` + `onFailsafeDeferred` | 记录差异，暂不替换 |
| 续跑 | 无 | persistence + file monitor | 后置（demo 不需要） |

**结论**：借这套"模式/任务/动作/健康检查/完成语义"的**分层**，不把栈换成 C++。
当前 Python 绑定还不完整，换库会把已验证的 Offboard 链路重新暴露在风险里。

## 3. px4_ros_com：官方基线，确认我们没走偏

- `src/examples/offboard_py/offboard_control.py`：10 拍设定点后切 Offboard + arm，
  然后按高度发位置点，到点就 `land` 并 `exit(0)`；**没有 ACK、没有看门狗、没有 failsafe 处理**。
- 我们的 `offboard_bridge` 已经有：状态新鲜度看门狗、ACK 日志、`land_fast`、
  `land_autodisarm`、速度/位置双模式、目标超时保持——比官方示例严格。

**结论**：只借它的话题命名/QoS/时间戳基线做回归，不引入代码。

## 4. mavsdk_drone_show：最接近"语言 → 受控飞行"的产品化参考

这是三份里**概念上最有价值**的，但许可证最严（PolyForm 多许可，非宽松）。
**只读设计，不复制代码。**

### 4.1 Simurgh：typed intent + 确认门 + 证据链

关键文件：`docs/guides/simurgh-operator.md`、`docs/plans/2026-07-27-simurgh-feasibility-checkpoint.md`。

它把一条语言指令拆成：

```text
一句自然语言
  → 结构化 intent frame（routing text / read-only plan / action draft /
     confirmation 信号 / confidence / reasons）
  → 人工确认门（guarded action draft，先展示 operator plan）
  → 确定性执行（registry schema 校验 / 运行时模式 / approval / circuit breaker /
     命令提交 / 监控 / 审计）
  → 证据（telemetry + ULog summary + action journal）
```

几条原则对我们**直接适用**：

1. **语义层不是权限层**：VLM/LLM 只做解释，schema 校验、确认、熔断、命令提交、审计
   必须由确定性代码拥有。
2. **本地已解析的事实，提供商不得改写**：例如本地解析出"10 m"，VLM 不能变成"100 m"。
3. **缺失/歧义遥测 = unknown**，不许猜；高度必须标帧（REL / LCL / BARO / MSL）。
4. **确认门 + 熔断 + 监控 + 审计**是四条独立控制，不合并。
5. **provider 失败不能丢掉本地 typed draft**：本地规则解析是兜底，不是附属品。
6. **不把聊天历史/原始日志当证据**：证据要落盘、可复核、带哈希。

这几乎就是我们"规则解析 + VLM 可选"应该走的结构，只是我们目前只有
`TaskSpec` + CLI 的 `FLY_TASK_CONFIRM=1`，缺少：
intent frame、provenance、显式确认计划、命令生命周期、证据摘要、熔断状态机。

### 4.2 QuickScout：SAR 任务模板、覆盖规划、发现物模型

关键文件：`quickscout_mission.py`、`gcs-server/sar/coverage_planner.py`、
`gcs-server/sar/schemas.py`、`gcs-server/sar/command_lifecycle.py`。

可以直接借的领域模型：

- **任务模板**：`point_dispatch / area_sweep / last_known_point / corridor_search`；
- **规划位置模式**：`live_drone_positions / configured_origin`（我们现在的"固定起始位姿"
  就是 configured_origin 的雏形）；
- **任务阶段**：planning / ready_to_launch / launch_queued / searching / mixed_control /
  holding / return_commanded / completed / aborted；
- **命令生命周期**：queued / preparing / awaiting_ack / accepted / delivery_unknown /
  rejected / executing / completed / failed / tracking_unavailable；
- **发现物（findings）**：type（person/vessel/vehicle/clue/hazard/…）/ priority /
  status（new/under_review/confirmed/dismissed/handed_off）/ confidence；
- **覆盖规划**：多边形 + 沿最长边定扫掠角 + `sweep_width × (1 − overlap%)` 间距 +
  多机分区 + 就近分配 + 转场航点（survey leg 标记）。

对照我们的 `mission/search_pattern.py`：目前只是**外接正方形的固定弓字形**，
没有多边形、扫掠角、重叠率、转场腿、发现物模型。这些是下一阶段升级点，不是现在必须做。

### 4.5 你指定的另外三个仓库（2026-10-18 第二轮）

#### `Jaeyoung-Lim/px4-offboard`（BSD-3，320 KB）

- `px4_offboard/offboard_control.py`：20 Hz 位置设定点 + 状态回调；**不发 arm/offboard
  命令**，假定由 QGC 切模式并解锁；没有 ACK/看门狗。
- `px4_offboard/visualizer.py`：RViz 显示 vehicle pose / velocity / path / setpoint path，
  带 `path_clearing_timeout`。**这是我们 demo 可以补的可视化参考**。
- 结论：控制层我们已更强；**只借 visualizer 的"计划轨迹 vs 实际轨迹"可视化**。

#### `AhmedHisham1/UAV-Search`（无 LICENSE，2.3 MB）

学生毕业设计，VTOL + MAVSDK + Jetson Nano 的 SAR：

- **A\* 2D 离线规划**：在地图上点选禁飞区，规划 home→goal，路径作为 Mission Waypoints
  上传（`planning_utils.py` / `path_planning.py` / `mavsdk_utils.py`）。
- **检测**：YOLOv3 / SSD-MobileNet / TensorRT，面向 UAV Challenge 的"人"。
- **正下视相机定位**：像素中心 + 无人机位置/航向 + 高度，用三角相似换算全局坐标；
  假设相机始终朝下（`cv_utils.py`）。

对照我们：我们用**深度图 + 相机内参 + 倾斜相机**做像素→ENU（更通用）；
他们的"正下视 + 高度"是更简单但受限的特例。结论：**只借思路**（离线规划→任务上传、
检测→定位的数据流），代码不可复制（无许可）。

#### `JacopoPan/aerial-autonomy-stack`（MIT，3.1 MB partial）——**首选架构参考**

这是三份新仓库里最值得深入看的：

- `aircraft/aircraft_ws/src/autopilot_interface_msgs/action/`：
  **ROS 2 Action 飞行原语**（`Takeoff / Land / Orbit / Offboard`），goal/result/feedback；
- `aircraft/aircraft_ws/src/mission/mission/behaviors.py`：
  `BaseActionBehavior` 非阻塞 action client：等 server → 发 goal → 处理
  `accepted/rejected`（最多重试 20 tick）→ 等 result → **中断时 cancel_goal_async**；
- `tree_builder.py` + `aircraft_resources/missions/*.yaml`：
  **YAML → py_trees 行为树**（Sequence/Selector/Repeat），动作参数化；
- `mission_node.py`：黑板上放 `yolo_detections / ground_tracks / lat/lon/alt`，
  用 `check_blackboard` 条件节点等"检测到目标"再动作；
- `offboard_control/src/px4_offboard.cpp`：
  `traj_ref_lawnmower_search()` —— 东西向扫、向北换行、leg separation、速度上限；
  检测 fix 要 **6 次一致** 才动作、10 s 过期、保持 3 m 地面距离/2.5 m 高度"盯着看"；
- 同时支持 PX4 Offboard、`px4-ros2-interface-lib` 外部模式和 ArduPilot/Betaflight。

**对我们的直接启发**：

| AAS 做法 | 我们的现状 | 建议 |
|---|---|---|
| ROS 2 Action 原语 + cancel | `String` 命令 + 阶段超时 | **P1 改成 action 接口**（takeoff/land/goto/hover/search），abort 走 cancel |
| YAML 行为树任务 | flat `TaskSpec.steps` | P2 可选；先用"step 列表 + Selector"即可 |
| 黑板上等检测 | `~/best` 回调 + 悬停 | 可借"条件节点"概念，但不必上 py_trees |
| 弓字搜索 + fix 确认/stand-off | `lawnmower` + tracker 3 帧 | 借参数：确认帧数、过期、保持距离/高度 |
| Action feedback | JSONL 报告 | 加 feedback 进度（当前 step/剩余航点） |

#### `casy-lab/PX4_Firmware`（PX4 派生，学术分支）

雪崩救援 ARVA 的 **extremum seeking**（寻找信号极值）+ IMB 姿态控制。它的"搜索"是
**信号源搜索**，不是视觉/语言任务；与当前 demo 方向不同。结论：**仅作领域概念参考**，
不纳入代码参考；若以后做"信号源/信标搜索"再回来看 `airborne` 分支。

## 5. 差距清单（按优先级）

| 维度 | 我们现在 | 参考做法 | 建议 | 优先级 |
|---|---|---|---|---|
| 意图/确认 | CLI `FLY_TASK_CONFIRM=1` | typed intent frame + operator plan + 确认门 | 增加 IntentFrame（含 provenance/confidence/missing）与"计划确认"输出 | **P0** |
| 命令生命周期 | bridge 日志有 ACK，执行器不跟踪 | queued→awaiting_ack→accepted→executing→… | 执行器跟踪每条命令的 ACK/超时/结果，写进报告 | **P0** |
| 飞行原语接口 | TaskSpec step + **ExecuteTask Action 适配层** | **ROS 2 Action**（takeoff/land/orbit/offboard，cancel/feedback） | ✅ P1：goal=TaskSpec，feedback/result/cancel 已通；独立 Takeoff/Goto Action 未拆 | 已落地 |
| 任务编排 | flat steps | YAML → py_trees（Sequence/Selector/Repeat） | 先保留 step 列表；需要条件/回退时再引入 BT 或 Selector | P2 |
| 搜索锁定 | 确认 3 帧 + 悬停 5 s | 6 次一致、10 s 过期、3 m/2.5 m stand-off | 借 AAS 的 fix 确认/过期/stand-off 参数 | P2 |
| 高度帧 | `px4_z`/ENU 裸值混用 | REL/LCL/BARO/MSL 标注，unknown 不猜 | TaskSpec/报告/高度门全部带 frame 标签 | **P0** |
| 解锁前健康检查 | `mission/health.py` + 执行器 | `checkArmingAndRunConditions` | ✅ 已做：状态/位置/preflight/failsafe/battery，失败不发 arm | 已落地 |
| 任务 schema | flat steps | `version + defaults + items` | 兼容式对齐（保留 steps 加载器，新增 defaults/items） | P1 |
| 证据/审计 | JSONL + `report_*.md` | durable journal + ULog summary + hash | ✅ Markdown 摘要已做；ULog 路径/哈希仍缺 | 部分落地 |
| 覆盖规划 | 矩形/旋转/多边形弓字 + 多区域/转场 | 多边形/扫掠角/重叠/转场腿 | ✅ 矩形/多边形/敏感性/多区域已做；转场仍为直线 | 部分落地 |
| 发现物/报告 | `mission/findings.py` + `report_*.md` | findings schema | ✅ 已做：type/priority/status/confidence/world/首次最后；写入 report/Action result/Markdown | 已落地 |
| 续跑/持久化 | 无 | persistence + file monitor | 后置 | P3 |
| 模式注册 | 裸 Offboard | PX4 外部模式 | 暂不换（成本/风险） | 不做 |

## 6. 建议的目标结构（先设计，不编码）

```text
语言层（规则解析 / VLM 可选，可替换）
  └─ IntentFrame：typed facts + provenance(local|vlm) + confidence + missing details
       ↓
确认门：展示 operator plan；人确认；dry-run 可看完整步骤
       ↓
Mission：version + defaults(速度/高度/围栏) + items(takeoff/navigation/action/rtl/land/hold)
       ↓
Executor：健康检查 → 命令生命周期(ACK/timeout) → 阶段机 → 熔断/接管暂停
       ↓
PX4：现有 Offboard 链路（暂不换成 external mode）
       ↓
Evidence：JSONL journal + ULog 路径 + 证据摘要 + findings
```

必须固化的不变量：

1. **VLM 不得覆盖本地已解析事实**（距离/高度/方向/目标）；
2. **缺失/歧义 = unknown**，不猜；高度必须标帧；
3. **发送 ≠ 完成**：每条命令有 ACK/超时/失败状态；
4. **arm 前必须过健康检查**，人工确认不可省；
5. **熔断/接管始终可用**，自主输出可被立即停止；
6. **证据可复核**：报告、ULog、参数、提交哈希一一对应。

## 7. 建议的推进顺序

**第 1 步（1–2 个会话，P0）**
- 给现有 `fly_task.py`/`task_executor` 加 **IntentFrame + 确认计划**（不换执行器）；
- 高度/位置全部带 **frame 标签**，报告里区分 PX4 REL/LCL 与 AirSim 真值；
- 执行器跟踪 **命令 ACK 生命周期**，报告里写 `awaiting_ack/accepted/timeout`。

**第 2 步（P1）**
- TaskSpec 兼容式对齐 `defaults + items`（保留旧 steps 加载）；
- arm 前健康检查；
- **飞行原语改 ROS 2 Action**（takeoff/land/goto/hover/search + cancel/feedback），
  参考 AAS `autopilot_interface_msgs/action` 与 `BaseActionBehavior`；
- 报告生成 Markdown 证据摘要（含 ULog 路径/哈希）。

**第 3 步（P2）**
- 覆盖规划升级（多边形/扫掠角/重叠/转场腿）；
- findings 模型（person/priority/status/confidence）；
- 搜索锁定参数对齐 AAS（确认帧数/过期/stand-off）；
- 需要条件/回退时再引入 py_trees / YAML 任务；
- 再谈 VLM 解析器替换（仍受"不得覆盖本地事实"约束）。

**暂不做**
- 换成 `px4_ros2_interface_lib` 的 C++ 模式（Python 绑定不完整、版本兼容成本高）；
- 换成 MAVSDK / PX4 Mission Mode（与现有 Offboard 栈是不同路线）；
- 大改行为树/多机/真实飞机（demo 阶段不需要）。

## 7.5 P0 落地（2026-10-18，假 PX4 + 真机复飞）

按上面的 P0 做了三件最小改动，不换栈：

| 项 | 文件 | 行为 |
|---|---|---|
| IntentFrame | `mission/intent.py` | 规则解析产出 typed facts + `source=local` + 置信度/缺失；`merge_intent` 本地事实优先，VLM 改写会被拒绝并记 warning；CLI `--remote-intent` 可验证 |
| 高度帧 | `mission/altitude.py`、`TaskSpec.altitude_frame` | `REL/LCL/BARO/MSL/TRUTH`；报告写 `altitude_lcl` 与 `task_altitude`；`TRUTH` 仅诊断、禁止作为任务帧；起飞 target 用 `ground_lcl + altitude_rel`，比较始终在 LCL 内 |
| 命令 ACK 生命周期 | `mission/command_tracker.py`、`task_executor.py` | `command_sent → command_ack → command_completed`；超时 `command_timeout → delivery_unknown` 并触发收工；`land_fast` 无 ACK，按"是否上锁"判完成 |

验证：

```text
· mission 单测 69 passed（新增 intent/altitude/command_tracker 共 21 项）
· 全包 196 passed / 2 skipped
· 假 PX4 自检（SIMCHECK_EXPECT=command_ack,command_completed,done）PASS：
    task_accepted → phase → command_sent(arm) → command_ack(accepted, 0.001 s)
    → command_completed → phase → command_sent(offboard) → command_ack → command_completed
    → takeoff → ... → land_fast → command_completed → commands → done
· CLI：本地"向北 10 m" + 远端"向北 20 m" → 保留 10 m，warning 写明拒绝
```

真机复飞：最小任务 3 次（前两次 arm ACK 超时→重试成功；加 `arm_settle_s=0.5` 后
arm ACK 0.011 s、offboard 0.016 s，takeoff LCL 2.008 / REL 2.0，安全上锁落地）。
小半径搜索也已真机飞通：东南侧 r=3 m、8/8 航点（7–75 s）、无 abort、仅地面碰撞。
未做：VLM 实际接入（只留了 `--remote-intent` 接口）。

## 7.6 P1 落地（2026-10-18）：健康检查 + 自检域隔离 + 帧标签

- `mission/health.py`：arm 前五项检查，失败不发 arm；真机验证"状态/位置未就绪先拒绝"。
- `task_executor_simcheck.sh`：默认 `ROS_DOMAIN_ID=77` + 真实栈互锁；真实栈在线时
  默认拒绝，独立域下允许并跑也不会污染真机。
- `vlm_closed_loop.py`：高度门/CSV 补 `px4_frame=LCL`、`truth_frame=TRUTH`、
  `gate_frame`；`height_source=airsim` 打 "TRUTH 仅诊断" 警告。

## 7.7 P2 落地（2026-10-18）：ExecuteTask Action

参考 AAS 的 `autopilot_interface_msgs/action` 与 `BaseActionBehavior`，但**没有重写控制**：

- 新增 `mission_interfaces/action/ExecuteTask.action`；goal 就是 TaskSpec JSON；
- `task_action_server` 把 goal 发到 `/mission/task`，从 `/mission/status` 取
  phase/step/altitude/report；feedback/result/cancel 全部复用现有执行器；
- 取消走 `/mission/cancel`，执行器落地上锁；
- 假 PX4 正常/取消两条路径 + 真机最小 Action 飞行均通过。

未做：独立 `Takeoff/Goto/Search` Action 类型、并发/抢占、YAML/py_trees。
下一步若要 BT，直接把 TaskSpec 作为 goal 即可，不需要再改执行器。

## 7.8 P3 落地（2026-10-18）：findings 模型

- `FindingsBook`：3 m 内合并、位置增量平均、状态只升级不降级；
- 执行器目标确认 → `finding(confirmed)`，对准 → `finding(inspected)`；
- `done.findings`、`/mission/status`、`ExecuteTask.Result.findings_json`、
  Markdown「发现物」表四条出口都有；
- 真机扫描 45 s：`#1 target [36.19,35.88,5.73]` → inspected；
- 持久化：`logs/findings/*.json` + `index.jsonl`，`task_findings` 跨任务汇总。

## 8. 参考仓库的许可证提醒

| 仓库 | 许可 | 能否复制代码 |
|---|---|---|
| `px4-ros2-interface-lib` | BSD-3-Clause | 可以（保留版权/许可声明） |
| `px4_ros_com` | BSD-3-Clause | 可以（保留版权/许可声明） |
| `Jaeyoung-Lim/px4-offboard` | BSD-3-Clause | 可以（保留版权/许可声明） |
| `JacopoPan/aerial-autonomy-stack` | **MIT** | 可以（保留版权/许可声明） |
| `mavsdk_drone_show` | PolyForm 多许可 | **不要直接复制**；只借概念/接口形状，自行实现 |
| `AhmedHisham1/UAV-Search` | **无 LICENSE** | **不要复制**（默认保留所有权利）；只借思路 |
| `casy-lab/PX4_Firmware` | PX4 BSD-3 派生 | 理论可复制，但领域不同；仅概念参考 |

> 本文件只记录设计对照，不包含任何从参考仓库复制的代码。
