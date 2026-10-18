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

## 5. 差距清单（按优先级）

| 维度 | 我们现在 | 参考做法 | 建议 | 优先级 |
|---|---|---|---|---|
| 意图/确认 | CLI `FLY_TASK_CONFIRM=1` | typed intent frame + operator plan + 确认门 | 增加 IntentFrame（含 provenance/confidence/missing）与"计划确认"输出 | **P0** |
| 命令生命周期 | bridge 日志有 ACK，执行器不跟踪 | queued→awaiting_ack→accepted→executing→… | 执行器跟踪每条命令的 ACK/超时/结果，写进报告 | **P0** |
| 高度帧 | `px4_z`/ENU 裸值混用 | REL/LCL/BARO/MSL 标注，unknown 不猜 | TaskSpec/报告/高度门全部带 frame 标签 | **P0** |
| 解锁前健康检查 | `preflight.sh` + 飞控检查 | `checkArmingAndRunConditions` | 执行器 arm 前做一次显式检查（位置新鲜/EKF/围栏/电池） | P1 |
| 任务 schema | flat steps | `version + defaults + items` | 兼容式对齐（保留 steps 加载器，新增 defaults/items） | P1 |
| 证据/审计 | JSONL report | durable journal + ULog summary + hash | 报告末尾生成 Markdown 摘要 + ULog 路径 | P1 |
| 覆盖规划 | 正方形弓字 | 多边形/扫掠角/重叠/转场腿 | 升级 `search_pattern` | P2 |
| 发现物/报告 | 无 | findings schema | 加 `perception` 的 finding 模型（type/priority/status/confidence） | P2 |
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
- 报告生成 Markdown 证据摘要（含 ULog 路径/哈希）。

**第 3 步（P2）**
- 覆盖规划升级（多边形/扫掠角/重叠/转场腿）；
- findings 模型（person/priority/status/confidence）；
- 再谈 VLM 解析器替换（仍受"不得覆盖本地事实"约束）。

**暂不做**
- 换成 `px4_ros2_interface_lib` 的 C++ 模式（Python 绑定不完整、版本兼容成本高）；
- 换成 MAVSDK / PX4 Mission Mode（与现有 Offboard 栈是不同路线）；
- 大改行为树/多机/真实飞机（demo 阶段不需要）。

## 8. 参考仓库的许可证提醒

| 仓库 | 许可 | 能否复制代码 |
|---|---|---|
| `px4-ros2-interface-lib` | BSD-3-Clause | 可以（保留版权/许可声明） |
| `px4_ros_com` | BSD-3-Clause | 可以（保留版权/许可声明） |
| `mavsdk_drone_show` | PolyForm 多许可 | **不要直接复制**；只借概念/接口形状，自行实现 |

> 本文件只记录设计对照，不包含任何从参考仓库复制的代码。
