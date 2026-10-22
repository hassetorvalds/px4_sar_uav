# C 的 real 级验证：PX4 System-wide Replay 计划

> **状态：计划（未实施，未录制，未回放）。**
> 本文只回答 OPEN_QUESTIONS #1：C 从 `injection` 级进入 `real` 级的最低成本路径。
> 不引入 Replay、不写脚本、不改 SITL、不烧真机；C 仍只声明 `model + injection`。
>
> 参考（外部资料，仅作机制参考）：
> - [PX4 System-wide Replay v1.16](https://docs.px4.io/v1.16/en/debug/system_wide_replay)
> - [PX4 Replay.cpp Doxygen](https://px4.github.io/Firmware-Doxygen/d4/d57/_replay_8cpp_source.html)
>
> 当前 PX4 基线：`d0c6c0df7f`（见 `docs/VERSIONS.md`）；官方文档是 v1.16，命令/模块名
> 需在当前 checkout 上核对，文中无法确认处标 `UNKNOWN`。

---

## 0. 先限定“Replay 能证明什么”

- PX4 System-wide Replay 回放的是 **PX4 uORB 消息**（传感器、RC、MAVLink、位置/姿态等），
  用 `replay` 模块按原时间戳重新发布；它不是 ROS 2 bag 回放，也不会执行 WSL 侧的 Python。
- 我们的 C 判定在 **WSL 侧**（`mission/height_gate.py` + `vlm_closed_loop.py`），输入包含
  `local_map/status` 的 `cluster_z` 与 `map_z`，这些不是 PX4 uORB 的默认日志内容。
- 因此本计划把 real 级拆成两段：

```text
R1  PX4 System-wide Replay：复现 PX4 状态/时间/设定点（官方机制）
R2  C-gate 离线重放：用录制时的 ROS 话题/CSV 重算 C 决策（轻量分析，不是新飞行机制）
```

**只做 R1 不能单独构成 C 的 real 级证据**；R2 未实施前，结论保持 `injection`。

---

## 1. 录制什么

### 1.1 PX4 uLog（System-wide Replay 的输入）

目标：覆盖 C 决策所需的“飞行高度 / 姿态 / 设定点 / 障碍输入 / 状态”。

| 数据 | uORB 话题（建议） | 对 C 的作用 | 备注 |
|---|---|---|---|
| 本地位置 | `vehicle_local_position` | `px4_z`、`map_z`、地速 | 默认 logger 通常有；需确认当前版本 |
| 姿态 | `vehicle_attitude` | 航向/俯仰诊断 | 默认通常有 |
| 载具状态 | `vehicle_status` | armed / nav_state / offboard | 默认通常有 |
| 位置设定点 | `trajectory_setpoint` 或 `vehicle_trajectory_waypoint` | 复现 offboard 指令 | **UNKNOWN**：当前版本实际记录的 topic 名/是否全速率，需 `ulog_info` 核对 |
| 障碍输入 | `obstacle_distance`（若启用 PX4 OA） | 对照 PX4 原生避障 | **UNKNOWN**：我们的 C 不用 PX4 OA；该话题可能不存在 |
| 距离传感器 | `distance_sensor` | 诊断 | 仅当有传感器/驱动 |
| 命令/ACK | `vehicle_command`、`vehicle_command_ack` | 起飞/降落/模式 | 便于对齐阶段 |
| 着陆检测 | `vehicle_land_detected` | 收尾判定 | 默认通常有 |
| 传感器 | `sensor_combined` 等 | Replay 必需的前置输入 | 若回放 EKF/模块，需按 ORB publisher rules 选 |

**要求**：C 相关话题必须**全速率**记录；uLog 中每条消息只能有一个绝对 `timestamp`，
其他时间字段必须相对主时间戳（官方 Replay 的硬要求）。录制结束用 `ulog_info` 导出实际
topic 列表，并把“实际有/无”写回本计划；不要把建议列表当成已确认。

### 1.2 WSL / ROS 侧（R2 的输入）

| 数据 | 话题/文件 | 对 C 的作用 |
|---|---|---|
| 地图簇顶/净空 | `/local_map/status`（含 `cluster_z`、`left_m/right_m`） | C 的 `cluster_top`、clearance、detour 侧向选择 |
| 飞行状态 | `/fmu/out/vehicle_local_position_v1`、`/fmu/out/vehicle_status_v4` | 与 uLog 交叉校验 |
| 目标/检测 | `/perception/best`、`/perception/best_meta`（若有） | 关联 finding/触发点 |
| 任务状态 | `/mission/status` | phase/step、`epoch_id`、`task_id` |
| 执行器报告 | `report_*.jsonl` / gate 决策 CSV | 已记录的 C 决策、时间线、anchor/transform |
| 图像/点云（可选） | `/airsim_camera/points` 或录制 bag | 复算 `cluster_z` 的原始输入；体积大，按需 |

录制方式：`ros2 bag record` 上述话题 + 保留现有 report/CSV；**不需要**新代码。
如果只录 uLog，R2 没有 `cluster_z`，不能复算 C。

### 1.3 会话/时间对齐

- 记录开始时间、PX4 boot time、`epoch_id`（会话级）与 `task_id`；
- C 的 `anchor_id/transform_id` 当前是 `@stage1_placeholder`，**不能**用于跨会话追溯；
- 分析键：`(epoch_id, task_id, boot_time + relative_timestamp)`；
- 不用 wall-clock 直接对齐；以 uLog 的 boot-relative 时间为主，ROS bag 的 header stamp 做交叉校验。

---

## 2. 回放什么

### R1：PX4 System-wide Replay（官方机制）

计划命令（参考 PX4 v1.16 文档，需在当前 checkout 核对）：

```bash
# 计划示例，未执行
export replay=/absolute/path/to/record.ulg
make px4_sitl_default
# 生成 build/px4_sitl_default_replay/
# 编辑 build/px4_sitl_default_replay/rootfs/orb_publisher.rules，
# 只允许 replay 模块发布被回放的 topic
make px4_sitl_default jmavsim
# 结束后 unset replay；新日志在 rootfs/fs/microsd/log，文件名带 _replayed
```

- 回放区段：从 C 的 `DESCEND`/候选点 gate 开始，到 `LAND`/`DONE`；不需要整段 180 s；
- 回放只按原速（官方当前限制 real-time），不能加速；
- 只回放 uORB；WSL 的 C gate 不会自动运行，必须在 R2 另做。

### R2：C-gate 离线重放（轻量分析，未实施）

计划：读取 1.2 的 `local_map/status` + `vehicle_local_position` + report/CSV，
按与 `height_gate.py` 相同的 `c_map_rel` 公式重算：

```text
gap = map_z - cluster_top_z
overflyable = gap >= clearance_m
```

并按时间线比对录制的 `overflyable/gap/detour_start` 事件。

**边界**：R2 是分析重放，不是飞行；它只能证明“同一输入下判定可复现”，
不能单独证明真实传感器/真实结构下的 C 正确性。R2 需要一个小分析脚本，
本轮不实现、不引入依赖。

---

## 3. 断言什么（real 级成立标准）

录制时先定义 baseline（来自实际 report/CSV），再比较 R1/R2：

1. **决策一致性**：同一时刻的 `overflyable` 与录制一致；
   `gap` 偏差建议 `|Δ| ≤ 0.10 m`（阈值附近 `|gap-clearance| < 0.10 m` 的样本需重复/标注）；
2. **事件序列**：`DESCEND` 候选 gate → `overflyable true/false` →（true）下一候选/降落，
   或（false）SCAN/APPROACH；序列不得出现录制中不存在的 gate 翻转；
3. **偏移不变量**：若录制包含 `px4_z` 漂移，C 用 `map_z` 判定，`overflyable` 不得随
   `px4_z` 偏移 ±0.5 m 改变；若改变，判定为“不是共同漂移”→ 不通过；
4. **detour**：只在 `goto` 段允许出现 `detour_start/detour_done`；
   `search/takeoff/land` 不应出现；触发点与录制相差建议 ≤ 0.5 s（待 baseline 校准）；
5. **时间/丢包**：uLog 回放 dropouts 必须为 0，或所有 dropout 都不覆盖 gate 窗口；
   `timestamp=0` 的消息不得参与判定；
6. **会话对齐**：`epoch_id/task_id` 与录制报告一致；anchor/transform 仍标
   `stage1_placeholder`，不得据此声称真实 anchor。

**real 级通过** = R1 状态回放无异常 + R2 决策与 baseline 在上述容差内一致 +
偏移不变量成立 + gate 事件序列一致。
**只完成 R1、未做 R2** = 仍为 `injection` 级，不得升级。

---

## 4. 成本估计

| 项 | 估计 | 说明 |
|---|---|---|
| 录制一次 | 一次真实/注入飞行 + 录制时间（约飞行时长） | 需要 uLog 全速率 + ROS bag + report/CSV；不要求新代码 |
| Replay 构建 | 一次性（约 10–20 min 级，视机器） | `replay=<ulog> make px4_sitl_default`，独立 build 目录；不动主 SITL 配置 |
| 回放一次 | ≈ 录制时长（real-time） | 官方当前只能按原速；不含分析 |
| R2 分析 | **UNKNOWN**，预计 0.5–1 人日 | 需要录制样本后才能定；本轮不实现 |
| SITL 配置 | 计划为**不改**主配置 | 只使用 `replay` 环境变量、`orb_publisher.rules`、可选 `replay_params*.txt` |

回放本身不重飞、不接 AirSim 真值进闭环；成本主要是录制和一次性构建。

---

## 5. 触发条件（什么时候值得做）

只在以下情况触发：

1. 准备对外声明“C/高度门已验证”或准备开 Batch2b；
2. 场景 2（真实低/高结构）就绪，需要把 `injection` 证据升级为 `real`；
3. 出现 C 决策争议（同一输入不同结论），需要可追溯回放。

不在以下情况做：日常 demo、只改 UI/任务层、没有可录制 gate 窗口时。
当前 #1 状态保持 `OPEN`，不因本文档自动升级。

---

## 6. 退出标准

**通过（real 级）**：

- R1 PX4 Replay 完成，无覆盖 gate 窗口的 dropout；
- R2 的 `overflyable/gap` 与 baseline 在容差内一致；
- `px4_z ±0.5 m` 偏移不改变 C 判定；
- detour 仅 goto、事件序列一致；
- 记录 `epoch_id/task_id`、uLog/ROS bag/报告路径。

**不通过**：

- 决策在阈值附近翻转且无法用重放/参数解释；
- `px4_z` 偏移改变判定（说明不是共同漂移）；
- Replay dropout 覆盖 gate 窗口；
- R2 输入缺失（无 `cluster_z`）→ 不得升级，回退 `injection`。

**退出动作**：不修改 C 口径；保留 `injection`，把失败样本加入 OPEN #1 证据。

---

## 7. 风险与前置依赖

| 风险/依赖 | 说明 | 处理 |
|---|---|---|
| Replay 只回放 uORB | WSL C gate 不会运行 | 必须做 R2；否则不能升级 |
| uLog 无 `cluster_z` | local_map 是 ROS 侧数据 | 必须录 `/local_map/status` 或 bag |
| 时间戳 | 只允许单一绝对 `timestamp`；t=0 无效 | 录制后用 `ulog_info`/回放检查 |
| 实时限制 | 官方当前只能按原速回放 | 预算按飞行时长 |
| dropouts | 官方明确会报告，影响回放 | gate 窗口必须 0 dropout |
| 版本差异 | 文档 v1.16，当前固件 1.18 beta | 在当前 checkout 核对命令/topic，不照抄 |
| anchor/transform | 仍是 `stage1_placeholder` | 不得跨会话 C 追溯；阶段 2 再落真实变换 |
| epoch/task | epoch 会话级、task 任务级 | 用 `(epoch_id, task_id, boot_time+ts)` 对齐 |
| 具体话题 | 当前未录制，默认 logger 列表未核对 | 标 `UNKNOWN`；录制后用 `ulog_info` 补 |

### UNKNOWN 清单

- 当前 PX4 `d0c6c0df7f` 的默认 logger 是否全速率记录
  `trajectory_setpoint` / `vehicle_trajectory_waypoint` / `obstacle_distance`；
- 当前 AirSim/local_map 的 `cluster_z` 能否从任何 uLog 字段重建（预计不能，需 ROS bag）；
- 当前 checkout 是否已具备 `px4_sitl_default_replay` 构建（未构建、未验证）；
- 允许的 `gap/时间` 容差需用第一份 baseline 校准，本文只给初始建议。

补齐这些信息后再更新本文；在此之前，本文只是**计划**，不是证据。
