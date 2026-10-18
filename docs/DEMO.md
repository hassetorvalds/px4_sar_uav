# Demo：用自然语言指挥飞行

> 目标：证明**"一句话 → 经过校验的任务 → 安全执行"**这条链路成立。
> 当前 demo 不依赖 VLM 服务；VLM 只是"语言解析器"的一个可选实现，后面替换即可。

## 0. 这个 demo 的边界（先说清楚）

**能证明**

- 自然语言被解析成结构化 `TaskSpec`（有 schema、有范围校验、非法指令拒绝）；
- 解析结果先打印计划，确认后才执行；
- 执行层只做低速任务（≤1.0 m/s），带围栏、前向净空门、总时限、人工接管暂停；
- 全过程写 JSONL 报告（每一步、每个确认目标、每次中止原因）。

**还不能证明**

- 真正的语义理解（当前是规则解析，不是 VLM）；
- 人员检测（场景里没有人；当前用橙色目标物验证检测/定位链路）；
- 真降落（EKF 高度漂移仍在，收尾用 `land_fast` 时间兜底）。

## 1. 前置：把链路拉起来

Windows 侧先确认 AirSim 在 **Play** 状态，并且 4560 能接受连接：

```bash
# WSL 里测；连不上就先在 UE 里 Stop → Play 重启场景
timeout 5 bash -c 'echo > /dev/tcp/172.21.192.1/4560' && echo 4560 OK
```

然后（建议分终端/后台）：

```bash
cd /home/sol/px4_sar_uav
scripts/run_agent.sh &                       # Micro XRCE-DDS Agent
PX4_SIM_HOST_ADDR=172.21.192.1 scripts/run_px4_sitl.sh &

# 等 PX4 打出 "Ready for takeoff!" 后再起 ROS 栈
bash scripts/bringup.sh with_arbiter:=false camera_rate_hz:=2.5 &
python3 scripts/flight_supervisor.py --ros-args -p timeout_s:=5.0 -p arm_grace_s:=6.0 &
```

> `with_arbiter:=false`：本 demo 的执行器直接发 `/offboard_bridge/velocity_setpoint`。
> 如果栈开着仲裁，请改用 `--via-arbiter`（执行器走 `/velocity_arbiter/vlm_cmd`）。

起飞前检查：

```bash
source scripts/env.sh
scripts/check_stack.sh                       # 全绿（频率达标）才继续
```

## 1.5 没有 AirSim 时也能自检（假 PX4）

```bash
# 简单飞行：takeoff → forward → hover → return_home → land → done
scripts/task_executor_simcheck.sh

# 搜索：8 个航点全部到位
SIMCHECK_TIMEOUT=120 scripts/task_executor_simcheck.sh \
  "搜索北侧 5 米、半径 3 米的区域，高度 2 米，降落"

# 净空门：注入 0.5 m 前向净空 → clearance_blocked → abort → land
SIMCHECK_TIMEOUT=70 SIMCHECK_BLOCK_AFTER=6 \
  SIMCHECK_EXPECT=clearance_blocked,abort,done \
  scripts/task_executor_simcheck.sh "向北飞 8 米，降落"

# 人工接管：切 POSCTL → manual_override → 暂停 → 恢复 → manual_resume → 继续
SIMCHECK_TIMEOUT=80 SIMCHECK_TAKEOVER_AFTER=14 \
  SIMCHECK_EXPECT=manual_override,manual_resume,done \
  scripts/task_executor_simcheck.sh "向北飞 8 米，降落"
```

每个场景最后都会打印事件序列与 `PASS/FAIL`（退出码 0 = PASS）。

> `simcheck` 默认跑在 **ROS_DOMAIN_ID=77**，与真实栈隔离；真实 PX4/bridge 在线时
> 默认拒绝启动（exit 2），确需并跑设 `SIMCHECK_ALLOW_LIVE=1`。

## 2. Step 1：只解析，不飞（随时可演示）

```bash
scripts/fly_task.py "起飞到 2 米，向北飞 10 米，悬停 5 秒，返航降落"
scripts/fly_task.py "搜索北侧 20 米、半径 10 米的区域，寻找橙色标记，高度 2 米，降落"
```

会打印：步骤表、搜索展开的航点数/预计时长、自动补的起飞/降落、任务 JSON 路径，
以及 **IntentFrame**（来源/置信度/缺失/每条事实的 provenance）。
**默认不会起飞**。

VLM 解析接入前，可以用 `--remote-intent` 验证"本地事实优先"的合并规则：

```bash
# 远端（模拟 VLM）说"向北 20 米"
scripts/fly_task.py "起飞到 2 米，向北飞 20 米，降落" --intent-json > /tmp/vlm.json

# 本地规则说"向北 10 米"：合并后仍应保留 10 米，并打印拒绝改写的 warning
scripts/fly_task.py "起飞到 2 米，向北飞 10 米，降落" --remote-intent /tmp/vlm.json
```

## 3. Step 2：最小飞行（起飞 → 前进 → 悬停 → 返航）

```bash
FLY_TASK_CONFIRM=1 scripts/fly_task.py \
  "起飞到 2 米，向前飞 5 米，悬停 5 秒，返航降落" --execute
```

预期（事件都带 `altitude_lcl`/`task_altitude` 帧标签）：

```text
→ 阶段 ARM → OFFBOARD → RUN
command_sent(arm) → command_ack(accepted) → command_completed(arm)
command_sent(offboard) → command_ack → command_completed(offboard)
执行 step 1/4：takeoff ...  → takeoff_complete（LCL 与 REL 分别标注）
执行 step 2/4：forward ...  → 到位
执行 step 3/4：hover   ...  → 5 s
执行 step 4/4：return_home  → land_fast → command_completed → done
```

`forward` 是**机体前向**：以起飞时的航向为准，飞 5 m。

## 4. Step 3：搜索 + 检测（可选）

**先飞"扫描→发现"闭环**（需要检测节点在线：`with_detector:=true` 或
`scripts/run_detector.sh`）：

```bash
FLY_TASK_CONFIRM=1 scripts/fly_task.py "起飞到 2 米，原地扫描 40 秒，降落" --execute
```

预期报告：`target_confirmed world=[...]` → `target_centered error≈-3.4°`
→ `scan_complete` → `land_fast → done`；执行器还会生成 `report_*.md` 证据页。
目标为场景内橙色物体（实测 depth≈32 m）。

**组合任务（搜索 + 步进扫描 + 发现）**：

```bash
FLY_TASK_CONFIRM=1 scripts/fly_task.py \
  "起飞到 2 米，搜索东侧 5 米、南侧 6 米、半径 3 米的区域，原地扫描 30 秒，降落" \
  --execute
```

预期：`search_waypoint 1/8 … 8/8` → `scan_dwell`（45° 一步，停 1.5 s）
→ `target_confirmed world=[...]` → `scan_complete → done`。

> 搜索区要放在**出生点以南/东**；朝北的航点会被前方净空门拦下（这是安全行为，
> 不是 bug）。场景正前方约 2 m 处就是墙体。

矩形覆盖（可带角度）：

```bash
FLY_TASK_CONFIRM=1 scripts/fly_task.py \
  "起飞到 2 米，搜索东侧 5 米、南侧 6 米、宽 6 米、长 4 米的矩形区域，降落" \
  --execute
```

预期：`search_plan` 带 `area_m2/path_length_m/estimated_s`，`search_waypoint 6/6`。
报告还会带 `coverage`；需要 AirSim `Down` 相机（Pitch -90）才能得到 24/24，前视只有 0/24，见 [COVERAGE](COVERAGE.md)。

再飞**小半径搜索**（东南侧，避开正前方墙体）：

```bash
FLY_TASK_CONFIRM=1 scripts/fly_task.py \
  "起飞到 2 米，搜索东侧 5 米、南侧 3 米、半径 3 米的区域，高度 2 米，降落" \
  --execute
```

预期报告事件：`search_plan(8 航点)` → `search_waypoint 1/8 … 8/8`
→ `land_fast → done`；无 `abort` / `clearance_blocked`。

再试带检测的版本：

```bash
# 另起一个终端，把检测节点加进栈（橙色 backend，默认就是橙色）
ros2 run perception victim_detector --ros-args -p detector:=color &

FLY_TASK_CONFIRM=1 scripts/fly_task.py \
  "搜索北侧 20 米、半径 10 米的区域，寻找橙色标记，高度 2 米，降落" \
  --execute --speed 0.8
```

执行器订阅 `/perception/best`（**连续 3 帧确认**的目标），收到后：

1. 在当前位置悬停 `target_hold_s`（默认 5 s）；
2. 在报告里写 `target_confirmed` + 世界坐标（ENU）；
3. 继续搜索，最后降落。

## 5. 看报告 / 中止

```bash
# 报告（JSONL，一行一个事件）
tail -f logs/tasks/report_*.jsonl

# 中止：执行器收到后立即 land_fast
ros2 topic pub --once /task_executor/abort std_msgs/Bool "{data: true}"

# 全停
scripts/stop_stack.sh
pkill -f flight_supervisor || true
```

报告里的关键事件：`task_accepted` / `step` / `takeoff_complete` / `clearance_blocked` /
`manual_override` / `target_confirmed` / `abort` / `land_fast` / `done`。

## 6. 当前已知问题（会直接影响 demo）

| 问题 | 影响 | 处理 |
|---|---|---|
| AirSim 4560 处于 `CloseWait` | PX4 连不上，整个链路起不来 | UE 里 **Stop → Play** 重启场景；WSL 侧 `4560 OK` 后再起 PX4 |
| EKF 高度漂移 | 真降落不可用；高度门可能判错 | 收尾统一 `land_fast`；高度门数据源待定（见 RESULTS 6.1） |
| 场景里没有人 | 只能验证"目标物"检测，不能宣称人员检测 | 需要给场景加人物/假人，或换有人的环境 |
| 规则解析覆盖面有限 | 复杂/歧义指令会被拒绝 | 拒绝是**预期行为**；后续换 VLM parser，接口不变；`--remote-intent` 已定义合并契约 |
| 首条 arm 依赖 0.5 s 静置 | `arm_settle_s` 已默认开启；否则首条 ACK 可能丢 | 若首条命令无 ACK，先看 `command_timeout` 与 DDS 订阅匹配 |
| 实机电池检查 | SITL 无电池读数时按 unknown/warn | 实机设 `require_battery:=true` + `min_battery_ratio` |
| 搜索步骤未飞验 | 起飞/悬停/降落、向前/返航已飞通；搜索航点尚未实飞 | 先小半径（r=3 m）验证，再放大 |
