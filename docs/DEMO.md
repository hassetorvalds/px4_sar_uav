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

## 2. Step 1：只解析，不飞（随时可演示）

```bash
scripts/fly_task.py "起飞到 2 米，向北飞 10 米，悬停 5 秒，返航降落"
scripts/fly_task.py "搜索北侧 20 米、半径 10 米的区域，寻找橙色标记，高度 2 米，降落"
```

会打印：步骤表、搜索展开的航点数/预计时长、自动补的起飞/降落、任务 JSON 路径。
**默认不会起飞**。

## 3. Step 2：最小飞行（起飞 → 前进 → 悬停 → 返航）

```bash
FLY_TASK_CONFIRM=1 scripts/fly_task.py \
  "起飞到 2 米，向前飞 5 米，悬停 5 秒，返航降落" --execute
```

预期：

```text
→ 阶段 ARM → OFFBOARD → RUN
执行 step 1/4：takeoff ...  → takeoff_complete
执行 step 2/4：forward ...  → 到位
执行 step 3/4：hover   ...  → 5 s
执行 step 4/4：return_home  → land_fast → 已上锁，任务结束
```

`forward` 是**机体前向**：以起飞时的航向为准，飞 5 m。

## 4. Step 3：搜索 + 检测（可选）

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
| 规则解析覆盖面有限 | 复杂/歧义指令会被拒绝 | 拒绝是**预期行为**；后续换 VLM parser，接口不变 |
| 执行器未飞验 | 代码/单测通过，但尚未真实起飞 | 等 4560 恢复后按 Step 2 → Step 3 顺序验收 |
