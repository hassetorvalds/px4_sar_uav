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

## 通用 Qwen 搜索目标检测（任意文本 prompt）

不启动固定颜色/人员 backend，用 Qwen 按文本 prompt 检测，发布到
`/perception/best`，复用 search/scan 的目标保持/finding/report：

```bash
# 1) Qwen 隧道 + 栈（不带 color detector）
scripts/vlm_tunnel.sh start
scripts/bringup_legacy.sh

# 2) Qwen 通用检测器（prompt 可换成“红色屋顶/车辆/…”）
python3 scripts/qwen_target_pub.py --ros-args \
  -p topic:=/perception/best \
  -p target_prompt:=橙色球 \
  -p require_orange:=false -p rate_hz:=2.0

# 3) 搜索/扫描
FLY_TASK_CONFIRM=1 scripts/fly_task.py \
  "起飞到 2 米，搜索东侧 5 米、南侧 6 米、半径 3 米的区域，扫描 20 秒，降落" \
  --execute
```

验收：搜索 8/8、扫描 20 s、目标 3 次确认、2 个 finding、无 abort；
报告 `logs/tasks/report_20261025-093438-842.md`。
目标仍是橙色代理物，不是人员；`require_orange=false` 时没有颜色 bbox 校验，
只有 Qwen 置信度/时序 spread/固定 anchor（`lock_radius_m=2.0`）约束。

加固定 anchor 后复测：目标 28.49/32.34，revisit 28.96/31.04（偏离 1.39 m），
finding #2 `revisit_of=1`；搜索/扫描/无 abort；
报告 `logs/tasks/report_20261025-094118-745.md`。

---

## NL 驱动的通用搜索检测

一条命令里说“寻找什么”，`fly_nl.py` 自动启动 Qwen 通用检测器，
不启动固定 color detector：

```bash
source scripts/env.sh
python3 scripts/fly_nl.py \
  "起飞到 2 米，搜索东侧 5 米、南侧 6 米、半径 3 米的区域，寻找橙色球，扫描 20 秒，降落" \
  --execute
```

验收：搜索 8/8、扫描 20 s、目标 3 次确认、2 个 finding（#2 `revisit_of=1`）、
无 abort；报告 `logs/tasks/report_20261025-094733-823.md`。

边界：detector 自动启动/清理；若 12 s 内没有目标发布，拒绝执行；
仍需 Qwen 隧道；不是人员检测。

额外：`verify_roi=true` 会裁 ROI 二次问 Qwen；目标贴近画面边缘
（<15% 或 >85%）时跳过 crop 验证，避免裁掉目标。
检测器还要求 `stable_hits=3` 且深度变化 ≤3 m 才发布。
自动 detector 默认仍用 **点 + ROI**；bbox 可手动加 `-p use_bbox:=true`。
bbox 截断到图像边缘时退回点；非截断 bbox 面积跨帧变化 >60% 会拒绝。

bbox 离线 IoU：5 帧 0.50–0.64。真机 bbox 搜索：3 次确认、无 abort；
贴边 bbox 会被裁到图像边界，中心仍有约 1.4 m 偏移（finding #2 revisit）；
报告 `report_20261025-100401-423`。

---

## Hybrid 自然语言任务（动作 + 视觉目标）

一条中文指令同时包含动作序列和视觉目标：

```bash
source scripts/env.sh
python3 scripts/fly_nl.py \
  "起飞到 2 米，飞到前方橙色目标附近，悬停 3 秒，降落" --execute
```

真机验收：目标 (28.59,31.98)，实际到点 0.21 m；报告
`logs/tasks/report_20261025-092311-206.md`。

搜索/扫描 + 视觉目标也支持：

```bash
source scripts/env.sh
python3 scripts/fly_nl.py \
  "起飞到 2 米，搜索东侧 5 米、南侧 6 米、半径 3 米的区域，扫描 10 秒，飞到前方橙色目标附近，降落" \
  --execute
```

验收：搜索 8/8、扫描 10 s、Qwen goto 到点 0.37 m、无 abort；
报告 `logs/tasks/report_20261025-092818-149.md`。
覆盖 0/144 是因为 legacy 单相机没有下视，不是该任务失败。

边界：视觉目标在起飞前从原点拍摄并作为相对起飞原点目标；搜索/扫描在前时会
按该目标飞；只支持一个视觉目标；仍为橙色代理物；不是 VLM 直接输出飞控速度。

---

## 完整 Demo 回归（2026-10-26）

```bash
python3 scripts/fly_nl.py \
  "起飞到 2 米，搜索东侧 5 米、南侧 6 米、半径 3 米的区域，寻找橙色球，扫描 20 秒，降落" \
  --detector dino --execute
```

结果：搜索 8/8、扫描 20 s、目标 3 次确认、单 finding（updates=3）、
无 abort、landed；报告 `report_20261026-124242-037` 含目标轨迹。
覆盖 0/144 是 legacy 单相机无下视，不是搜索失败。

---

## Qwen 自然语言导航（one-shot / dynamic）

前提：Qwen 18081 隧道、PX4/ROS 栈已启动；目标仍为橙色代理物。

```bash
# 只生成目标与 TaskSpec，不飞
scripts/run_qwen_nav.sh --target-only "飞到前方橙色目标附近"

# one-shot：Qwen 取目标 → go_to → 报告
scripts/run_qwen_nav.sh "飞到前方橙色目标附近"

# dynamic：后台 2Hz 标称 provider（实测约 0.75Hz）更新目标
scripts/run_qwen_nav.sh --dynamic "飞到前方橙色目标附近"
```

边界：Qwen 只做图像 grounding；ENU/TaskSpec/PX4 offboard 由本地层负责；
dynamic 不是 4Hz 视觉伺服；目标丢失/超时会 abort，不猜初始目标。

---

## SAR Demo v0：搜索 + 代理目标检测 + 报告

> **边界**：目标物是场景里的橙色物体，不算人员检测；报告会显式写
> “代理目标（非人员检测）”。VLM JSON 路径仍未接，使用规则解析。

```bash
# 1) 栈里带检测器（前视单相机即可）
scripts/bringup_legacy.sh with_detector:=true
python3 scripts/flight_supervisor.py --ros-args -p timeout_s:=5.0 -p arm_grace_s:=6.0 &

# 2) 一条命令跑完整 demo
scripts/run_sar_demo.sh
```

默认指令：起飞 2 m → 搜索东侧 5 m/南侧 6 m、r=3 m → 原地扫描 30 s → 降落。
报告会带：

- `search_waypoint 8/8`、`search_plan`；
- `target_confirmed world=[...]`（若检测到）、`target_demo_kind=proxy`；
- `findings`/`target_findings` 与 Markdown 证据页；
- `land_fast → done`，不把 `land_fast` 说成 AUTO_LAND。

`run_sar_demo.sh` 不会自动起链路；检测器缺失时直接退出并提示，不会静默跳过。

**2026-10-24 基线（3 轮，同一指令）**：搜索均 8/8、无 abort、均落地；
代理目标检测 2/3、finding 2/3（轮3 未命中）。报告：
`report_20261024-144302-567` / `144506-886` / `144705-850`。
这证明链路成立，但不声称检测可靠；第 3 轮 miss 记入 OPEN_QUESTIONS #8。

---

> 搜索区要放在**出生点以南/东**；朝北的航点会被前方净空门拦下（这是安全行为，
> 不是 bug）。场景正前方约 2 m 处就是墙体。

矩形覆盖（可带角度）：

```bash
FLY_TASK_CONFIRM=1 scripts/fly_task.py \
  "起飞到 2 米，搜索东侧 5 米、南侧 6 米、宽 6 米、长 4 米的矩形区域，降落" \
  --execute
```

预期：`search_plan` 带 `area_m2/path_length_m/estimated_s`，`search_waypoint 6/6`。
报告还会带 `coverage`；需要 AirSim `Down` 相机（Pitch -90）才能得到 24/24，前视只有 0/24。
敏感性示例：2 m/间距 4 m（0% 重叠）实测 97.2%，见 [COVERAGE](COVERAGE.md)。

多边形（JSON）示例：

```bash
python3 - <<'PY'
import json
task = {
  "version": 1, "source_text": "L 形多边形搜索",
  "altitude_m": 2.0, "altitude_frame": "REL",
  "max_speed_mps": 0.5, "max_duration_s": 180.0, "geofence_radius_m": 40.0,
  "steps": [
    {"type": "takeoff", "params": {"altitude_m": 2.0}},
    {"type": "search", "params": {
      "shape": "polygon", "center_east_m": 5.0, "center_north_m": -6.0,
      "polygon": [[-3,-3],[3,-3],[3,-1],[-1,-1],[-1,3],[-3,3]],
      "spacing_m": 2.0, "target": {"kind": "any"}, "altitude_m": 2.0}},
    {"type": "land", "params": {}}
  ]
}
open('/tmp/polygon_l.json','w').write(json.dumps(task))
PY
FLY_TASK_CONFIRM=1 scripts/fly_task.py --task-json /tmp/polygon_l.json --execute
```

预期：`search_plan` 面积 20 m²、足印 4 m、重叠 50%，`search_waypoint 8/8`，
下视 `coverage 20/20`。

多区域 + 转场（两个 search step，仓库内示例）：

```bash
FLY_TASK_CONFIRM=1 scripts/fly_task.py \
  --task-json examples/multi_region_task.json --order-regions --execute
```

预期：`search_plan` 两次、`transit` 一段、报告「搜索区域」表两行；下视合计 144/144。

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


## 结构面 goto smoke（灰色墙面，**不是独立物体**）

用 DINO bbox 生成灰色建筑墙面目标；**该目标是大面积墙面/立面，不是独立物体**：

```text
目标：东 6.5 / 北 22.76
实际：东 6.35 / 北 22.28
误差：0.48 m
abort：0
landed：true
```

报告：`logs/tasks/report_20261026-124715-842.md`

说明：这只证明“DINO 选中的图像区域 → ENU → goto”可行，**不能**证明
换独立物体导航成功；右侧蓝色建筑中心距离 51.7 m，超过 50 m 围栏，未飞。
要验证独立物体，需要场景里另加一个非人员目标物（如蓝色方块/锥桶）。
