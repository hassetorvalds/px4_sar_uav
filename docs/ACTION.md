# ExecuteTask：ROS 2 Action 接口

## 为什么是适配层

没有重写控制逻辑。Action 的 goal 就是一份 **TaskSpec JSON**（与 `logs/tasks/task_*.json`
同 schema），适配层把它发布到 `/mission/task`，再从 `/mission/status` 读阶段、步数、
高度和结果。这样 BT/VLM 有了标准的 feedback/result/cancel 接口，而执行器仍只有一套。

```text
BT / VLM / 脚本
      │  ExecuteTask action
      ▼
task_action_server ── /mission/task ──▶ task_executor ──▶ offboard_bridge ──▶ PX4
      ▲                                   │
      └──────── /mission/status ◀─────────┘
```

## 接口

`mission_interfaces/action/ExecuteTask.action`

| 字段 | 说明 |
|---|---|
| goal `task_json` | TaskSpec JSON（`fly_task.py --json` 或 VLM 生成） |
| feedback `phase/step_index/step_total/step_type/progress/altitude_lcl_m/target_altitude_m/status` | 阶段、步数进度、LCL 高度 |
| result `success/message/report_path/findings_json` | 是否成功、失败原因、JSONL 报告、结构化发现物 |
| cancel | 接受；执行器收到 `/mission/cancel` 后 **落地上锁**（不恢复任务）；`--cancel-twice` 验证幂等 |
| external_land | 执行器已 armed 后检测到 disarmed（supervisor/failsafe）→ `external_land`，不再发落地 |

- 一次只接受一个 goal；执行器不在 `IDLE/DONE` 或状态过期时拒绝。
- goal 在接受前会做 `TaskSpec.validate()`，非法任务直接拒绝。
- 反馈由 `/mission/status` 驱动，约 2 Hz；结果在执行器进入 `DONE` 时返回。

## 启动

```bash
# 1) 执行器空跑等待 /mission/task（不要给 task_file）
ros2 run mission task_executor --ros-args \
  -p dry_run:=false -p report_path:=logs/tasks/report_action_demo.jsonl

# 2) Action 服务器
scripts/run_action_server.sh

# 3) 生成 TaskSpec JSON 并提交
python3 scripts/fly_task.py "起飞到 2 米，悬停 3 秒，降落" --json > /tmp/task.json
ros2 run mission task_action_client --task-file /tmp/task.json --timeout 60

# 完整搜索+检测（真机已飞通）
python3 scripts/fly_task.py \
  "起飞到 2 米，搜索东侧 5 米、南侧 6 米、半径 3 米的区域，原地扫描 60 秒，降落" \
  --json > /tmp/task_search.json
ros2 run mission task_action_client --task-file /tmp/task_search.json --timeout 240

# 可选：4 秒后取消（--cancel-twice 再发一次测幂等）
ros2 run mission task_action_client --task-file /tmp/task.json --cancel-after 4 --cancel-twice
```

## SemanticIntent → TaskSpec

Action 的 goal 仍是 TaskSpec；上层可以先把手写/模型语义意图编译成 TaskSpec：

```bash
python3 scripts/fly_task.py --semantic-intent examples/semantic_intent_area_cover.json \
  --json > /tmp/task.json
ros2 run mission task_action_client --task-file /tmp/task.json --timeout 200
```

详见 [SEMANTIC_INTENT.md](SEMANTIC_INTENT.md)。

## findings 落盘与汇总

任务结束执行器会写 `logs/findings/findings_<task_id>.json` 并追加 `index.jsonl`；
Action result 的 `findings_json` 与这份记录一致。

```bash
ros2 run mission task_findings --limit 20
ros2 run mission task_findings --json
ros2 run mission task_findings --md logs/findings/summary.md
```

## 自检

```bash
bash scripts/action_simcheck.sh                       # 正常路径
ACTION_SIMCHECK_CANCEL_AFTER=4 bash scripts/action_simcheck.sh   # 取消路径
```

两者都跑在 `ROS_DOMAIN_ID=77` 的假 PX4 上，默认拒绝在真实 PX4 在跑时启动
（`ACTION_SIMCHECK_ALLOW_LIVE=1` 才允许并跑）。

## 已验证

- 假 PX4 正常：goal 接受 → feedback `ARM/RUN/LAND/DONE` → `success=True` → 报告；
- 假 PX4 取消：4 s 取消 → `success=False, message=cancel_requested`，报告有
  `abort + done`，安全落地；
- 真机最小任务：goal 接受 → `success=True`，arm ACK 0.007 s、offboard 0.012 s，
  takeoff LCL 2.011 / REL 2.0，上锁落地；
- 真机完整任务：搜索 8/8 → 扫描 60 s → `target_confirmed [29.19,30.78,4.87]`
  → `target_centered 3.5°` → `success=True`，上锁落地
  （`report_action_full_20261018-170422.jsonl`）；
- 真机 findings：`#1 target [36.19,35.88,5.73]` → `inspected`，
  `findings_json` 随 result 返回，Markdown 有「发现物」表
  （`report_action_findings_20261018-171030.md`）；
- 持久化：`logs/findings/findings_action_fstore_20261018-171431.json`，
  `task_findings` 汇总 1 任务/1 发现物。

## 限制

- 仍是"单一 goal + 单任务"，没有并发/抢占；
- 飞行原语仍是 TaskSpec 的 step，不是独立的 Takeoff/Goto/Search Action 类型；
- 取消 = 落地上锁，不支持"取消后继续上一个任务"；
- 尚未接 YAML 行为树；BT 可直接把 TaskSpec 作为 action goal 发过来。
