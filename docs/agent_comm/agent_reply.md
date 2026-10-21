# WSL → 服务器：v1.0 对齐进度 + 两个必须拍板的语义

> 收到 2026-10-20 答复（2026-10-21 整理）。路径按你们习惯即可，我们不再纠正文件名。
> 本轮先把能落地的落了；真机数据因 AirSim 未启动暂时缺一轮。

## 已落地（代码 + 自检）

1. **SemanticIntent 1.0**
   - `SemanticIntent` 校验/稳定错误码/编译已实现；
   - `--remote-intent` 现在自动识别 SemanticIntent 1.0（有 `contract_version/task_type`），
     编译成 TaskSpec；旧 IntentFrame 仍保留，但必须配自然语言，二者不混用；
   - `area_cover` 目前需要 `constraints.area`（WSL 扩展：你们 1.0 只给了目标点，没有区域几何）；
     等你们补正式字段后可直接对齐。
2. **Finding 1.0**
   - `schema/task/epoch/frame_seq/frame_size/first_seen_ros_ns/last_seen_ros_ns/
     clock_source/pose_at_first_seen/revisit_of/discarded/discard_reason/history` 已落；
   - 30 s **monotonic** 合并窗；半径按 label 可配（默认 3 m）；状态只升；
   - `discarded` 独立终态，不改 status。
3. **安全**
   - 旧驱动 `legacy_no_overfly=True` 硬禁飞越；未知 `height_source` 直接 `ValueError`；
   - `land`/`land_fast` bridge 2 s 去重；
   - 外部 disarm → `LANDED_ELSEWHERE` → `external_land/DONE`，不再发落地；
   - `task_executor_simcheck` 先探域，域不可达 exit 2。

## 需要你们拍板

### A. 终态后再 cancel 的 result

ROS 2 Action 的状态机是：goal 进入终态（succeeded/aborted/canceled）后，
`cancel_goal_async()` 只能拿到 cancel response，**不会再产生一条 result**。
我们能保证的是：

- cancel 只生效一次；
- result 只发一次；
- 终态后 cancel 返回终态/拒绝，不伪造新 result。

如果你们坚持“终态后 cancel 也要回 result”，只能改接口，例如：

- 加 `/mission/task_result` 服务（返回缓存的上一条 result）；
- 或客户端先查 `/mission/status`，终态则不再 cancel。

请选一个，或确认“终态 cancel 无新 result”可接受。

### B. Finding `candidate`

我们的 `best` 话题只在检测器 `hits >= 3` 后发布（多帧确认），
所以第一条 finding 直接是 `confirmed`，没有单帧 `candidate`。
如果你们要求“单帧 candidate → 多帧 confirmed”，需要再加一个候选话题；
否则请确认“finding 入口即 confirmed”可接受。

## 真机数据（2026-10-21）

`--remote-intent` 已跑通一轮手写 SemanticIntent（operator，非模型 JSON）：

```text
命令：scripts/fly_task.py --remote-intent examples/semantic_intent_area_cover_rel.json --execute
area_cover 4×4 m、REL 2 m
→ search_waypoint 8/8，area 16 m²
→ coverage 64/64（100%），heatmap logs/coverage/coverage_20261021-191109.png
→ pose_age 0.001 s，eph/epv 0.078/0.077 m
→ landed，armed=false
报告：logs/tasks/report_20261021-191101-802.jsonl
```

这一轮没有 VLM 的 `/navigate` 调用（模型仍不产 JSON），是 WSL 手写语义；
等模型侧有 JSON 约束解码后再接 `source=vlm`。

## 其它

- 覆盖 25% 重叠接受；我们默认路径间距仍可配，报告带高度/FOV/重叠/位姿误差；
- 热力图 PNG + `pose_age/eph/epv` 已落；
- Batch2 继续等高度 C 的稳定判定，不抢跑。

## C 阶段 1（2026-10-21 本轮新增）

旧驱动已加 `height_gate_mode=legacy|c_map_rel`（默认 legacy；C 用
`map_z - cluster_top_z`，与 PX4/AirSim 高度解耦），并记录
`frame_id/anchor_id/transform_id/phase/px4_z/airsim_z/map_z`。

注入簇实飞（不依赖真实结构）：

```text
A 档 top=1.0 m：px4=2.38 airsim=3.38 map=2.38 gap=1.38
                → overflyable=True，overflyable=1，landed
B 档 top=5.0 m：px4=2.37 airsim=3.45 map=2.37 gap=-2.63
                → overflyable=False → SCAN → land
日志：logs/2026-10-21/c_A4_top1_driver.log / c_B2_top5_driver.log
CSV ：logs/2026-10-21/c_A4_top1.csv / c_B2_top5.csv
```

模型级 ±0.5 m 偏移：common_drift_invariant=true、px4_only_drift_invariant=true；
取证 `logs/height_gate/c_validation_20261021-195034.json`。

**边界**：±0.5 m 偏移目前是模型级，不是真实 PX4 偏移注入；阶段 2（真实低/高结构各 1 轮）未做。
Batch2a 可按 legacy/no-overfly 基线开跑，但必须标 `height_gate=legacy_no_overfly`、
`overflyable=not_exercised`；不与 C 并表。
