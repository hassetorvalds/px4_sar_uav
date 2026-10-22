## [2026-10-22] 接口卫生：会话 epoch / C 占位标签 / detour 仅 goto

### 变更

- `epoch_id` 改为**会话级**（进程启动生成，可由参数覆盖），与 `task_id` 解耦；
  findings/status/task_accepted 使用会话 id；
- C 的 `anchor_id/transform_id` 改为显式占位：
  `local_map@stage1_placeholder` / `static_arm@stage1_placeholder`；
- detour 只在 `goto` 段启用；takeoff/land/search 不触发（对齐 PX4 按状态分派）；
- `OPEN_QUESTIONS.md` 增加证据等级 `model/injection/real` 与对应决策。

### 验证

```text
· mission 135 passed；全包 271 passed / 2 skipped
· 假 PX4 detour（goto 段 block 8–13s）：PASS
```

### 边界

- epoch 是 WSL 会话 id，不是 PX4 boot time；
- 占位标签不得用于跨会话 C 追溯；
- detour 真机注入仍未触发，真实结构绕行未验证。

---

## [2026-10-22] 新增未决事项/重要决策单一入口

- `docs/OPEN_QUESTIONS.md`：按 `OPEN/BLOCKED/DEFERRED/DECIDED/CLOSED` 记录；
- 当前未决：C 真实偏移/阶段2、Batch2a 进场、绕行真机验证、area 地图外校验、
  epoch_id、anchor/transform、覆盖默认 overlap、起飞净空、VLM JSON；
- 已拍板/已关闭项保留证据与边界，不重复讨论；
- README 进度速览已挂链接；后续契约变化/验证结论/路线选择先更新此文档。

---

# 变更日志

本文件记录本项目**所有**值得留痕的变更：新增功能、结构性调整、验证结果、
已知问题与移除内容。格式参考 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，
按日期倒序排列，最新在最上方。

## 记录规范（后续所有改动都按此写）

1. **每次提交/合入前**：在顶部新增或补充当天条目，不要事后补记。
2. 分类固定为：`新增 / 变更 / 修复 / 移除 / 验证 / 已知问题`，没有内容的小节可省略。
3. 每条都要**可验证**：写清命令、实测数值或文件路径，避免“已优化”“已完善”这类空话。
4. 涉及飞控、接管、failsafe 的改动，必须在 `验证` 中给出触发条件与结果；只跑通正常流程不算验证。
5. 涉及目录搬迁、依赖升级的改动，必须写明**是否需要重建**以及重建命令。
6. 版本基线变化（PX4 / px4_msgs / Agent / ROS 2）单独在 `变更` 中列出前后 commit 或版本号。

---

## [2026-10-22] 新执行器有界侧向绕行（detour）

### 解决的问题

真实高结构进场在 6.3 m 处被障碍挡住，旧执行器直接 `clearance_blocked`/abort；
任务没有“绕过去继续”的能力。新执行器增加有界侧向绕行：

- 解析 `left_m/right_m`，选更宽一侧；
- 每次侧移 2 s、0.25 m/s（约 0.5 m），最多 2 次，间隔 1 s；
- 事件 `detour_start/detour_done`；两次都用尽仍 blocked 才 abort；
- 不改变高度门 C / 语义接口。

### 验证（假 PX4，block 8–13 s）

```text
事件：takeoff_complete → detour_start → detour_done
      → clearance_blocked → detour_start → detour_done
      → hover → return_home → done
PASS（done=True, abort=0）
```

### 边界

- 这是有界侧移，不是全局路径规划；侧向也可能被挡；
- 仅新执行器；旧驱动 Batch2a 仍走原逻辑；
- 未做真实结构绕行验证。

---

## [2026-10-22] Batch2a 高结构第二次：8 m/90 s，仍被真实障碍挡住

```text
session=s1022-201223，384 帧
转场 8.0 m / 90 s；到 ENU E-13.9/N+14.0 后下高
known-point (21.5,-21.9) 换算后 (7.4,-7.6)
距目标 6.3 m 处净空 0.81 m ≤ 1.0 m → 停住
回波判别：后退 0.25 m 后机体偏移明显变化 → 真障碍
→ TRANSIT 超时；候选不可达；land_fast；0 episode
```

- 这次确认了**真实障碍**，不是超时/无结构；但进场路径被挡，仍没有闭合/避让 episode；
- 按“不做无必要测试”停止重复跑；不再硬改点位；
- Batch2a 要跑，需要先解决进场绕行，或换可达点位。

---

## [2026-10-22] Batch2a 真实高结构尝试：转场超时，未完成

```text
场景：真实 TemplateCube E23.5/N-21.9，top≈11.5 m
known-point：E21.5/N-21.9；GEOFENCE_M=35
session=s1022-200701，173 帧
结果：TRANSIT 45 s 超时于 ENU E14.0/N-14.2（未到结构）
      → 候选不可达，land_fast；0 episode
```

- preflight 新逻辑实测：`frames=83`，连续 2 次，等待 5 s；
- **不能**算 Batch2a 数据：没到结构、没有避让/闭合 episode；
- 未硬改点位/时长（服务器要求）；下一步要么换进场路径/提高转场预算，要么先用阶段 1。

---

## [2026-10-21] 高度 C 阶段 1：注入簇 A/B 两档 + 旧驱动 C 门

### 新增

- `mission/height_gate.py`：`legacy` / `c_map_rel` 两模式；C 用 map/REL
  的飞行高度减簇顶，与 PX4/AirSim 高度解耦；记录 `frame_id/anchor_id/transform_id/phase`；
- 旧驱动新增 `height_gate_mode`（默认 legacy，可设 `c_map_rel`），并把门决策
  写进日志；
- `scripts/inject_height_status.py`：向 `/inject/height_status` 注入可控 `cluster_z`；
- `scripts/height_gate_validate.py`：两档 top + 共同/仅 px4 ±0.5 m 的模型级取证。

### 真机注入验证（旧驱动，不依赖真实结构）

```text
A 档 top=1.0 m：px4_z=2.38 airsim_z=3.38 map_z=2.38 gap=1.38
                → overflyable=True，overflyable=1，landed
B 档 top=5.0 m：px4_z=2.37 airsim_z=3.45 map_z=2.37 gap=-2.63
                → overflyable=False → SCAN → land
日志：logs/2026-10-21/c_A4_top1_driver.log / c_B2_top5_driver.log
CSV ：logs/2026-10-21/c_A4_top1.csv / c_B2_top5.csv
```

模型级偏移：`common_drift_invariant=true`、`px4_only_drift_invariant=true`；
取证 `logs/height_gate/c_validation_20261021-195034.json`。

### 结论

- C 的 A/B 两档 **真实注入**通过；A 真飞越、B 不飞越；
- ±0.5 m 偏移目前是模型级（未做真实 PX4 偏移注入）；
- 阶段 2（真实低/高结构各 1 轮）仍未做。

---

## [2026-10-21] 旧驱动 legacy 1 轮（不是高度 C）

### 修复（旧脚本）

- `preflight.sh`：`local_map/status` 等待从 6 s 放宽到 12 s；
  且先 `grep frames=` 再解析，避免 FastDDS SHM 错误行被 `head -1` 当成状态；
- 旧驱动要求单相机：本轮用 `with_down_camera=false` 启动栈。

### 飞行

```text
session=s1021-194003，KNOWN_POINTS="4.6,13.0"
legacy 高度门：一律判障碍、硬禁飞越
242 帧，识别到 0 个障碍区间；0 episode；扫描 25 s 未找到障碍
land_fast，armed=false，landed=true
取证：logs/2026-10-21/legacy_c1_s1021-194003_{driver,vlm,arbiter}.log
CSV：logs/2026-10-21/legacy_c1_s1021-194003_detour.csv
```

### 如实结论

- 这一轮验证了旧驱动的 legacy 路径**未出现飞越**，但**没有遇到障碍**，
  不能当作“高度 C 验证”；
- 旧驱动仍是 legacy；Batch2 仍需等真正 C 判定或服务器接受 legacy 口径。

---

## [2026-10-21] cancel 三态全部验证

```text
active cancel：return_code=0, goals_canceling=1
terminal cancel：return_code=3, goals_canceling=0
unknown goal：return_code=2, goals_canceling=0
result 只发一次；终态后无新 feedback/result
```

对应服务器 A 的三条要求；`task_action_client` 增加 `--cancel-unknown`。

---

## [2026-10-21] n=3 同点位 area_cover + search 高度保持修复

### 修复

- `_run_search` 之前只发水平速度、`up=0`，第 3 轮高度漂到 0.04 m 后
  `clearance_blocked`；改为 `_altitude_hold_vz(position)`，搜索全程保持任务高度。

### 验证（同一 map polygon 4×4，overlap .25，3 轮）

```text
轮1 report_20261021-192645-123：6/6，64/64，pose_age 0.016 s，alt 2.01–2.06
轮2 report_20261021-192735-681：6/6，64/64，pose_age 0.018 s，alt 1.77–1.78
轮3 report_20261021-192831-246：6/6，64/64，pose_age 0.013 s，alt 1.92–1.93
eph/epv 三轮均 0.078/0.077 m；无 abort；落点 E1.78–1.80 / N2.02–2.03
```

结论：同点位 3/3 覆盖 100%，高度保持修复后不再漂到地面。

---

## [2026-10-21] 回信落地：area 1.1 / entry_status / cancel 三态

### 新增

- `constraints.area` 支持 1.1 正式字段：`frame_id`、`polygon`、`line_spacing_m`、`overlap`；
  - polygon 校验 ≥3 点、面积>0、自交检测；`area_cover` 必填；
  - `line_spacing_m` 与 `overlap` 二者取一，同时给以 spacing 为准并写进 notes；
  - 都不给默认 overlap=0.25（90° FOV，spacing=1.5×alt）；
  - `source=vlm` 的 `area_cover` 一律 `OPERATOR_REQUIRED`；
  - polygon 使用 map 坐标：`TaskSpec.coordinate_frame='map'`，执行器不再加起飞原点偏移。
- Finding 增加 `entry_status`、`detector_hits`；detector best_meta 带 `hits/confirm_hits`。
- cancel 三态验证：active -> `ERROR_NONE` + 1 goal；terminal -> `ERROR_GOAL_TERMINATED` + 0 goals；
  重复 cancel 不再产生新 result。

### 验证

```text
· mission 131 passed；全包 267 passed / 2 skipped（检测器入口 268）
· 假 PX4：active cancel return_code=0 goals=1；
           terminal cancel return_code=3 goals=0；
           repeat cancel 3/0；result 仍只一次。
· area polygon 单测：map 坐标、spacing=3.0（overlap .25）、自交拒绝、vlm 拒绝。
```

### 待补

- 同点位/同参数 n≥3 真机样本（服务器要求）；
- `area` polygon 的真实飞行一轮（当前只有相对矩形真机数据）。

---

## [2026-10-21] 真机转场高度剖面：36/36 → 爬升 3 m → 36/36

### 验证

```text
命令：起飞到 2 m，搜索东侧 0 m/r1.5，搜索东侧 7 m/r1.5，降落
· 第一区：search 6 航点，coverage 36/36（100%）
· 转场：transit_climb distance=5.19 m, climb_to=3.0 m
        transit_move → transit_descend(to=2.0) → transit_complete
· 第二区：search 6 航点，coverage 36/36（100%）
· done
报告：logs/tasks/report_20261021-191221-106.jsonl
```

真机确认：转场先爬升/下降，不再是贴任务高度直穿。

---

## [2026-10-21] 真机 --remote-intent：SemanticIntent → TaskSpec → 飞行（64/64）

### 验证

```text
命令：scripts/fly_task.py --remote-intent examples/semantic_intent_area_cover_rel.json --execute
意图：area_cover，4×4 m，REL 2 m，operator，手写 SemanticIntent 1.0
结果：search_plan 8 航点，area 16 m²；
      search_waypoint 8/8；
      coverage 64/64（100%），heatmap logs/coverage/coverage_20261021-191109.png；
      pose_age=0.001 s，eph/epv=0.078/0.077 m；
      landed=true，armed=false。
报告：logs/tasks/report_20261021-191101-802.jsonl
```

这说明 `--remote-intent` 已经真正接上 SemanticIntent 1.0 编译链（不是只跑 `--semantic-intent` 别名）。

---

## [2026-10-21] 契约对齐：--remote-intent 接 SemanticIntent / LANDED_ELSEWHERE / simcheck fail-closed

### 新增

- `--remote-intent` 自动识别 SemanticIntent 1.0（`contract_version/task_type`）并编译；
  旧 IntentFrame 仍走“自然语言 + 本地事实合并”，两者不混用；
- 外部 disarm 先进 `LANDED_ELSEWHERE`，再 `DONE`；status 增加 `landed_elsewhere`，
  仍保证不再发落地命令；
- `task_executor_simcheck.sh` 先 `ros2 topic list` 探域，域不可达/ros2 不可用直接 exit 2。

### 验证

```text
· --remote-intent examples/semantic_intent_area_cover_rel.json --json 输出编译后的 TaskSpec；
· 假 PX4 external disarm：phase 序列 ARM→OFFBOARD→RUN→LANDED_ELSEWHERE→DONE，
  result success=False external_land；
· 全包单测 262 passed / 2 skipped（mission 126）。
```

### 待补

- 真实 `--remote-intent` 飞行数据：当前 AirSim 4560 未监听，UE/仿真未启动；
- 终端 cancel 的 result 语义（ROS 2 Action 终态后无法再产生新 result）需服务器确认。

---

## [2026-10-20] 转场高度剖面：爬升 → 巡航 → 下降（假 PX4 PASS）

### 新增

- 多区域搜索区间距离 > `transit_min_distance_m`（默认 5 m）时：
  - 先在当前位置爬升到 `transit_altitude_m`（默认任务高度 + 1 m）；
  - 巡航到下一区首航点；
  - 下降到任务高度后再开始搜索；
- 事件：`transit_climb → transit_move → transit_descend → transit_complete`；
- 参数：`transit_altitude_m`、`transit_min_distance_m`、`transit_rate_mps`；
- 报告时间线/转场合计支持新事件。

### 验证

```text
· 假 PX4（域 77）双区域：
  transit_climb → transit_move → transit_descend → transit_complete
  → 第二区 search_waypoint → done；PASS，abort=0
· 坐标偏移真机已验证（Task1 落点 + Task2 相对 search 64/64）；
  真机转场因上一落点贴 TemplateCube / AirSim reset 异常暂未复飞。
```

### 已知

- 转场巡航未接前向净空门；默认爬升到 3 m 只是高度剖面，不是完整避障；
- 真机转场待干净起点复飞；
- 起飞前仍无"机体周围净空"检查。

---

## [2026-10-20] 坐标偏移真机验证：goto/search 相对起飞点

### 验证

```text
· Task1（原点起飞）：飞到东侧 5 m、南侧 6 m
  → AirSim 落地 E4.74 / N-5.77，goto 偏移正确
· Task2（从该落点起飞）：搜索东侧 3 m、半径 2 m
  → search polygon E5.61–9.61 / N-7.80–-3.80（= 落点 + 相对坐标）
  → search_waypoint 6/6，下视 coverage 64/64（100%），done
  报告：logs/tasks/report_20261020-200007-994.jsonl
```

结论：任务相对坐标现在真正相对"本次起飞点"，不再被当成世界绝对坐标。

---

## [2026-10-20] 修复：任务坐标按起飞点偏移（search/goto）

### 修复

- `search` 航点与覆盖多边形、`goto` 目标此前直接用任务相对坐标当成世界 ENU；
  只有从世界原点起飞时正确。现在统一加 `_origin_enu`（起飞点）偏移；
- 抽 `offset_waypoints()` 纯函数 + 单测；
- 影响：从任意落点起飞后，任务相对坐标才真正相对当前起飞点。

### 验证

```text
· mission 126 passed；全包 262 passed / 2 skipped（检测器入口 263）
· 真机复飞被环境问题挡住：从上一落点起飞时无人机贴住 TemplateCube，
  AirSim reset 后位置异常（10,-20），搜索未飞出去；
  这不是偏移逻辑本身，需重置 AirSim 后再复飞验证。
```

### 已知

- 尚未在干净起点复飞验证偏移修复；
- 起飞前没有"机体周围净空"检查，落在障碍旁可能起飞即碰撞。

---

## [2026-10-20] 覆盖热力图 + 位姿误差（真机 144/144）

### 新增

- `CoverageGrid.render_ascii()` / `render_png()`：纯标准库 PNG，
  绿=已观测、灰=漏扫、白=多边形外；
- `coverage_monitor` 每 5 s 刷新 `logs/coverage/coverage_<stamp>.png`，
  status 带 `heatmap_path/pose_age_s/eph_m/epv_m/heading_deg/pose_valid`；
- PX4 state 增加 `eph/epv`；报告新增位姿新鲜度/热力图行；
- 前视 0/24 正对照说明：63,984 地面点 + 目标检测，证明是足印不是相机坏。

### 验证

```text
· 单测：mapping 21 passed；全包 261 passed / 2 skipped
  （PYTHONNOUSERSITE=1 时 262）
· 真机 6×6：coverage 144/144（100%）；
  heatmap 144×144 PNG（PIL verify OK）；
  pose_age=0.017 s，eph=0.078 m，epv=0.077 m，heading=36.1°
  报告：logs/tasks/report_20261020-193947-837.jsonl
  热力图：logs/coverage/coverage_20261020-193955.png
```

### 已知

- 热力图 1 m/0.5 m 网格、PNG 为纯色块，不是插值热图；
- `eph/epv` 来自 PX4 估计，不是真值误差；
- 正对照是历史数据（目标检测 + 地面点），未单独做"已知目标放进多边形"实验。

---

## [2026-10-20] Action cancel 幂等 + external_land 收工

### 新增

- `task_action_client --cancel-twice`：第一次 cancel 后 0.3 s 再发一次，测幂等；
- 执行器：已 armed 过但当前 disarmed，且不是自己发起的 LAND（ARM/OFFBOARD/RUN/PAUSED）
  → 记 `external_land`，`done(forced=true, external_land=true)`，`aborted=external_land`，
  **不再发任何落地命令**；
- `fake_px4_sim --disarm-after`；`action_simcheck` 新增 external_land 场景。

### 验证（假 PX4，域 77）

```text
· cancel 两次：result success=False cancel_requested 一次；
  报告 abort(cancel_requested)+done 各一次；land_fast 事件 1 次。
· 外部 disarm：result success=False external_land；
  报告 external_land+done；没有 land_fast。
· 全包单测 260 passed / 2 skipped（检测器入口 261）。
```

### 已知

- ROS 2 Action 终态后再 cancel 不会产生新 result；我们保证 result 只发一次、
  cancel 只生效一次，终态后 cancel 返回终态/拒绝；
- external_land 的判定基于"已 armed 且当前 disarmed"；若 PX4 在起飞前从未 armed，
  不会触发。

---

## [2026-10-20] 安全契约：land_fast 去重 + 旧驱动 legacy 硬禁飞越

### 新增

- `px4_interface/command_dedupe.py`：`CommandDeduper`，`land`/`land_fast` 各自
  2 s 窗口去重；bridge 新增 `land_dedupe_s=2.0`，重复落地命令只记日志不转发。
- 旧 `vlm_closed_loop.py`：`legacy_no_overfly=True`（默认）时
  `overfly_clearance=inf`，一律判障碍、**硬禁飞越**；`height_source` 未知时
  直接 `ValueError`，不再静默回退 px4。

### 验证

```text
· 单测：px4_interface 23 passed；全包 260 passed / 2 skipped
  （PYTHONNOUSERSITE=1 时 261）
· 去重：land_fast 10.0/11.0 只接受一次，12.5 再接受；arm 不去重。
· 旧驱动：py_compile 通过；legacy 分支强制 inf。
```

### 已知

- 去重是 bridge 级、按命令名分别计时；`land` 与 `land_fast` 不共享窗口；
- 旧驱动改 legacy 后未复飞；新执行器不受影响；
- supervisor 接管后的 `LANDED_ELSEWHERE` 状态机仍未实现。

---

## [2026-10-20] Finding 1.0：字段/时间/合并窗口/按 label 半径/history（真机验证）

### 新增

- `Finding`：`schema_version/task_id/epoch_id/frame_id/frame_seq/frame_size/
  first_seen_ros_ns/last_seen_ros_ns/clock_source/pose_at_first_seen/
  revisit_of/discarded/discard_reason/history`；
- 合并：label + 半径（`findings_radius_by_label`，默认 3 m）+ 30 s 窗口；
  窗口外同位置新建并记 `revisit_of`；状态只升级；`discarded` 独立终态；
- detector 新增 `/perception/best_meta`（seq/width/height）；
  执行器订 `CameraInfo` 与 best_meta，填 frame_seq/frame_size；
- `task_id/epoch_id` 取报告名去 `report_` 前缀；
- `findings_merge_window_s`、`findings_radius_by_label` 参数。

### 验证

```text
· 单测：mission 125 passed；全包 257 passed / 2 skipped
  （PYTHONNOUSERSITE=1 时 258）
· 真机扫描 45 s：
  finding target [17.45, 34.36, 4.65]
  frame_seq=10，frame_size=[1280,720]，first_seen_ros_ns 已填，
  pose_at_first_seen={world_enu:[-0.04,-0.1,0.86], heading_deg:60.2}，
  history 1 条；status=confirmed（本轮目标在起飞阶段确认，未进 inspected）
  报告：logs/tasks/report_20261020-193009-416.jsonl
  持久化：logs/findings/findings_20261020-193009-416.json
```

### 已知

- 目标在 takeoff 阶段确认，未触发扫描/对准，status 停在 confirmed；
- `frame_seq` 来自 best_meta（detector 序号），不是 PX4 时间戳；
- `merge_window_s` 用 monotonic；跨任务不做空间去重。

---

## [2026-10-20] SemanticIntent 1.0：手写 JSON → TaskSpec → Action（真机 144/144）

### 新增

- `mission/semantic_intent.py`：SemanticIntent 1.0 校验/稳定拒绝码/编译；
- 支持 `area_cover / go_to / inspect / return`；`patrol` 需 area；
- `constraints.area` 为 WSL 扩展（服务器 1.0 没有区域几何）；
- `fly_task.py --semantic-intent`（直接执行）；CLI `semantic_intent`（编译/拒绝）；
- 示例 `examples/semantic_intent_area_cover.json`；`docs/SEMANTIC_INTENT.md`。

### 验证

```text
· 单测：mission 121 passed；全包 253 passed / 2 skipped
  （PYTHONNOUSERSITE=1 时 254）
· 真机：
  SemanticIntent（operator，area_cover，6×6 m、REL 2 m）
  → TaskSpec（takeoff/search/land）
  → ExecuteTask Action（goal=TaskSpec JSON）
  → search_waypoint 8/8，area=36 m²，path=30 m，足印 4 m/重叠 50%
  → coverage 144/144（100%），漏扫 0
  → result success=True
  报告：logs/tasks/report_semantic_action_20261020-192310.jsonl
  证据页：logs/tasks/report_semantic_action_20261020-192310.md
```

### 已知

- 模型仍不产 JSON；本轮是手写 `source=operator`；
- `constraints.area` 是扩展，待服务器正式字段；
- `patrol` 仅 area 形式；`keepout` 只检查目标点，不检查整条航迹。

---

## [2026-10-20] 区域最近邻排序 + 服务器接口信

### 新增

- `mission/regions.py`：多 search step 按区域中心最近邻重排，其他 step 位置不变；
- `fly_task.py --order-regions`；
- 给服务器侧的信：`docs/agent_comm/agent_reply.md`（上一封归档为
  `114_reply_height_gate_c1.md`），询问 VLM 输出格式、findings schema、高度源、Action 调用方式。

### 验证

```text
· 单测：mission 115 passed；全包 247 passed / 2 skipped
  （PYTHONNOUSERSITE=1 时 248）
· 合成 3 区域：起点 (0,0) → (5,5) → (10,0) → (0,10)；
  takeoff/land 位置不变，notes 记录排序。
```

### 已知

- 只做区域顺序；转场仍是直线，未做高度剖面/避障；
- 排序不做 TSP 最优，只最近邻。

---

## [2026-10-20] 多区域 + 转场腿：36 m²、144/144（100%）

### 新增

- 多区域用多个 `search` step 组合，每区独立多边形/矩形；
- 执行器在区间直飞，报告 `transit`（from/to/distance）；
- Markdown 新增「搜索区域」表、合计覆盖、转场段数/距离；
- `examples/polygon_l_task.json`、`examples/multi_region_task.json`。

### 验证（0.5 m 网格、下视）

| 区域 | 面积 | 航点 | 覆盖 | 漏扫 |
|---|---|---|---|---|
| A L 形 | 20 m² | 8 | 80/80（100%） | 0 |
| B 矩形 4×4 | 16 m² | 6 | 64/64（100%） | 0 |

```text
· 转场：1 段，10.86 m（E2.25/N-3.22 → E12/N-8）
· 合计：36 m²，144/144 格（100%），漏扫 0
· 报告：logs/tasks/report_20261020-190400-083.jsonl
· 证据页：logs/tasks/report_20261020-190400-083.md
· 单测：mission 112 passed；全包 244 passed / 2 skipped
  （PYTHONNOUSERSITE=1 时 245）
```

### 已知

- 转场是直线，未做避障/高度剖面；
- 区域按顺序扫，未做最优排序；
- 0.5 m 网格、平面 Blocks 场景。

---

## [2026-10-18] 覆盖率敏感性：2m/4m 97.2%（漏 4 格），3m/6m 100%

### 变更

- 覆盖网格默认 1.0 → **0.5 m**（更严格）；执行器新增 `coverage_cell_size_m`，
  随 `/coverage/polygon` 下发；`coverage_monitor` 采用该值。

### 真实数据（6×6 m 矩形，下视 FOV 90°）

| 高度 | 间距 | 足印 | 重叠 | 航点 | 覆盖 | 漏扫 |
|---|---|---|---|---|---|---|
| 2 m | 3 m | 4.0 m | 25% | 6 | 144/144（100%） | 0 |
| 2 m | 4 m | 4.0 m | 0% | 6 | 140/144（97.2%） | 4 |
| 3 m | 6 m | 6.0 m | 0% | 4 | 144/144（100%） | 0 |

报告：

```text
logs/tasks/report_20261018-175144-995.jsonl
logs/tasks/report_20261018-175308-305.jsonl
logs/tasks/report_20261018-175430-488.jsonl
```

### 结论

- 固定 2 m 高度，间距 3 m（25% 重叠）全覆盖；间距 4 m（0% 重叠）漏 4 格；
  覆盖数据对间距敏感，不是恒 100%；
- 3 m 高度足印 6 m，间距 6 m 仍全覆盖——覆盖率主要由足印/间距比决定；
- 0.5 m 网格比旧 1 m 更严格，旧数据不能直接横向比较；
- 单测不变：243 passed / 2 skipped（mission 111，mapping 20）。

### 已知

- 平面 Blocks 场景，未测地形/风/航向；
- 只测了一个 6×6 区域；多区域转场腿未做。

---

## [2026-10-18] 多边形航迹 + 覆盖率敏感性：L 形 8/8、20/20

### 新增

- `lawnmower_polygon()`：任意多边形（含凹形）扫描线裁剪、可旋转；
- `search` 支持 `shape=polygon` + `vertices/polygon` 顶点，TaskSpec 校验围栏；
- `search_metrics` 增加下视足印/重叠率：`footprint_width_m`、`lane_overlap_ratio`；
- `fly_task.py --task-json`：直接执行 TaskSpec JSON（多边形等解析器不覆盖的结构）；
- 报告打印面积/折线/预计/足印/重叠。

### 验证

```text
· 单测：mission 111 passed；全包 243 passed / 2 skipped
  （PYTHONNOUSERSITE=1 时 244）
· 真机 L 形（20 m²、8 航点、下视 2 m）：
  search_plan area=20.0 m²，path=20.5 m，est=54.9 s，
  footprint=4.0 m，overlap=50%
  search_waypoint 1/8 … 8/8
  search_coverage 20/20（100%），漏扫 0，地面点 2,725,200
  → done；arming_state=1、landed=true
  报告：logs/tasks/report_20261018-174709-451.jsonl
  证据页：logs/tasks/report_20261018-174709-451.md
· 观测：搜索段 LCL z 2.08→2.71（漂移），未 abort。
```

### 已知问题

- 多边形只从 JSON 输入，自然语言还不解析任意顶点；
- 多块不连通区域/转场腿未优化；
- 覆盖率是 1 m 网格，高度/间距敏感性只做了 2 m/2 m 一组；
- LCL 高度漂移仍存在。

---

## [2026-10-18] 下视相机：观测覆盖 0/24 → 24/24

### 配置/代码

- AirSim settings 增加 `Down` 相机（Pitch -90、FOV 90、640×480），备份
  `settings.json.bak-20261018-173936`；UE Stop→Play 重载；
- `airsim_camera` 新增 `camera_pitch_deg`；下视时点云光学系→机体 FLU 用
  `[-y_cam, -x_cam, -z_cam]`；
- 启动第二个 `airsim_camera_down` 节点；`coverage_monitor` 改订
  `/airsim_camera_down/points`；前视/检测不变。

### 验证（同一矩形 6×4 m、6 航点、2 m）

```text
· 前视：0/24（0.0%），地面点 63,984，区域内 0
· 下视：24/24（100.0%），地面点 2,745,600，区域内 1,170,872
· 报告：logs/tasks/report_20261018-174221-192.jsonl
· 证据页：logs/tasks/report_20261018-174221-192.md
```

### 结论

- 覆盖瓶颈确实是前视足印，不是网格/航迹；
- 下视相机让覆盖有意义；下一步可做多边形航迹、覆盖率随高度/重叠的敏感性。
- 单测不变：239 passed / 2 skipped（mapping 20，mission 107）。

---

## [2026-10-18] 观测覆盖监视：真实点云 + 负结果（0/24）

### 新增

- `mapping/coverage.py`：`CoverageGrid`（点在多边形、1 m 网格、地面点过滤、
  覆盖率/漏扫/点数统计）；
- `mapping/coverage_monitor.py`：相机点云 → ENU → 地面网格，发布 `/coverage/status`；
- 执行器：搜索开始时发布 `/coverage/polygon`，结束/`done` 带 `coverage`；
  报告新增「观测覆盖」行；
- `docs/COVERAGE.md`。

### 验证（真实数据）

```text
· 单测：mapping 20 passed；mission 107 passed；全包 239 passed / 2 skipped
  （PYTHONNOUSERSITE=1 时 240）
· 真机矩形 6×4 m、6 航点：
  1,062,311 点；地面点 63,984；落在多边形内 0；覆盖 0/24（0.0%）
  报告：logs/tasks/report_20261018-173323-406.jsonl
· 落地点单帧：地面点 2980，east 3.68–10.97，north -3.60–3.54，
  median (7.70, -3.18)；多边形 north -8..-4 → 地面点整体在区域外（前方）
```

### 结论

- 不是网格/坐标 bug，是**前视相机足印**问题：2 m 高度看到的是前方数米到十几米，
  不是脚下/航线两侧；6×4 m 区域覆盖为 0；
- 继续加多边形航迹不会改善覆盖；先加下视相机/云台或航点扫描。

### 已知

- 覆盖监视需要执行器先发 `/coverage/polygon`；无多边形时不统计；
- 1 m 网格、`|z|≤0.6 m` 地面判据；地形起伏/高差会误差；
- 下视相机/云台、航点扫描尚未做。

---

## [2026-10-18] 覆盖规划升级：矩形/旋转搜索 + 覆盖指标（真机 6/6）

### 新增

- `search_pattern.lawnmower_rectangle()`：矩形区域、可绕中心旋转的弓字航点；
- `search_metrics()`：面积/折线长度/预计时间；
- `search` step 支持 `shape=rectangle`、`width_m/height_m/angle_deg`；
- 解析器支持"搜索东侧 5 米、南侧 6 米、宽 6 米、长 4 米的矩形区域"；
- 执行器 `search_plan` 报告带 `area_m2/path_length_m/estimated_s`；
- 报告/CLI 打印矩形摘要与覆盖指标。

### 验证

```text
· 单测：mission 107 passed；全包 234 passed / 2 skipped
  （PYTHONNOUSERSITE=1 时 235）
· 假 PX4：矩形搜索 search_plan + 6/6 search_waypoint + done PASS
· 真机（起飞 2m → 矩形 6×4 m、r=0、center E5/N-6 → 降落）：
  takeoff LCL 1.977 / REL 2.0
  search_plan 6 航点，area=24.0 m²，path=22.0 m，est=54.0 s
  search_waypoint 1/6 … 6/6（22.8–64.2 s），z 1.72–2.10 m
  → done；arming_state=1、landed=true
  报告：logs/tasks/report_20261018-172142-008.jsonl
  证据页：logs/tasks/report_20261018-172142-008.md
```

### 已知问题

- 矩形只做轴对齐/绕中心旋转，多边形/障碍内裁剪未做；
- 覆盖指标是几何估计，不含风/转弯半径/相机 FOV 重叠；
- 旋转角超过 ±180° 未支持。

---

## [2026-10-18] P3 收尾：findings 持久化 + 跨任务汇总

### 新增

- `mission/findings_store.py`：
  - `persist_findings()` 原子写 `logs/findings/findings_<task_id>.json`，
    并追加 `logs/findings/index.jsonl`；
  - `load_records()` 优先读 index、按 task_id 去重；
  - `summarize_records()`/`render_summary()` 输出任务数/发现物数/类别/状态/表格。
- 执行器：`findings_dir`、`persist_findings` 参数；`_finish_done()` 在写
  `done` 前落盘，报告 `findings_saved`，`done` 带 `findings_path/findings_task_id`。
- CLI `ros2 run mission task_findings`（`--json` / `--md` / `--limit`）。
- `docs/FINDINGS.md`。

### 验证

```text
· 单测：mission 102 passed；全包 229 passed / 2 skipped
  （PYTHONNOUSERSITE=1 时 230）
· 真机 Action 扫描 45 s：
  finding #1 target [31.22, 29.69, 4.76] → inspected
  落盘 logs/findings/findings_action_fstore_20261018-171431.json + index.jsonl
  task_findings 汇总：任务 1、有发现 1、发现物 1、状态 inspected
  Markdown 含「发现物」表 + 发现物记录路径
```

### 已知问题

- 跨任务不做空间去重（同一目标在不同任务会各记一条）；
- 没有数据库/索引服务，只有 JSON JSONL + 文件；
- `task_id` 冲突时文件覆盖，正常每任务只写一次。

---

## [2026-10-18] P3：findings 模型 + Action result / Markdown 证据页

### 新增

- `mission/findings.py`：`Finding` / `FindingsBook`
  - 同一目标按世界距离（默认 3 m）合并，位置取增量平均，减少单帧深度误差；
  - `status` 只升级不降级（candidate → confirmed → inspected → resolved）；
  - `priority` 由类别映射（person/victim=high、vehicle=medium、其他 unknown，不猜）；
  - `findings_merge_radius_m` 可配。
- 执行器：目标确认时写 `finding(action=confirmed)`，对准时写
  `finding(action=inspected)`；`/mission/status` 与 `done` 都带 `findings`。
- 报告：`mission/report.py` 新增「发现物」表（ID/类型/位置/置信度/优先级/状态/首次/最后）。
- Action：`ExecuteTask.Result` 新增 `findings_json`，result 里直接回传结构化发现物。

### 验证

```text
· 单测：mission 97 passed；全包 224 passed / 2 skipped
  （PYTHONNOUSERSITE=1 时 225）
· 真机 Action（起飞 2m → 扫描 45s → 降落）：
  target_confirmed world=[36.19, 35.88, 5.73]（t=37.68）
  → target_centered 4.4°
  → finding #1 status=inspected（first/last=37.68/39.71）
  → result success=True，findings_json 回传
  报告：logs/tasks/report_action_findings_20261018-171030.jsonl
  证据页：logs/tasks/report_action_findings_20261018-171030.md（「发现物」表）
```

### 已知问题

- 单次扫描只确认一次时 `updates=1`，合并逻辑主要靠单测覆盖；
- 置信度仍是检测器确认的 1.0，没有把 bbox/深度质量映射进去；
- 优先级对 `any` 保持 unknown，不臆造；
- findings 尚未持久化到跨任务数据库。

---

## [2026-10-18] Action 完整任务复飞：搜索 8/8 + 扫描检测 + 对准 + 落地

### 验证

```text
· Action goal = 完整 TaskSpec：
  起飞 2 m → 搜索东侧 5 m/南侧 6 m、r=3 m（8 航点）→ 原地扫描 60 s → 降落
· feedback：IDLE → ARM → RUN(takeoff/search/scan/land) → LAND → DONE
· search_waypoint 1/8 … 8/8（30.5–87.1 s）
· scan_dwell 167/-147/-103/-59/-14/29/94/139/-176°
  → target_confirmed world=[29.19, 30.78, 4.87]（t=123.96, step=scan）
  → target_centered error=3.5°
  → scan_complete → done
· health_check 通过；takeoff LCL 1.971 / REL 2.0；
  arm/offboard/land_fast 全 completed；arming_state=1、landed=true
· result：success=True，report_path=logs/tasks/report_action_full_20261018-170422.jsonl
· 证据页：logs/tasks/report_action_full_20261018-170422.md
```

### 结论

- `ExecuteTask` 不只能跑最小任务，完整 TaskSpec（搜索+扫描+检测）可直接作为 goal；
- 单测不变：217 passed / 2 skipped（mission 90）；
- README/CHANGELOG 已同步。

---

## [2026-10-18] P2：ExecuteTask ROS 2 Action 接口（复用 TaskSpec/执行器）

### 新增

- `mission_interfaces/action/ExecuteTask.action`：goal = TaskSpec JSON；
  feedback = 阶段/步数/步型/进度/LCL 高度；result = success/message/report_path。
- `mission/task_action_server.py`：薄适配层，发布 `/mission/task`、订阅
  `/mission/status`，不重写控制；一次一个 goal，非法/忙碌/状态过期直接拒绝。
- `mission/task_action_client.py`：`--task-file/--task-json`、`--cancel-after`。
- 执行器：`/mission/cancel`（Empty）→ `_abort('cancel_requested')` 落地上锁；
  状态补 `step_total/report_path/aborted/abort_reason`；DONE 时先发一条状态再退出
  （否则 Action server 收不到结果）。
- `scripts/run_action_server.sh`、`scripts/action_simcheck.sh`；
  Action 自检默认 `ROS_DOMAIN_ID=77`。
- `docs/ACTION.md`。

### 验证

```text
· 单测：mission 90 passed；全包 217 passed / 2 skipped
  （PYTHONNOUSERSITE=1 时 218）
· 假 PX4 正常：goal 接受 → feedback ARM/RUN/LAND/DONE → success=True → 报告
· 假 PX4 取消（4 s）：success=False, message=cancel_requested；
  报告 abort(cancel_requested) + done；安全落地
· 真机最小 Action：goal 接受 → success=True；
  health 通过，arm ACK 0.007 s、offboard 0.012 s；
  takeoff LCL 2.011 / REL 2.0 → done，arming_state=1、landed=true
  报告：logs/tasks/report_action_20261018-170128.jsonl
```

### 已知问题

- 仍是单 goal、无并发/抢占；飞行原语仍是 TaskSpec 的 step，不是独立
  Takeoff/Goto/Search Action 类型；
- cancel = 落地上锁，不支持取消后恢复；
- 尚未接 YAML/BT；BT 可直接把 TaskSpec 作为 goal。

---

## [2026-10-18] Demo 收口：目标自动对准 + Markdown 证据页

### 新增

- 执行器 `center_on_target`（scan/hover 阶段）：确认目标后保持位置/高度，
  用世界点算方位并 yaw 对准，报告 `target_centered`（含 `error_deg`）。
  search 阶段的 hold 不偏航，避免改变航向影响净空门方向。
- `mission/report.py`：把 `report_*.jsonl` 汇总成 Markdown 证据页
  （任务/意图/健康检查/目标/命令生命周期/时间线）；`fly_task.py` 执行后自动生成
  同目录 `report_*.md`，也可 `ros2 run mission task_report <jsonl>`。
- `scan` 驻留默认 1.5 → **2.5 s**（2.5 Hz 相机约 6 帧，给 3 帧确认留余量）。

### 修复

- 起飞阶段收到目标确认时，`_advance_step()` 被 target hold 阻塞，
  `takeoff_complete` 重复写 8 次、扫描晚开始；takeoff 现在 `ignore_hold=True` 前进。

### 验证

```text
· 单测：mission 88 passed；全包 215 passed / 2 skipped
  （PYTHONNOUSERSITE=1 时 216）
· 真机（起飞 2m → 原地扫描 60s → 降落）：
  health_check 通过；takeoff LCL 1.996 / REL 2.0；
  target_confirmed world=[32.98, 26.51, 4.72]（step=scan）
  → target_centered error=-3.4°（heading 42.5°）
  → scan_dwell 85/130/172… → scan_complete → done；arming_state=1、landed=true
  报告：logs/tasks/report_20261018-164832-632.jsonl
  证据页：logs/tasks/report_20261018-164832-632.md
```

### 已知问题

- 目标世界 z≈4.72 与 `OrangeBall` 位姿 z≈3.68 仍有偏差（深度/像素近似）；
- LCL 高度漂移仍在（起飞 LCL 1.996 与 REL 2.0 对齐，长航段会漂）；
- 对准只在 scan/hover 生效；search hold 保持航向。

---

## [2026-10-18] P1 安全加固：arm 前健康检查 + 自检域隔离 + 高度门帧标签

### 新增

- `mission/health.py`：起飞前健康检查（纯逻辑）——PX4 状态新鲜、本地位置有效/新鲜、
  `pre_flight_checks_pass`、failsafe、电池（可选 require）五项；返回逐项结果与失败原因。
- 执行器订阅 `/fmu/out/battery_status_v1`；PHASE_ARM 在 `arm_settle` 后先过健康检查，
  **不通过就不发 arm**（fail-closed），报告/状态新增 `health_check` 与 `health`。
- `scripts/task_executor_simcheck.sh`：
  - 默认 `ROS_DOMAIN_ID=77`，与真实栈物理隔离；
  - 保留"检测到真实 PX4/offboard_bridge 默认拒绝"的互锁（`SIMCHECK_ALLOW_LIVE=1` 才并跑）。
- `scripts/vlm_closed_loop.py`：高度门/CSV 补帧标签 `px4_frame=LCL`、`truth_frame=TRUTH`、
  `gate_frame=LCL|TRUTH`；`height_source=airsim` 时打印"TRUTH 仅诊断"警告；
  SEEK/DESCEND 日志带帧。

### 验证

```text
· 单测：mission 86 passed；全包 213 passed / 2 skipped
  （PYTHONNOUSERSITE=1 时 214）
· 真机（起飞 2m→悬停 5s→降落）：
  health_check 在 0.53/1.63 s 因 status/position 未就绪而拒绝 arm；
  2.03 s 通过（px4_status age=0.10s、position age=0.02s、preflight=True、failsafe=False、
  battery=unknown/warn）→ arm ACK 0.008 s、offboard ACK 0.012 s；
  takeoff LCL 1.986 / REL 2.0 → done；arming_state=1、landed=true。
  报告：logs/tasks/report_20261018-095424-586.jsonl
· 互锁：真实栈在线时 simcheck 默认 exit 2、fake 进程 0；
  SIMCHECK_ALLOW_LIVE=1 + 域 77 时 simcheck PASS，真机仍 arming_state=1、landed=true。
· 假 PX4 自检（域 77）：health_check → command_ack → command_completed → done PASS。
```

### 已知问题

- SITL 的 `battery_status_v1` 无消息 → 电池按未知处理（warn，不阻断）；
  实机/有电池仿真应设 `require_battery:=true` 并给阈值。
- 旧驱动 `vlm_closed_loop.py` 只补了帧标签与警告，**未复飞**；高度门数据源仍是
  px4/airsim 二选一，下一步可切到"相对高度"并统一到 `mission.altitude`。
- 健康检查目前不查 EKF flags（只依赖 `pre_flight_checks_pass` + 位置新鲜）；
  若需要更严格，可接 `estimator_status_flags`。

---

## [2026-10-18] 安全事件：simcheck 与真实栈同域误 arm；已加互锁 + 安全案例 6

### 事件

真实栈在线时运行 `task_executor_simcheck.sh`。假 PX4 与真实栈共用
`/offboard_bridge/command` 和 `/fmu/out`，自检执行器的 `arm/offboard` 被真实
`offboard_bridge` 转发给真 PX4，真机被 arm 到 OFFBOARD，AirSim 高度约 1.8 m。

### 处置

- 立即 kill 假 PX4/自检执行器；
- 向 `/offboard_bridge/command` 发 `land_fast`；
- 8 s 后：`arming_state=1`、`landed=true`、`at_rest=true`、AirSim z≈-0.06 m；
  无碰撞、无损伤。

### 修复

- `scripts/task_executor_simcheck.sh` 增加**安全互锁**：检测到真实 PX4 或
  `offboard_bridge` 在跑时默认拒绝启动（exit 2）；确需并跑必须显式
  `SIMCHECK_ALLOW_LIVE=1`；
- `docs/safety_casebook.md` 增加案例 6；
- 通则：假飞行器/回放/自检必须与真实控制链路隔离（不同 `ROS_DOMAIN_ID` 或显式互锁）。

### 验证

```text
· 真实栈在线时运行 simcheck → 退出码 2，fake_px4_sim 进程数 0（互锁生效）
· 误 arm 后 land_fast → arming_state=1、landed=true、AirSim z≈-0.06 m
```

---

## [2026-10-18] 搜索+检测闭环：yaw/scan 原语 + victim_detector 接入（真机）

### 新增

- `TaskSpec`/解析器/执行器新增 **`yaw`（原地转向）** 与 **`scan`（原地扫描）**：
  - `yaw`：相对角度 ±1–360°、速率 5–60°/s，ENU 正=左；
  - `scan`：1–120 s、速率 5–60°/s，扫描期间如果收到确认目标会暂停偏航；
  - 解析支持"向左/右转 N 度""原地扫描 N 秒"；报告新增 `yaw_start/yaw_complete/scan_complete`。
- 执行器 `hold_once_per_step`（默认 true）：单步内只对第一个确认目标悬停一次，
  避免目标持续可见时反复触发、扫描永远走不完。
- 检测器接线：`best_topic` 统一到 `/perception/best`（launch 与 `run_detector.sh`），
  执行器默认订阅同一话题。
- **速度模式高度保持**：`_altitude_hold_vz()`（P 控制，±0.3 m/s）；扫描改**步进 45° + 驻留 1.5 s**，
  给检测器 3–4 帧确认时间。

### 修复

- `perception/victim_detector.py`：`state.position_ned()` 误当方法调用（property 实际是 tuple），
  与执行器同源 bug；改为 `state.position_ned`。
- 检测器 `max_depth_m` 30 → **60 m**：场景目标实测 32 m，原来刚好被上限滤掉，
  表现为"检出但 depth_m/world_enu 全 null"。

### 验证

```text
· 单测：mission 76 passed；全包 203 passed / 2 skipped
  （PYTHONNOUSERSITE=1 时 204）
· 假 PX4 自检：yaw_complete → scan_complete → done PASS
· 真机扫描（起飞 2 m → 原地扫描 40 s → 降落）：
  target_confirmed world=[30.77, 27.82, 5.01]（step=scan）
  scan_complete → land_fast → done；arming_state=1、landed=true、at_rest=true
  报告：logs/tasks/report_20261018-093926-672.jsonl
· 组合任务（南侧搜索 8 航点 + 步进扫描 30 s）：
  search_waypoint 1/8 … 8/8 → scan_dwell（126/171/-144/-99/-54°）
  → target_confirmed world=[27.41, 30.66, 4.95]（t=76.1, step=scan）
  → scan_complete → land_fast → done；无 abort；报告
  logs/tasks/report_20261018-094737-473.jsonl
· 观测：搜索段 LCL z 1.39→0.67（EKF 漂移），扫描驻留段回到 2.01；
  无触地；说明帧标签/漂移仍需在 P1 统一处理。
```

### 已知问题

- 起飞阶段也会收到 `target_confirmed`（执行器只在 scan/search 阶段暂停，不影响起飞）；
- 目标世界 z≈5.0 m，与 `OrangeBall` 位姿 z≈3.7 m 有偏差（深度/像素近似），
  作为"发现并上报"够用，精确三维定位另做；
- "搜索路径 + 扫描"组合任务尚未真机飞（当前是原地扫描闭环）。

---

## [2026-10-18] P0：IntentFrame + 高度帧 + 命令 ACK 生命周期（假 PX4 自检）

### 新增

- `mission/altitude.py`：高度帧 `REL / LCL / BARO / MSL / TRUTH`；`format_altitude`/
  `require_same_frame`/`to_dict`；`TRUTH` 只允许诊断。
- `mission/intent.py`：`IntentFrame`（typed facts + provenance + confidence + missing）；
  `merge_intent()` 实现"本地事实优先"——VLM 改写本地距离/高度/目标会被拒绝并记 warning。
- `mission/command_tracker.py`：命令生命周期
  `queued → awaiting_ack → accepted → executing → completed`，
  以及 `rejected / timeout / delivery_unknown`；`land_fast` 用 `expect_ack=False`。
- 三个模块共 21 项单测；`mission` 单测 69 passed。

### 变更

- `mission/task_spec.py`：新增 `altitude_frame`（默认 REL，TRUTH 被拒绝）；
  JSON 往返带帧标签。
- `mission/task_executor.py`：
  - 订阅 `/fmu/out/vehicle_command_ack_v1`，用 `CommandTracker` 跟踪
    `arm/offboard/land_fast`；报告新增 `command_sent / command_ack /
    command_timeout / command_completed / commands`；
  - arm/offboard ACK 被拒绝或重试用尽 → 收工降落；
  - `takeoff` 用 `ground_lcl + altitude_rel` 生成 LCL 目标，**比较不再跨帧**；
  - 新增 `arm_settle_s`（默认 0.5 s）：进 ARM 后先静置再发第一条 arm，避免 ACK 订阅
    尚未匹配时首条命令 ACK 丢失。
  - 报告/状态新增 `altitude_lcl` 与 `task_altitude`（带帧/来源）。
- `scripts/fly_task.py`：改用 `parse_intent()`；打印来源/置信度/缺失/provenance；
  新增 `--intent-json`、`--remote-intent`；落盘 `intent_<stamp>.json` 并传给执行器
  （报告里的 `intent_path`）。
- `scripts/fake_px4_sim.py`：对 arm/offboard/land/disarm 发布 `VehicleCommandAck`，
  让假 PX4 自检覆盖 ACK 生命周期。

### 验证

```text
· 全包单测：196 passed / 2 skipped（mission 69 / vlm 45 / perception 28+1skip /
  px4_interface 20 / planner 18 / mapping 15 / px4_offboard 1+1skip）
· 假 PX4 自检（SIMCHECK_EXPECT=command_ack,command_completed,done）PASS：
  command_sent(arm) → command_ack(accepted, 0.001 s) → command_completed
  → command_sent(offboard) → command_ack → command_completed
  → takeoff → ... → land_fast → command_completed → commands → done
· CLI 合并：本地"向北 10 m" + 远端"向北 20 m" → 保留 10 m 并打印拒绝 warning
· 真机复飞（3 次，AirSim+PX4）：
  - 前两次：首条 arm 的 ACK 没收到 → command_timeout → 重试后 accepted（0.815–0.914 s）
  - 加 arm_settle_s=0.5 后：arm ACK **0.011 s**、offboard **0.016 s**，首次即 accepted
  - takeoff：地面 LCL 0.02 + 任务 REL 2.0 → LCL 2.008，帧换算正确
  - land_fast → command_completed → done；arming_state=1、landed=true、at_rest=true
  - 报告：logs/tasks/report_20261018-092416-359.jsonl
· 小半径搜索真机复飞（东南侧 center E5/N-3、r=3 m、8 航点）：
  - takeoff LCL 2.01 vs REL 2.0；search_plan 8 航点
  - search_waypoint 1/8 … 8/8 全部到位（7.0→74.8 s），无 abort / 无 clearance_blocked
  - land_fast → done；AirSim 真值 north 0.02 / east 2.14（最后一个航点）、仅 Ground_5 碰撞
  - 报告：logs/tasks/report_20261018-092917-110.jsonl
```

### 已知问题

- 首条 arm 命令依赖 0.5 s 静置（`arm_settle_s`）才能稳定拿到 ACK；
  若后续出现首条命令 ACK 丢失，优先检查 DDS 订阅匹配与 `arm_settle_s`。
- 真实 PX4 的 ACK 话题为 `/fmu/out/vehicle_command_ack_v1`；若 PX4 版本变化需同步。
- 高度门（`vlm_closed_loop.py`）仍用 `height_source` 二选一，尚未统一到本次帧标签；
  这是下一步（P1）的事。

---

## [2026-10-18] 真机复验 + local_map 回归修复 + 参考仓库对照

### 修复

- `mapping/voxel_grid.cluster_centroid_in_cone`：空地图提前返回 **3 元组**，而调用方解包 6 个
  → `local_map` 启动即崩（`not enough values to unpack`）。改为始终返回 6 元组
  `(None, 0, inf, None, None, None)`，并补两条回归测试（空地图/高度范围）。
  与 b1–b5 的崩溃同源；此前只重启栈、未修根因。
- `mission/task_executor.py`：
  - 启动时 `status_age_s()=inf` 被误判为"PX4 状态陈旧"，报告里写了 3 条假 `abort`；
    现在 ARM/OFFBOARD 阶段等待首个状态，由阶段超时兜底；
  - 任务完成后节点常驻、CLI 一直挂到超时；新增 `exit_on_done`（默认 true），
    先写 `done` 再退出；
  - `_finish_done()` 统一收尾，避免"退出早于写 done"。
- `scripts/fly_task.py`：子进程找不到 `ros2`（未 source 环境）→ 自动
  `bash -c 'source scripts/env.sh && exec …'`。
- `scripts/task_executor_simcheck.sh`：去掉 `pkill -f`（模式可能匹配外层作业，实测会
  把整条命令 SIGTERM）；改为 `kill` + `wait`。

### 验证（真机复验，**不是新能力**）

```text
最小任务（起飞 2 m → 悬停 5 s → 降落）：
  report_20261018-090236-211.jsonl
  task_accepted → ARM → OFFBOARD → RUN → takeoff_complete(1.9 m) → hover → land
  → done(15.8 s)，无 abort；arming_state=1、landed=true、at_rest=true

前进/返航（起飞 2 m → 向前 3 m → 悬停 3 s → 返航降落）：
  report_20261018-090305-803.jsonl
  takeoff_complete(1.85 m) → forward（~2.6 m）→ hover → return_home → land → done(24.4 s)
  无 abort；安全上锁；AirSim 碰撞对象 Ground_2

观测：done 时 PX4 local z = -0.54 / -0.57（EKF 触地漂移），AirSim 真值在地面
  → 直接支持 docs/REFERENCES.md 的"高度必须标帧（REL/LCL/BARO/MSL）"结论
```

### 新增

- `docs/REFERENCES.md`：参考仓库对照与设计结论（意图帧/确认门/命令生命周期/
  高度帧/覆盖规划/发现物模型；哪些借、哪些不借）。
- 本地 `reference/` 克隆（已 gitignore）：
  - `px4-ros2-interface-lib`（BSD-3）、`px4_ros_com`（BSD-3）；
  - `mavsdk_drone_show`（PolyForm，仅概念参考，未复制代码）；
  - `px4-offboard`（BSD-3，最小 Offboard + RViz visualizer）；
  - `UAV-Search`（无 LICENSE，仅思路：正下视定位/离线规划→任务上传）；
  - `aerial-autonomy-stack`（**MIT，首选架构参考**：ROS 2 Action 飞行原语 +
    YAML 行为树 + 黑板上条件 + 弓字搜索的 fix 确认/stand-off）；
  - `PX4_Firmware`（casy-lab，partial；雪崩 ARVA/ES 学术分支，领域不同，未深入）。

### 已知问题

- `local_map` 修复后建议 `colcon build --symlink-install --packages-select mapping`，
  确保 `build/` 副本与源码一致。
- 参考仓库只做设计对照；`docs/REFERENCES.md` 中不含任何复制代码。

---

## [2026-10-18] 自然语言指挥飞行：TaskSpec/解析器/执行器（最小/前进/返航已飞验）

### 新增

- `mission/task_spec.py`：**唯一可控中间层**。`Step`（takeoff/goto/forward/search/hover/
  land/return_home）+ `TaskSpec`，含 schema 版本、范围校验（高度 0.5–10 m、速度 0.1–1.0 m/s、
  时限 10–600 s、围栏 2–50 m）、顺序校验（首个必须 takeoff、末尾必须 land/return_home）、
  JSON 往返。
- `mission/task_parser.py`：规则版自然语言 → TaskSpec。支持中英混合、全角数字、中文数字
  （一–九十九）、方向/距离、机体前向、悬停、返航、搜索（中心/半径/间距/目标颜色/人员）；
  无法识别或越界直接拒绝并给出原因。自动补起飞/降落、按搜索航迹自动放宽任务时限。
- `mission/search_pattern.py`：弓字形覆盖航迹 + 折线长度/时长估算。
- `mission/executor_core.py`：纯逻辑（P 控制限速、前向净空门、围栏、机体前向目标、
  搜索航点展开、单步超时）。
- `mission/task_executor.py`：ROS 执行节点。阶段 IDLE→ARM→OFFBOARD→RUN→LAND→DONE；
  订阅 PX4 状态/地图净空/检测目标，发布 `offboard_bridge` 指令与
  速度/位置目标；2 Hz 心跳；`~/status` JSON；JSONL 报告。
  安全：默认 `dry_run=true`；围栏 + 前向净空门 + 任务总时限 + 状态陈旧 +
  人工接管暂停（非 Offboard 立即停发，交还后恢复）+ 卡阶段超时 → `land_fast`。
- `scripts/fly_task.py`：CLI。默认**只解析并打印计划**；`--execute` 且
  `FLY_TASK_CONFIRM=1` 才执行；`--via-arbiter` 适配开着仲裁的栈。
- `scripts/fake_px4_sim.py` + `scripts/task_executor_simcheck.sh`：**无 AirSim/PX4 的集成自检**。
  假 PX4 发布状态/位置/地图净空，订阅执行器指令并做限速运动学近似；可注入
  "前向净空 0.5 m" 与 "人工接管（切 POSCTL）" 两种场景，按报告里的事件序列判定 PASS/FAIL。
- `docs/DEMO.md`：从起链路到"一句话飞行/搜索"的完整步骤与预期输出。
- `scripts/stop_stack.sh`：补 `perception/victim_detector`、`mission/task_executor`、
  `vlm/vlm_navigator` 的清理。

### 验证

```text
· 单测：mission 44 passed（task_spec 11 / parser+search 22 / executor_core 11）；
        全包 169 passed / 2 skipped（PYTHONNOUSERSITE=1 时 170）
· 解析示例：
    "起飞到 2 米，向北飞 10 米，悬停 5 秒，降落"
      → takeoff(2) → goto(北 10) → hover(5) → land
    "搜索北侧 20 米、半径 10 米的区域，寻找橙色标记，高度 2 米，降落"
      → takeoff(2) → search(center 北 20, r=10, spacing=5, orange) → land
      → 展开 10 个航点、折线 129 m、预计 258 s（时限自动放宽到 387 s）
· 非法指令："今天天气不错" → 拒绝并列出支持的句式
· 执行器 dry_run 冒烟：接受任务、打印计划、写
    logs/tasks/smoke.jsonl（task_accepted 含完整 TaskSpec）
· 假 PX4 集成自检（task_executor_simcheck.sh，4 个场景全部 PASS）：
    ① 简单飞行：takeoff → forward → hover → return_home → land → done
    ② 搜索：8 个航点全部到位 → land → done
    ③ 净空门：注入 nearest_forward_m=0.5 → clearance_blocked → abort → land → done
    ④ 人工接管：切 POSCTL → manual_override → 暂停 → 恢复 OFFBOARD → manual_resume
       → 继续飞 → land → done
```

### 已知问题 / 待办

- **执行器飞验范围**：最小/前进/返航已在 AirSim+PX4 上飞通；**搜索步骤尚未飞验**。
- **AirSim 4560 不可用（2026-10-18 现场）**：UE4Editor 在跑、RPC 45000 正常，
  但 4560 处于 `CloseWait`，Windows 本机也连不上 → 需要 UE 里 Stop → Play 重启场景。
- 规则解析覆盖面有限：复杂/歧义指令会被拒绝（这是预期行为，后续换 VLM parser）。
- 场景内无人物；demo 先用橙色目标物验证"检测→世界坐标→悬停"链路。

---

## [2026-10-18] 收口：结果页/版本基线/check_stack 阈值化 + 三路高度 + perception 包（离线）

### 新增

- `docs/RESULTS.md`：对外可读的**结果与证据**页（能力矩阵、按批次的量化指标、
  安全案例、已知限制、复现入口）；README 顶部加入链接。明确"不跨版本合并 n""不宣称 100%"。
- `docs/VERSIONS.md`：版本基线（PX4 `d0c6c0df7f` / `px4_msgs 598c7aa` /
  Agent `v2.4.2@57d0862` / ROS 2 Humble / AirSim 1.8.1 / LightNav-0 各 md5）。
- `ros2_px4_ws/src/perception/`（新包）：
  - `detectors.py`：`color`（颜色连通域，不依赖 cv2）/ `hog`（行人 SVM）/ `haar`（全身级联）；
  - `geometry.py`：像素+深度 → 光学系 → 机体 FLU → 世界 ENU（与 `frames.py` 同一约定）；
  - `victim_detector.py`：订阅 RGB/深度/CameraInfo + PX4 状态，发布
    `~/detections`（JSON）、`~/tracks`（多帧确认目标）与 `~/best`（PoseStamped）；
    **只观测，不发飞控指令**；
  - `tracking.py`：多帧关联（世界距离 ≤2 m / 无深度时用 bbox IoU）+ N 帧确认（默认 3）
    + TTL 过期（默认 2 s）+ 世界点 EMA；
  - 29 项单测（检测内核 + 几何/深度 + 多帧关联）。
- `scripts/eval_detector.py`：离线评估检测器的检出率/耗时（用已存帧，不依赖 ROS）。
- `scripts/run_detector.sh`：统一启动入口（`PYTHONNOUSERSITE=1`，规避本机 cv2 与 numpy 2.x 冲突）。
- `scripts/analyze_height_sources.py`：对 CSV 的 `px4_up/truth_up/up_offset_m` 做漂移统计
  （均值/标准差/极值/斜率/分阶段），给"高度门用哪路高度"提供数据。
- `scripts/check_stack.sh` 新增 `logs/checks/check_stack_<时间>.log` 报告落盘。
- `bringup/sim_stack.launch.py`：`with_detector:=true` + `detector_backend:=color|hog|haar`。

### 变更

- `scripts/vlm_closed_loop.py`：**三路高度**
  - 新参数 `height_source`（`px4` 默认＝保持原行为 / `airsim`）与 `truth_sample_s`（默认 2 s）；
  - 高度门统一走 `_flight_up()`；簇 z 用 `_to_flight_frame()` 换算，避免两路混用；
  - CSV 新增 `px4_up` / `truth_up` / `up_offset_m` / `height_src`（`record` 与 `record_approach` 列集一致）。
  - AirSim 真值采样按 `truth_sample_s` 节流（LockStep 单次 RPC ~330 ms，不能逐帧调）。
- `scripts/run_detour_trial.sh`：新增 `HEIGHT_SOURCE` / `TRUTH_SAMPLE_S` 环境变量透传。
- `scripts/env.sh`：`ROS_LOG_DIR` 默认指向 `logs/ros`（`~/.ros` 不可写时节点不再启动即失败）。
- `scripts/check_stack.sh`：**频率阈值化**——`n/a`/无数据/低于阈值一律 FAIL 并非零退出，
  不再"话题存在即 OK"（外部评审 P2）。

### 验证

```text
· 单测：vlm 45 + perception 28(+1 skip) + px4_interface 20 + planner 18 + mapping 13
        + mission 5 + px4_offboard 1(+1 skip) = **130 passed / 2 skipped**；
        `PYTHONNOUSERSITE=1` 时 HOG 用例也跑 → **131 passed**
· check_stack：无栈环境下 4 个 /fmu/out 话题全部 FAIL、退出码 1（旧版会打 OK）
· 离线检测（60 张已存帧）：
    color(橙色) 19/60 有检出、mean 57 ms → 稳定命中场景橙色球；
    hog(行人)   0/60、mean 8.9 ms → 场景内确实没有人（不是检测器失效）
· analyze_height_sources.py：合成 CSV 上 slope=+1.200 m/min 与设定一致
· detector 节点冒烟：color/hog 两种 backend 均能启动（hog 在 PYTHONNOUSERSITE=1 下可用）
```

### 已知问题 / 待办

- **场景里没有人物**：人员检测的"能不能找到人"必须先在 AirSim 里放入人物/目标再评估；
  当前只能用橙色球做"目标物"demo。
- 三路高度**只落地未飞验**；`height_source` 默认仍为 `px4`，等 Batch2 数据决定。
- 本机 `~/.local` 的 numpy 2.2.6 与系统 cv2 4.5.4 冲突（不影响飞控栈，影响 CV）；
  统一用 `scripts/run_detector.sh` / `PYTHONNOUSERSITE=1`。
- `main` 分支仍停在初始提交；`dev` 已 116+ commits（本地多 2 个未推），推送需 GitHub 凭据。

---

## [2026-10-17] c1/c2：RUN 窗口已扩（180/150/200）；c1 暴露 EKF 高度漂移影响高度门

### 变更
- `run_detour_trial.sh`：驱动 `run_s 115→180`、VLM `run_duration_s 95→150`；
  `flight_supervisor.py`：RUN 预算 `150→200`（嵌套 180<200<600）。

### 验证
```text
c1（s1017-162235）：交权成功，但 **交权点 ENU 上 +0.1 m**（应在 2 m）；CSV `px4_z` 在 **−3.52 ~ +0.47** 漂移
  → 项目已知的 **EKF 高度估计漂移**（README 4.5），现在也会污染**高度门/世界系 z**
  episode 未闭合（0/1）；飞机安全落地、上锁
c2：无 episode
```

### 待办
- 高度门依赖"世界 z"，而 EKF z 在漂移 → 需要先确认高度门用的是**哪一路高度**（PX4 local z vs AirSim 真值 vs 地图自洽），
  否则会出现"结构高度判错"。建议下一轮先只记录/对比三路高度，再决定高度门的数据源。

---

## [2026-10-17] b6/b7：local_map 崩溃被 fail-closed 拦下；b7 episode 被 RUN 窗口截断

### 事件与验证
```text
· Batch2 b1–b5 全部被 preflight 拦下（local_map 0 实例）——local_map 在 t3/t4 之间因
  cluster_centroid_in_cone 解包异常退出；fail-closed 保证了一次没起飞。
· 重启栈后 b6 无 episode；b7 交权成功（持久 track+60 s 逼近），但
  **episode 仅 5 帧（seq 650–654 / 总 655 帧）就在 RUN 窗口末尾** → 0/50 未闭合 = **窗口截断**，非绕行失败。
  ⇒ 60 s 逼近让交权更晚，`run_s=115 s`（VLM run_duration=95 s）不够留出"保持→绕行→恢复"。
```

### 下一步
- 把 `run_s` 115 → 180、VLM `run_duration_s` 95 → 150（并确认 geofence 与监督层 RUN 预算 150 s 的嵌套），
  给"保持→脱困→恢复"留足时间；随后重跑 Batch2。

---

## [2026-10-17] t3 成功：60 s 逼近预算 + 持久 track → 首次交权并闭合（可绕 1/1）

### 验证（t3 s1017-160???）
```text
扫描→复验通过→对准→逼近，**60 s 预算内到达停止阈值**：
  交权点：ENU (东 +10.1, 北 +22.1, 上 +2.3)、航向 14.9°、前向净空 0.66 m、左 0.46 / 右 inf
  摆位完成：前向净空连续 3 次 ≤0.7（0.66）→ 交权给 VLM
episode 1：135 帧、避让 left、脱困 0 → **最终闭合 15/50（快速 11/20）**
主指标：可绕闭合率 **1/1 = 100%**（Wilson [0.21, 1.00]）
⇒ 「持久 track（不误作废）+ 60 s 逼近预算」组合打通了"发现→复验→逼近→交权→闭合"全链路
t4：未生成 driver.log（预检/启动阶段未通过，待查）
```

---

## [2026-10-17] 持久 track 飞验（t1）：不再误作废；新卡点=逼近太慢 → pre_timeout 35→60 s

### 验证
```text
t1（s1017-155059）：
  扫描发现 2.78 m 候选 → 微转复验通过（世界点偏差 0.61 m）→ 对准 → 逼近
  **全程没有"净空回升→作废"**（持久 track 生效，不再因出锥误判）
  但 35 s 只把净空从 2.78 → 1.42 m（需 ≤0.7）→ **摆位超时** → 不交权、作废、收工
t2（s1017-155548，pre_timeout 已改 60 s）：
  3 个候选点扫描均无可复验障碍（命中率波动）→ 未走到逼近，60 s 预算未受检
```

### 变更
- `run_detour_trial.sh`：`pre_timeout_s` 35 → **60 s**（逼近偏慢；既有 35 s 不足以走完 ~2 m）。
- 持久 track（世界质心 ±2 m / 3 s 丢失才作废；净空回升只重对准）已飞验有效。

### 下一步
- 同点位/自动流程再跑几轮，检验 60 s 预算下能否到达停止阈值并交权；随后开 Batch2（n≥10）。

---

## [2026-10-17] 持久 track：作废只看"track 丢失"；净空回升只重对准（按服务器侧口径）

### 变更
- 驱动新增 **世界系持久 track**（质心世界位置 + 最后可见时刻；±2 m 内视为同一目标）。
- APPROACH：
  - 净空回升**不再作废**；重对准到上限时，若 **track 仍在** → 停下、原地小幅偏航重捕获（不作废）；
  - **只有 track 丢失**（±2 m 内超过 3 s 无簇）才作废该候选点；既有 35 s 逼近超时保留为时间闸。
  - 新候选点时 track 重置；首次 ALIGN 用当前簇质心初始化 track。

### 验证
- 语法通过；**飞验待做**（改动刚完成，下一步跑 1 轮确认"track 不丢→不作废、能继续逼近到停止阈值"）。

---

## [2026-10-17] seek_margin 3.0→1.5 飞验（m1）：发现+复验全通；卡在"逼近中障碍脱离视野"

### 变更
- `run_detour_trial.sh`：`seek_margin_m` 3.0 → **1.5**（候选点更靠近结构；净空硬停仍兜底）。

### 验证（m1 s1017-154045）
```text
SEEK 最优 90° 中位 16.9 m、MAD 0.02 → 3 个候选；前两个点扫描 inf（无簇）；
第三个点（东 +4.4 北 +18.1）：
  高度门：簇 z=[1.3, 6.8] m、跨度 8.6 m、飞行 2.4 m → 顶部净空 −4.44 m → 判"障碍"（正确）
  扫描发现候选：**前向 3.94 m**（首次进入 4.0 m 门）
  微转复验 **通过**：世界点偏差 0.65 m、距离差 0.10 m（簇 6347→9231 体素）
  对准完成（误差 +0.4°、宽侧 inf）→ 0.15 m/s 逼近
  → **净空回升到 4.34 m（障碍脱离视野）** → 重对准 3 次 → 候选作废 → 收工
```

### 结论/下一步
- 高度门 + 更近候选点让"发现候选→复验通过"这条线**首次全通**；
- 剩余瓶颈：**逼近阶段障碍脱离视野**（净空回升）→ 候选作废。
  下一步方向：逼近时不要只依赖前向锥（用体素簇质心方位持续对准），或把"净空回升"判据与"簇是否仍在图内"联动；
  也可考虑在候选点直接用体素簇质心方位做段内对准（不依赖扫描瞬间的航向）。

---

## [2026-10-17] 高度门 v2 飞验（v1 轮）：判定链生效（体素簇 z → 顶部净空 → 障碍/可飞越）

### 验证（v1 s1017-153614）

```text
候选点高度门实测：簇 z=[1.3, 7.0] m、水平跨度 8.7 m、飞行高度 2.3 m
  顶部净空 = 2.3 − 7.0 = **−4.62 m**（阈值 0.50）→ 判"障碍"（7 m 高结构，应当绕）
另两个候选：local_map 里没有簇（cluster_z 缺失）→ 走 SCAN 但前向 inf
本轮仍无 episode（高结构候选点不在 4 m 扫描门内 / 低矮样本未出现）
```

### 结论

- 高度门（voxel cluster z + `overfly_clearance_m=0.5` 负余量、候选点评估）**判定链已验证**；
- 剩余问题是"候选点离结构 >4 m"（SEEK 的 median−3 放置），与高度门正交；
  下一步可在候选点前移（seek_margin 3→1.5）或把 SCAN 门限与"结构在体素图里的距离"联动。

---

## [2026-10-17] 高度门 v2：改用**体素簇 z 范围**+ 修正符号（overfly_clearance）；在候选点评估

### 变更（按服务器侧 2026-10-17 两点修正）

- `voxel_grid.cluster_centroid_in_cone` 额外返回簇的 **min_z / max_z / 水平跨度**；
  `local_map` 状态串新增 `cluster_z=min,max span_h=…`（世界 ENU，体素实测）。
- 驱动在**候选点**（DESCEND 完成、结构已进体素图）做高度门：
  `overflyable ⇔ flight_altitude − top_z ≥ overfly_clearance_m`（默认 **0.5 m**）。
  2 m 飞行 ⇒ top≤1.5 m 才可飞越；**top>1.5 m（含 1.9 m 墙）都算障碍**。
  低矮则记 `overflyable` 计数（单独统计）、换下一个候选；够高才进 SCAN/逼近。
- 不再使用"深度图仰角"估高（h1/h2 实测 33/28 m，被地平线/天空污染）。

### 验证

- 语法/构建通过；`colcon test` **65 passed**。
- **飞验待做**：本次未飞行（改动刚完成）。

---

## [2026-10-17] 高度估计器改进（距离带 + 90 分位）仍不可用：h2 top_z 28 m → 需改用体素簇 z 范围

### 变更
- SEEK 高度估计从"整扇区 max(up)"改为"**中位距离 ±max(1.5·MAD,1 m) 内点的 90 分位**"。

### 验证（h2 s1017-152436）
```text
扇区 90°：中位 16.2 m、MAD 0.02、顶部z **28.4 m**、底部z 0.1 m
扇区 120°：顶部 28.4；扇区 60°：顶部 28.2
⇒ 顶部仍高得离谱：地平线/天空点与中位距离相近（深度≈中位数），但 up 值巨大，
  单靠"距离带"无法把它们与结构分开。
```

### 下一步（更本质的做法）
- **用 local_map 的体素簇**：它已在世界 ENU 里建好 3D 体素（含 z），对"锥内最近簇"直接取
  **簇的 min/max z 作为底部/顶部**——这才是真实的 3D 结构高度；深度图的 (方位, 仰角) 上再聚类也可，
  但复用体素簇更省。
- 实现 `cluster_centroid_in_cone` 的扩展（返回 top_z/bottom_z），`local_map` 状态串加字段，
  SEEK 的高度门改用它。

---

## [2026-10-17] 高度门（3D 障碍定义）首版 + 首轮数据暴露"高度估计器被污染"

### 变更（按服务器侧 2026-10-17 的核心判断）

- **地面策略搬到空中必须补 3D 高度语义**：2 m 飞行时 0.6 m 低矮物可直接飞越，不该算障碍。
  `vlm_closed_loop.py` 的 SEEK 新增 **高度门** `seek_top_margin_m`（默认 0.0）：
  候选扇区要求 `顶部z ≥ 飞行高度 + 余量`；统计表新增 **顶部z / 底部z** 两列。
- `scan_found` 已按要求**回退 6.0 → 4.0**（a1 证明 6 m 内仍全 inf，放宽无效）。

### 验证（h1 s1017-151858）

```text
高度门首轮数据（扇区, 中位m, 点数, 跨度°, MAD, 顶部z, 底部z, 判定）：
  90°  19.1 1889 36.0 0.06 **33.1** -2.9 OK
  120° 20.9 1940 35.6 0.37 **33.1** -2.5 OK
  60°  23.5 1576 33.0 0.55 **31.1** -1.9 OK
⇒ 顶部 z 高达 31–33 m，明显被**远处/高处地形（或天空）点**污染：
  当前用"整扇区 max(up)"，不是"最近簇的顶部"。
```

### 已知问题 / 下一步

- 高度估计器需改为"**该扇区最近簇（或距离分位内点）的顶部 z**"，而不是整扇区 max；
  否则高度门形同虚设（所有扇区都会被判"很高"）。
- h1 仍无 episode；本批（受时间限制）未再跑。

---

## [2026-10-17] 标定轮 cal1 + 新点位 k30：确认"扫描门限 4 m vs 实测 5.72 m"是 A 类主因

### 验证

```text
cal1（s1017-150235，正常 SEEK）：
  最优扇区 90°、中位 19.1 m、垂跨度 35.4°、MAD 0.05 → 高结构（大而平）
  候选点 (局部 东 -4.2, 北 15.6) 到达后，扫描诊断：**最小前向净空 5.72 m（航向 117.9°）**
  ⇒ 结构在候选点外 5.72 m，而 `scan_found=4.0 m` → **差 1.7 m，扫描判"未发现候选"**
k30（s1017-150912，按反算把已知点挪到"结构前 ~2 m" 世界 -5.7,18.5）：
  转场在距目标 2.8 m 处被另一障碍挡住（前方 0.7–0.9 m）→ 45 s 超时 → 标记不可达 → 收工
  ⇒ 反算点**直达路径被挡**，不能直接用
```

### 待定方向（需用户/服务器侧拍板）
- **A（我倾向）**：把 `scan_found` 4.0 → **6.0 m**（纯"发现门限"，非安全参数），让 SEEK 候选点
  （实测结构在 5.72 m）能被发现，随后由 0.15 m/s 逼近（35 s 预算）走完最后 ~2 m；
- **B**：改点位/转场线（避开中间障碍），代价是继续人工标定；
- **C**：降低扫描高度（2.0 → 1.2 m），修"低矮结构被垂直窗口裁掉"那一类（与 A 正交）。

---

## [2026-10-17] 命中率归因定位到根因：**2 m 扫描高度 × 垂直窗口**裁掉了 0.6 m 低矮结构

### 验证（k29 诊断轮，s1017-145922）

```text
现象：飞到已知点（世界 东4.6/北13.0）后整圈扫描**前向净空恒 inf** → "未发现候选"
落地后同点再看地图：nearest_forward=0.53 m、nearest_centroid=1.11 m（59 体素）⇒ 附近确有物体
该物体质心相对 z=-0.11 m（飞机在地面 z=0.7）⇒ 高度约 **0.6 m**
扫描高度 2.0 m；local_map 垂直窗口 上+0.6/下-0.25 m ⇒ 2 m 处只能看到 ≥1.75 m 的点
⇒ 0.6 m 的低矮结构被垂直窗口裁掉 ⇒ 整圈 inf ⇒ A 类"未发现候选"（10-16 归因的 13 轮主导项）
```

### 待定修法（下一轮三选一）
- (a) 降低扫描/摆位高度（如 2.0 → 1.2 m）——最直接，但贴近地面飞行风险略增；
- (b) 放宽 `cone_down_tolerance_m`（0.25 → 0.5~0.8 m）——仍远低于 2 m 地面距离，但属传感器窗口变更；
- (c) 换已知点位到**高结构**（SEEK 常报的 15–22 m 那栋建筑）——不改参数，改点位。

---

## [2026-10-17] 离线命中率诊断器 + 扫描点诊断日志（定位"到了点位发现不了候选"）

### 新增

- `scripts/diagnose_hitrate.py`：**不需要飞行**的命中率归因工具——从驱动日志把每轮归为
  `A 未发现候选 / B 复验被拒 / C 摆位未到 / D 已交权 / E 交权后无保持`，并列出候选数、复验通过/拒、
  交权点（ENU/航向）与最后转场到位。
- `vlm_closed_loop.py`：每个扫描点结束时打印**扫描诊断**（本点最小前向净空 + 对应航向(ENU) +
  当时左右净空 + 两个门限），供下一轮直接判断"点位太远 / 侧向门限 / 扫描-地图刷新"。

### 验证（离线，使用 2026-10-16 全部 31 轮日志）

```text
A 未发现候选 = **13 轮**（主导）   C 摆位未到 = 1
D 已交权 = 12                      E 交权后无保持 = 5
⇒ 命中率低的主因是"飞到点位后整圈扫描都找不到 前向≤4 m 且一侧≥1.5 m 的扇区"，
  不是复验/绕行问题（k24 典型：转场到位后"全部候选点扫描完毕仍未找到可复验障碍"）。
```

---

## [2026-10-16] Batch1（5 轮）+ Wilson CI + 交权点全轮次落盘 + 安全案例集；本轮结束关停链路

### 新增

- `analyze_detour.py`：主指标行增加 **Wilson 95% 置信区间**（小样本/接近 100% 时比正态近似准），
  与原始计数一起打印（服务器侧 2026-10-16 要求；不单报"100%"）。
- `vlm_closed_loop.py`：交权完成时新增 **`交权点记录：ENU 东/北/上、航向(ENU)、前向净空、左右净空`**，
  供"有/无 episode 轮次"同口径归因命中率。
- `docs/safety_casebook.md`：**安全分层案例集**（5 例：位置点转场撞墙 / 传感器盲区仍推进 /
  转角封顶硬推 / 持续不可绕的最外层兜底 / 落地后仍 running），每例含现象-根因-修法-证据与通则。

### 验证

```text
Batch1（同点位不变，5 轮）：k24–k27 无 episode、k28 1/1 闭合
修复后累计：**可绕闭合率 8/8 = 100%，Wilson 95% CI [0.68, 1.00]**（副：快速闭合 7/8、命中 7/13）
n=8 未达 10 → Batch2 留待下次（本轮结束用户要离开，已按规程 `scripts/dev_chain.sh stop` 全停）
```

---

## [2026-10-16] 最外层兜底时间线首次走通（sup2）；新增 `escape_hold` 注入场景

### 新增

- `inject_clearance.py` 新增场景 **`escape_hold`**（900 s 持续 `l=r=0.4`、**永不解除**）：
  用于验证"持续不可绕"下的最外层兜底。
- `run_escape_trial.sh` 支持 `RUN_S` 覆盖驱动运行时长（测监督层接手时设大，避免驱动先收工）。

### 验证（sup2 s1016-200910）

```text
完整时间线（每步有日志）：
  保持 >20 s → 第 1 次脱困（0.92 m/9.2 s 有界）→ 结束后退、恢复保持在位判断
  再保持 >20 s → 第 2 次脱困（0.91 m/9.1 s 有界）
  → 脱困重试用尽 → **不再后退**（ERROR 级明确放弃）
  → 驱动保持悬停（前向恒 0，不硬推）
  → **监督层：阶段 RUN 已持续 151 s（>150 s）→ land_fast**
  → 安全落地（arming_state=1）
⇒ 安全链最外环（机载 → 驱动 → 监督层 → 全局）在"持续不可绕"场景下首次完整走通
```

### 已知问题

- 服务器侧指出 **n=5 支撑不起"100%"**（95% 置信上界仍 ~45%）→ 需把**可绕 episode 做到 n≥10**
  再冻结，并报**原始计数 + 置信区间**；该批次约 10 轮（~40 分钟），已排队待跑。

---

## [2026-10-16] 冻结口径：修复后 10 轮 **可绕闭合 5/5 = 100%**；"转角封顶不推"已证；"不可绕"样本已造

### 验证（三个待办一次做完）

- **① 修复后版本单独跑 5 轮（k19–k23）** + k14–k18 → **修复后 10 轮冻结口径**：

```text
主指标 可绕闭合率 = **5/5 = 100%**（PASS）；副指标 快速闭合(20 帧) = 4/5
不可绕 episode = 0；命中率(副) 5/10 = 50%（与修复前一致 ⇒ 命中率不受此修复影响）
（修复前 k7–k13 的 8/9 按服务器侧要求只作历史对照，不参与冻结）
```

- **② e2 情形已消失**：修复后 `yaw_capped=True` 帧共 **193** 个，其中**前向>0 的 = 0**
  （唯一非零是 `obstacle_escape` 的 −0.1 m/s 脱困后退，属设计行为）。
- **③ "不可绕"样本已造**（注入 `l=r=0.4`）：`esc5` episode 167 帧，**判为不可绕**
  （保持段侧向中位净空 0.40 m）；行为 = 全程不硬推（前向恒 0）→ 两次有界脱困（0.92/0.91 m）
  → 重试用尽"不再后退"交监督层 ⇒ **不可绕分支的安全性得到验证**。

### 修复（打分器分类口径）

- `analyze_detour.py`：①"可绕"只看**保持帧**（`guard=obstacle`）的侧向净空（episode 末尾常含
  "已放开但释放确认没走完"的帧）；②取这些帧的**中位数**而非最大值。
  不修这两条，`esc5` 这类"不可绕"样本会被误判成"可绕"。

---

## [2026-10-16] 修"转角封顶仍前推"缺陷；打分器加"可绕/不可绕"分类；已知点位 12 轮：可绕闭合 **8/9**

### 修复（安全）

- `safety.apply_map_safety`：`yaw_capped=True`（累积转角达 `max_turn_deg`）时**前向也归零**
  （原来只把偏航置 0，creep 仍以 0.05 m/s 前推）。缘由：k7-e2 逐帧显示
  `turn_deg 64–69° / yaw_capped 16/18 帧` 时飞机"既不再转、又继续推"→ 贴死在结构上
  （而它**右侧是 inf**，按口径属"可绕"，本应绕开）。修后语义：**转不动就不推**，
  交给脱困/预算/超时兜底。单测 +1 → **65 passed**。

### 变更（口径，服务器侧 2026-10-16 建议）

- `analyze_detour.py` 增第三类：**可绕 episode**（episode 期间至少一侧净空 ≥ 2.0 m）
  与**不可绕 episode**（两侧均紧/盲区持续，单列、不进口径）。
  主指标改为 **可绕闭合率**（最终闭合 50 帧窗口），副指标仍是快速闭合（20 帧）+ 命中率。

### 验证（同点位，k7–k18）

```text
k14 0 episode；k15 1/1（**仅 50 帧窗口才判闭合**：20/50 帧、快速窗口仅 2/20）；k16 1/1；k17/k18 0
  ⇒ 修复后 k14–k18：可绕闭合 2/2 = 100%
k7–k18 全批：**可绕闭合率 8/9 = 89%**；快速闭合 7/9；不可绕 0
  命中率（副）6/12 = 50%；无 episode 6/12
  含脱困段的闭合 episode 6 个以上 ⇒ 五项安全改动在真实闭合上全部通过
```

### 已知问题

- k7–k13 为修复前版本、k14–k18 为修复后版本，两组合并统计口径偏松；
  已提议在修复后版本上再跑一批，给出**版本一致**的口径。

---

## [2026-10-16] 打分器改"主=最终闭合(50 帧)/子=快速闭合(20 帧)"；同点位 7 轮：闭合 **6/7**

### 变更

- `scripts/analyze_detour.py`：按服务器侧 2026-10-16 的"指标口径 vs 时间预算"意见改为**双窗**：
  **主判据 = 最终闭合**（episode 结束后 **50 帧 ≈23 s** 内 ≥5 帧前向非零；
  50 = `hold_max_s` 20 s≈44 帧 + `escape_max_s` 10 s≈22 帧的余量），
  **子判据 = 快速闭合**（原 20 帧口径），两者都打印。
  动机：原 20 帧(≈9 s)窗口**短于**"保持+脱困"的时间预算，会把"合法但较慢"的绕行误判为未闭合。

### 验证（同点位 k7–k13，参数不变）

```text
k7  3 episode / 2 闭合       k8  0 episode
k9  0 episode                k10 1 episode / 1 闭合
k11 1 episode / 1 闭合       k12 0 episode
k13 2 episode / 2 闭合
原始计数：**episode 7 个 / 闭合 6 个 = 6/7（86%）**；命中率（副）4/7 = 57%；无 episode 3/7
其中 **5 个闭合 episode 含脱困段（7–15 帧）** ⇒ 「脱困有界 + 释放连续确认 + 盲区底线 + creep 双闸」
在真实闭合上全部通过；每轮仲裁 winner 序列仍为 none(摆位) → vlm(交权后)
```

### 已知问题

- 用新口径复核 k7 的 episode 2：**最终 0/50、快速 0/20** → 它**不是窗口截断**，是"贴大结构
  （簇 3000–4000 体素）、盲区禁止前推、creep 预算近用尽、脱困未触发"导致的一次真未闭合。
- 命中率波动大（同点位 0/1/0/1/2 个 episode）；已请服务器侧定"是否再跑 5 轮把 n 做到 ~12"。

---

## [2026-10-16] ② 仲裁接进主线（winner 落盘）+ 已知点位回归（k7 2/3 闭合）；整簇质心；点位改世界坐标

### 新增/变更

- **② 仲裁接进主线**：试验里 `vlm_navigator` 的速度改发 `/velocity_arbiter/vlm_cmd`，
  由仲裁器（20 Hz，`avoid>vlm`）转发到 bridge；**驱动（SEEK/TRANSIT/DESCEND）仍直连 bridge**——
  这正是服务器侧认可的数据流。试验脚本每轮自带一个**写 CSV 的仲裁器**（`*_arbiter.csv`），
  并在打分后打印 winner 序列；`NO_ARBITER=1` 可退回直连对照。
- **已知结构点位模式**（走 C 的闭合回归）：`KNOWN_POINTS="东,北;…"`（**AirSim 世界系**，
  环境变量直传，绕开 `-p` 传含分号字符串的解析坑）→ 起飞后先用 start_pose 飞到世界原点、
  记下"世界原点↔局部 ENU"偏移，再把世界点位换算成本机局部目标，逐个飞过去扫描。
- **整簇质心**（k4 诊断结论）：`cluster_centroid_in_cone` 改为**在整张局部地图上聚类**、
  质心取**整簇**（此前只对"锥内体素"聚类，一偏航参与子集就变：337→457 体素、世界点漂 2.92 m）。
- `flight_supervisor.py`：TRANSIT 预算 90 → **150 s**（驱动"每段 45 s + 受阻换点"会被按同名阶段累计，
  90 s 会误杀；嵌套仍成立：45 s < 150 s < 600 s）。

### 验证（原始计数）

```text
k7 s1016-184559：**3 个合格 episode / 2 个闭合（2/3）**，命中率 1/1
  ep1 63 帧(0.63 m, 脱困 14 帧) ✅ ；ep2 21 帧(0.58 m, 脱困 0) ❌ ；ep3 60 帧(0.50 m, 脱困 12 帧) ✅
  ⇒ 「脱困有界 + 释放连续确认 + 盲区底线」在**真实闭合**上成立（含 14/12 帧脱困段）
k8 s1016-185022：0 episode（2 个障碍区间无保持态）——同点位变异性仍大
两轮的仲裁 winner 序列均为 `none`（摆位/转场=驱动直连）→ `vlm`（交权后经仲裁），
与服务器侧要求的数据流一致
```

### 已知问题

- **点位标定已纠正**：历史 d3/d5/m4 的"交权点"其实是**转场目标点**（交权还要再前进 ~1.7 m），
  用现在的净空硬停飞不进去（k5/k6 被真障碍挡在 3.5–3.8 m 外）。现改为"结构前 ~2 m"的
  世界坐标点位（`4.6,13.0`），k7 才复现出 episode。
- 同点位变异性大（2/3 vs 0）；已提议同点位连跑 5 轮把 n 做够再谈比率。

---

## [2026-10-15] 新增 `scripts/dev_chain.sh`（一键起停开发链路）；按服务器侧结论**改走 C**

### 新增

- `scripts/dev_chain.sh start|status|stop`：开发链路一键起停（隧道 → Agent → PX4（含电池阈值清零）
  → ROS 栈 → 监督层，并逐项自检）。用途：用户每次暂离会**彻底关闭 WSL**，回来后需要一次性拉起。
  `start` 的自检**不含 VLM 节点**（它只在试验期间存在）——真正的飞行门禁仍是 `preflight.sh`。
  README 4.6 与 `docs/next-session-handoff.md` 都写明了 `stop`/`start` 的暂离-恢复流程。

### 变更（方向，来自服务器侧 2026-10-15 深夜来信，已收到）

- **走 C**：主线转为 ②（VLM × local_planner 仲裁的实飞联调）+ 回归；
  **绕行闭合率统计改在"已知有结构的点位"上做**（摆位后交权，像 m4），使 episode 可复现、样本可做大；
  自动寻障**命中率降为副指标**（照旧记录，不作主线门禁）。
- 转场限速 **0.5 → 0.8 暂不动**（切到已知点位后转场很短）；将来做长距离转场时再上，
  并要求 `transit_stop_clearance_m ≥ 制动距离`。
- 将来攻命中率的优先方向（服务器侧建议）：到达后**小幅偏航重新捕获 + 按簇质心方位对准 +
  转场点放更近（结构距离 ×0.6）**，而不是只调 `scan_found`。

### 验证

- `scripts/dev_chain.sh stop` 已执行：WSL 侧 **0 残留进程**（栈/监督层/VLM/驱动/PX4/Agent/隧道全停）。
- 特别提示：最近的"释放连续 3 帧确认 / 盲区底线 / creep 双闸"**尚未在真障碍的
  "保持→释放→恢复"闭合上验证过**（c1/d3/d5 都在这些改动之前）——已知点位回归正是验证它们的载体。

---

## [2026-10-15] 回波判别有界 + 转场受阻改"标记不可达→换点"；确立"超时必须严格嵌套"通则（e1/e2 飞验）

### 变更（按服务器侧 2026-10-15 三条）

- **回波判别有界**：`transit_echo_max_probes=2`、`transit_echo_max_back_total_m=0.5`
  （单次转场最多判别 2 次、累计后退 ≤0.5 m），用尽后**按真障碍处理、不再后退**；
  每段转场独立计数（`enter('TRANSIT')` 时清零）。
- **转场受阻不绕行**：新增 `_give_up_candidate()`——硬停 → 超时 → 该候选点标记**不可达** → 转下一个；
  用尽 → 收工降落（与"无确认不交权"同一哲学；绕行职责属 local_planner + 仲裁器）。
  `TRANSIT` 超时由"就地 SCAN"改为"换下一个候选"。
- **通则（写进文档/交接清单）**：**超时必须严格嵌套，内层 < 外层**
  （机载 < 驱动 < 监督层 < 全局）。本次即：驱动 TRANSIT 45 s < 监督层 90 s；
  驱动 SCAN 75 s（3 点×25 s）< 监督层 120 s < `max_flight_s` 600 s。

### 验证

- 按服务器侧要求先跑 **0.5 m/s 干净批次**（e1/e2）：

```text
e1 起始位姿转场遇 0.61 m → 回波判别判"真障碍"（判决正确）；SEEK 合格扇区 7（最优 90° 17.2 m）
   → 转场到位（东 -3.4 北 +13.3）→ DESCEND → SCAN
e2 SEEK 合格扇区 5（最优 90° 18.3 m）→ 两次转场均到位（东 -4.0 北 +14.3 / 东 -11.8 北 +12.4）
两轮：转场全到位、无死锁、脚本正常退出、安全落地；仍无 episode（命中率 0/2）
```

- 单测 64 passed。

### 已知问题

- **命中率仍是主瓶颈**：SEEK 能筛出 4–9 个合格扇区，但到达候选点后 SCAN 很少复验出"可保持障碍"。
  已请服务器侧定：限速 0.5→0.8 的时机；是否切 C（主线转 ②+回归）或先做"已知结构点位"可闭合专项。

---

## [2026-10-15] 转场改速度控制（真正限速）+ 修"落地后不退出"死循环 + 三处参数对齐（d13–d19 飞验）

### 修复（安全/健壮性）

- **转场从"位置目标点/位置追踪点"改成速度控制**（`vlm_closed_loop.py` 新增 `publish_velocity_enu`）：
  位置方式交给 PX4 会被它按自身巡航速度执行（`MPC_XY_CRUISE≈3`、上限 12 m/s）——这是"高速撞障碍"的根因；
  改成"每周期推一点点位置"后实测只有 **~0.11 m/s**（98 s 走 11 m，20 m 外的点到不了）。
  现在速度由我们直接给（`transit_speed_mps`，精确限幅）+ 方位对齐（`transit_yaw_rate`）
  + 前方净空硬停 + 贴地板读数的自机体回波判别。
- **驱动新增安全闸**：一旦"曾经解锁 → 现在上锁"立即收工退出。
  缘由：监督层按阶段预算下发 `land_fast` 后，驱动原来自顾自继续跑阶段机（还会 SCAN↔ROTVERIFY 死循环），
  表现为"飞机早落地、终端一直 running"。
- **SCAN 计时不再被复验失败重置**：改为绑定"本点扫描起点"，跳点/换点才重置——
  否则"发现候选→复验失败→回 SCAN"会无限循环、`scan_max_s` 永不到期。
- `run_detour_trial.sh`：驱动加 `timeout 900` 硬超时兜底；`transit_timeout_s` 120 → **45 s**
  （让驱动先于监督层 90 s 预算处理"受阻转场"）。
- `flight_supervisor.py`：`phase_budgets` 里 **SCAN 35 → 120 s**（驱动"每点 25 s × 3 点"本身 ~75 s，
  35 s 会误杀正常扫描；单点 25 s 硬上限仍在，故 120 s 仍是卡死兜底）。

### 验证（全部安全落地）

```text
d13 被自机体回波卡死（硬停用了原始净空）→ 加"后退 0.25 m 判别"后恢复
d14 起始位姿转场正常，但**跳过 SEEK**（已修：DESCEND 后回到深度寻障）
d15/d16 SEEK 正常（合格扇区 4–9 个）；转场受限于旧控制方式
d17 落地后驱动不退出（已修）：日志出现 `ERR 检测到飞机已上锁（当前阶段 TRANSIT）→ 驱动收工退出`，
    脚本正常退出、无残留进程
d18 速度控制转场把飞机送到 19 m 外候选点（"转场到位 东 -4.9 北 +18.5"）；下一段遇真障碍 0.75–0.96 m → 硬停
d19 **三次候选转场全部到位**（+12.0 / +10.3 / +16.8 m）→ 扫描完 3 点未找到可复验障碍 → 按预算收工
```

### 已知问题

- 命中率仍低（d18/d19 无 episode）：SEEK 能筛出 4–9 个合格大结构，但到达后 SCAN 很少能复验出可保持障碍。
- 转场遇真障碍目前只"停住→超时→就地扫描"，**不做绕行**（已请服务器侧定是否要加）。

---

## [2026-10-15] ⚠️ "高速撞障碍"真凶：**位置目标点式转场无限速**（已修）；盲区覆盖所有前向路径 + creep 双闸 + 固定起始位姿

### 修复（安全，最高优先级）

- **转场限速 + 前方净空硬停**（`vlm_closed_loop.py`）：用户现场观察到"很多次以高速撞上障碍物"，
  查数据确认**与 creep 无关**——转场以前直接发**最终位置点**，PX4 用自己的巡航速度执行
  （`MPC_XY_CRUISE≈3 m/s`、上限 12 m/s），20–25 m 外目标点中间隔着建筑就是高速撞击。
  证据（d9 CSV）：t=25.8→26.3 s 局部坐标跳 36 m（位置阶跃签名）。
  现改为**限速追踪点**：每周期只推进 `min(transit_speed_mps×dt, 剩余)`，且
  **前方净空 ≤ `transit_stop_clearance_m` 且距目标还远 → 就地停住**（不再盲飞；超时则放弃转场）。
  新参数：`transit_speed_mps=1.0`、`transit_stop_clearance_m=1.0`。

### 变更（按服务器侧 2026-10-15 三条要求）

- **盲区覆盖所有前向路径**：保持态 creep、压速带前向下限、VLM 自身 forward（经缩放）在
  净空 ≤`obstacle_blind_zone_m`(0.5) 时**一律归零**（保留偏航）。
- **区分"测到触底"与"真值"**：`local_map` 状态串新增 `fwd_floor_m=`（地板值）；
  导航侧解析后落盘 `guard_detail['at_min_range']` 与 `map_floor_m`，并把触底视同盲区。
- **creep 双闸**：①`obstacle_creep_open_side_m` 默认 **1.5 → 2.0**（侧向确有空间才蹭）；
  ②新增 `obstacle_creep_budget_m=0.5` / `obstacle_creep_budget_s=10.0`——单次保持内累计超预算
  就**只转不推**，落盘 `creep_used_m` / `creep_budget_ok`，障碍解除即清零。
- **固定起始世界位姿**（不调 `reset()`）：新增 `start_pose_enabled` / `start_world_north_m` /
  `start_world_east_m`——起飞后用 AirSim 只读位姿算差值，复用**限速转场**飞到固定世界位姿，
  使各轮起点一致、命中率可横向比较。

### 验证

- 单测 **64 passed**、`colcon test` 全绿。
- **以上四项改动均未飞验**（服务器侧服务与监控已按要求停止，`preflight` fail-closed，未起飞）。

### 已知问题

- 高速撞击**此前已经发生过**（用户现场观察）；AirSim 只保留最后一次碰撞记录
  （当前为 `Ground_5`，落地触地），历史撞击无法追溯。这是本项目迄今最严重的飞行安全问题，
  已按"先停飞、再定位、再加固"处理。

---

## [2026-10-15] ⚠️ 安全修正：**盲区底线**（净空触底时不再前推）；并更正"重启 PX4 能复位飞机"的错误假设

### 修复（安全）

- **盲区底线** `safety.MapSafetyLimits.blind_zone_m`（默认 **0.5 m**）/ `vlm_navigator`
  参数 **`obstacle_blind_zone_m`**：`local_map` 的 `min_range` 过滤（本机 0.45 m）会把更近的点当
  自机体回波丢掉，**净空读数在 0.45 m 处触底——"看不见"≠"没有障碍"**。现在只要净空 ≤ 0.5 m：
  - 保持态**不再 creep**（前向强制 0，**保留偏航**，继续朝宽侧转把视野打开）；
  - 压速带里同样归零（含刚被"前向下限"抬起的值）；
  - 落盘 `guard_detail['blind_zone']`。
  动机：2026-10-15 用户观察到测试中贴近障碍；数据核查显示净空 ≤0.5 m 且"仍在前进"的帧里，
  真正下发给飞机的分别是**保持态 creep 30 帧**、新下限 1 帧（其余 1887 帧是 `dry_run`/`idle`
  期间 VLM 的"打算发"指令，飞机当时由摆位脚本控制、并未前进）——**根因是既有的 creep 会推进盲区**。
- 单测 +2（`test_blind_zone_zeroes_forward_but_keeps_yaw` 等）→ **64 passed**；
  既有的"保持态 creep"用例改用 0.55 m（> 盲区）以免与新规则冲突。

### 更正（重要）

- **"重启 PX4 会把 AirSim 复位到出生点"是错的。** 实测：AirSim 侧飞机世界坐标
  `(北 23.4, 东 -22.6, 下 0.68)` 仍在原处，而 PX4 报 `x≈0,y≈0`——
  **PX4 重启只是把本地坐标原点重新设在飞机当前位置**，飞机在世界里并没有被搬回去。
  因此本日 d1–d9 各轮的"起点"并不可比（飞机是接着上一轮落点起飞的），
  这也是命中率忽高忽低的原因之一。`scripts/restart_px4_sitl.sh` 仍有用（复位**估计器/原点**、
  清电池阈值），但**不**用于复位世界位置。
- 可比性做法（待用户/服务器确认）：每批之前重启 AirSim 场景（世界回到出生点），
  或用只读方式记录每轮**AirSim 世界位姿**用于归因；不在本项目里调用 `reset()`（会打断 lockstep）。

### 验证

- 单测 64 passed；`colcon test` 全绿。
- **盲区底线尚未飞验**（需要先解决可比性与安全确认）；建议用既有"注入净空"的 esc 式飞行
  （2 m 悬停 + 注入 0.45 m）验证"前向归零、偏航保留"，不靠近真实结构。

### 已知问题

- AirSim 报告的最后一次碰撞是 `Ground_5`（地面、穿透 0.0）——更像 `land_fast` 落地触地；
  **但 AirSim 只保留最后一次碰撞**，此前若蹭到建筑会被覆盖，无法追溯。

---

## [2026-10-15] "进出双门限"：释放也需连续确认（修 d4 的保持↔释放抖动）；d3–d7 统计

### 新增

- `safety.MapSafetyLimits.release_confirm_frames`（默认 **3**）/ `vlm_navigator`
  参数 **`obstacle_release_confirm_frames`**：**解除保持同样要连续 N 帧满足**
  （`forward ≥ release_m` 且所选侧 `≥ side_release_m`），中途回到保持条件即清零（不许累计）。
  这是服务器侧 2026-10-13 "进出双门限"建议的落地；释放延后只让飞机**多保持一会儿**，更保守。
  触发缘由：d4 实测"保持↔释放"来回切换（净空 0.45–0.58 m、每段仅 14–15 帧），绕行做不完。
- 单测 +2：`test_hysteresis_keeps_holding_until_release_threshold` 改为按连续帧记账；
  新增 `test_release_confirm_resets_on_interruption` → **62 passed**。

### 验证

- **d3–d7 五轮（全部安全落地）**：

```text
d3 s1015-130823  ✅ 闭合（保持 7 帧 avoid_side=left → 无脱困 → 恢复 200 帧）
d4 s1015-131406  ❌ 3 个 episode 只闭合 1 个（保持↔释放抖动，见上）
d5 s1015-131904  ✅ 闭合（episode 19 帧，恢复窗口前向非零 10 帧）
d6 s1015-132557  无 episode（未交权）
d7 s1015-132907  无 episode（未交权）

双指标：主指标 绕行闭合率 3/5 = 60%；副指标 寻障命中率 3/5 = 60%；无 episode 2/5
对照：引入 ①(a)+②(a)+压速下限之前，命中率累计 1/10
```

- 压速带前向下限（`obstacle_slow_min_forward_mps=0.05`）在 d3 生效 **57 帧**；
  d3–d5 三轮**全部**交权并进入保持。

### 已知问题

- **释放确认尚未在飞行中验证到**（d6/d7 无交权，没有保持样本）；待拿到 episode 再确认。
- 命中率波动仍大（d2/d6/d7 无交权），样本需做厚；已请服务器侧定优先级。

---

## [2026-10-15] ①(a)+②(a) 飞验证：**首次自动链路闭合 episode**；新增压速带前向下限

### 新增

- `vlm_navigator` / `safety`：新增参数 **`obstacle_slow_min_forward_mps`**（默认 **0 = 关闭**；
  试验里开 **0.05**）——压速带（`stop_m`~`slow_start_m`）里的**前向下限**。
  语义：**只放大"本来就想前进"的帧**（`command.forward > 0`），**不覆盖 VLM 的显式停止**，
  **不改变 `obstacle_stop_m`=0.6 m 安全阈值**；量级与保持态已有的 `obstacle_creep_mps`(0.05) 一致。
  触发缘由见下"已知问题"。
- `run_detour_trial.sh`：`pre_stop_clearance_m` 0.8 → **0.7**（服务器侧预先授权：
  "若仍只到 ~0.75 m 就收到 0.7，仍不动 0.6 安全阈值"）；显式开 `obstacle_slow_min_forward_mps:=0.05`。

### 验证

环境：AirSim 45000/4560 正常；隧道 `/health` ready（GPU 基线 11578 MiB）；Agent + PX4（连 4560）
+ ROS 栈（含 `velocity_arbiter`）+ 监督层各 1；电池阈值已清零；`/fmu/out` ~100 Hz；preflight 全过。

- **d1（s1015-125711）**：微转复验第 3 次通过时世界点偏差 **1.16 m** —— 旧容差 1.0 m 会否掉，
  证明 ②(a) 到 1.5 m 是**必要**改动。摆位在 0.75 m 交权（①(a)=0.8 生效）。
  交权后净空在 0.61–0.96 m 徘徊（最小 0.58 m 单帧）→ **未进保持**。
- **d2（s1015-130241）**：无交权（两次复验均真被拒：候选 1 世界点漂 1.62 m 且簇 473→179→14 体素
  在消解；候选 2 ±15° 两姿态都看不到）→ 属命中率波动，非回归。
- **d3（s1015-130823）**：**首次全链路闭合** ——

```text
652 帧：dry_run 380 / obstacle_slow 190 / duration 61 / ok 10 / obstacle 7 / idle 2 / stale 2
保持 7 帧 avoid_side=left、前向恒 0.050；脱困 0 帧（未到 20 s 即释放，符合预期）
恢复 200 帧 → episode 79 帧 → 闭合
双指标：绕行闭合率 1/1 = 100%；寻障命中率 1/1 = 100%
creep_floor 生效 57 帧；pose_defer 2 帧；落地 arming_state=1、z≈0.03 m
```

### 已知问题

- **"压速带是吸引子"**（d1 实测）：压速按净空线性缩放，净空越接近 `stop_m` 前向越趋近 0，
  d1 慢速带内前向指令**常常正好 0.000** → 飞机长期停在 0.6~0.9 m，跨不进保持带。
  这与"没有障碍"是两回事，属控制律问题；已用 `obstacle_slow_min_forward_mps`（0.05）应对，
  待服务器侧确认是否作为试验默认（不认可就退回 0）。
- 交权点固定 **0.7 m**（d3 用该值成功）；是否回到 0.8 待确认。

---

## [2026-10-15] 本机环境：Windows 保留端口挡住 AirSim 的 45000（已修，**待撤销**）

### 新增

- `scripts/win_port_exclusion.sh`（`status` / `apply` / `revert`）：处理"Windows 保留端口段
  压住 AirSim `ApiServerPort`"这一本机环境问题；`status` 会打印保留段并做**真实 bind 测试**
  （试完立即释放），`apply`/`revert` 通过 UAC 提权执行 netsh 增删。README 第 **4.6** 节
  记录了现象/根因/证据/撤销步骤。

### 已知问题（环境）

- 启动 AirSim 报 `Error at startup: bind: <…权限不允许…>`。根因**不是 AirSim**：
  Hyper-V/WSL 每次开机在 Windows 上随机预留一段 TCP 端口，本轮 `44986–45085` 正好覆盖
  `ApiServerPort=45000` → 没人占用也绑不上。证据：

```text
netsh int ipv4 show excludedportrange protocol=tcp  → 44986–45085（覆盖 45000）
Windows 直接 bind：45000 = FAIL（权限不允许）；45600 = OK
UE4 日志：Waiting for TCP connection on port 4560（它自己绑上的）→ 随后才弹 bind 错误
          ⇒ 失败的是 45000（RPC），不是 4560
```

- 处置（用户选择方案 1）：`scripts/win_port_exclusion.sh apply` 把 45000 设为持久静态保留。
  **⚠️ 这是临时环境改动：实验结束、用户确认后必须 `scripts/win_port_exclusion.sh revert` 撤销。**

---

## [2026-10-14] 按服务器侧选择落地两问 (a)：交权 0.8 m + 复验容差 1.5 m（本地暂停收尾）

### 变更

- `vlm_closed_loop.py`：`pre_stop_clearance_m` 默认 **2.0 → 0.8 m**（`run_detour_trial.sh` 里同值），
  让摆位交权点**贴近保持带**——交权后只需再走 ~0.2 m（VLM 前向 0.12–0.15 m/s ≈ 1.5 s）即可进保持。
  **保持门限 0.6 m 不动**（服务器侧明确：那是安全参数，不该为跑通流程削弱）。
- `vlm_closed_loop.py`：`rot_tol_m` / `rot_dist_tol_m` 默认 **1.0 → 1.5 m**（世界系点/距离一致性容差）。
  依据：1.5 m 对 20 m 外大面是 7.5% 相对误差（仍很紧），而实测假读数漂 3.49 m 照样被拒；
  复验次数**保持"单次通过即算"**（基元已换成簇质心，稳定性足够）。
- `run_detour_trial.sh`：`ROT_TOL` 默认 1.5、`pre_stop_clearance_m=0.8`。

### 验证

- 单元测试：`colcon test` **61 passed**；驱动脚本语法检查通过。
- **未做飞行验证**：服务器侧服务与监控按操作者要求停止，`scripts/preflight.sh` 是 fail-closed
  （要求远端 `/health` ready），因此**未起飞**；本地同步暂停（停 VLM 隧道 + ROS 栈，
  保留 PX4 SITL 与 Micro XRCE-DDS Agent）。

### 已知问题

- ①(a)/②(a) 的飞行验证待服务恢复；恢复顺序见 `docs/agent_comm/77_*.md` 与
  `docs/next-session-handoff.md`。
- 寻障命中率累计 **1/10**；待交权点收到 0.8 m 后看能否稳定产出 episode。

---

## [2026-10-14] 簇质心"限定 ②"（漂移 3.49 m→~1.0 m，验证通过）+ 仲裁实飞联调（修接线 bug）

### 新增

- **锥内连通簇质心**（服务器侧建议的"限定 ②"）：`voxel_grid.cluster_centroid_in_cone()`
  把锥内体素按 **26 邻域连通聚类**，取最近一簇的质心；`local_map` 新增**只读**字段
  `nearest_centroid=(dx,dy,dz)@d@size`（参数 `cluster_min_points=3`）；
  `vlm_navigator._nearest_world_point` 与驱动微转复验的世界点**优先用质心**（退回宽锥/窄锥最近点）。
  动机：`nearest_*` 是"锥内最近点"，在**大平面上会随姿态滑移**，导致"世界点一致"判据对大面失效。

### 修复

- **仲裁接线 bug**（实飞联调时抓到）：`sim_stack.launch.py` 把 `local_planner` 的速度发到
  `/velocity_arbiter/avoid_cmd`，而仲裁器**默认**听 `/local_planner/velocity_cmd` → 话题不一致、
  仲裁器 **0 订阅者**，planner 速度被丢弃。已改为在 launch 里显式对齐
  `avoid_topic/vlm_topic/output_topic`。
- `scripts/stop_stack.sh` 未清理 `velocity_arbiter` → 曾同时存在 **2 个仲裁器实例**；已加入清理列表。

### 验证

- 单元测试：`colcon test` **61 passed**。
- **簇质心复验（c1，s1014-212655）**：同一建筑面（约北 22 m、z≈3.3、簇 385–1199 体素）
  三次复验世界点偏差 **1.37 / 1.02 / 0.97 m** → 第三次通过并交权。
  对比同一场景用"锥内最近点"时的漂移 **3.49 m**（a2）→ 稳定性提高约 3.5 倍。
- **交权后仍未进保持**：c1 交权后 1151 帧 guard = ok 194 / duration 45 / obstacle_slow 8 /
  stale 7；前向净空最小 **0.75 m**（保持门限 0.6 m）→ 只压速、无 episode。
  根因：摆位交权阈值 1.0 m 与保持门限 0.6 m 之间有 **0.4 m 缺口**，需靠 VLM 自己挪完。
- **② 仲裁实飞联调（真起飞 2 m 悬停）**：接线核对 bridge 速度话题 = **1 发布者（仲裁器）+ 1 订阅者**；
  赢家序列（`logs/2026-10-14/arb_live.csv`）：`none → vlm → avoid（抢占，vx 0.87）→ vlm（avoid 过期回落）
  → none（全过期）`；落地 `arming_state=1`、z≈-0.01 m。

### 已知问题

- 待服务器侧定：①交权距离（1.0 → 0.8 m？）或保持门限；②微转复验容差（1.0 → 1.5 m？）。
- 命中率累计 **1/10**；`c2` 未交权（408 帧全 dry_run）。

---

## [2026-10-14] 位姿门阈值修正为 0.45（配对规则）；local_map 加只读宽锥字段；A 诊断轮结果

### 变更

- **位姿门阈值修正**（服务器侧 2026-10-14 指出"阈值是反的"）：`pose_max_yaw_rate` 默认
  **0.6 → 0.45**。原值 0.6 高于 RUN 的 `max_yaw_rate=0.5`（实测峰值 0.59）→ 门在 RUN 期间
  永不触发，等于死区。配对规则写入注释：RUN 0.3 ⇒ 门限 0.45（留 0.15 余量）；
  若 RUN 仍 0.5，门限必须明显更低，否则会"贴着门限反复 defer"。
- **`local_map` 新增只读诊断字段 `nearest_wide=(dx,dy,dz)@d`**（新参数 `cone_wide_deg=35`）：
  与原有窄锥（`cone_deg=30`）并行计算，仅多发布一个字段，**不影响任何判定/安全逻辑**。
  供微转复验区分"真实但被转出窄锥"与"本来就是假的"（服务器侧建议的 A）。
- `vlm_closed_loop.py`：微转复验的采样改用**宽锥**最近点（`_NEAREST_WIDE_RE`），日志同时打印
  宽锥/窄锥距离；`run_detour_trial.sh` 支持 `ROT_DEG`/`ROT_TOL` 外部覆盖，并把数值格式化成
  小数（整数写法会被 ROS 解析成 INTEGER 而报错——a1 就因此**未起飞**，无安全影响）。

### 验证

- **A 诊断轮（a2，s1014-211404，宽锥 ±35° + 微转 ±10°）**：

```text
候选 1（SCAN 前向 3.82 m）：A/B/C 三姿态**宽锥也是 inf** → 停转 2.5 s 内彻底消失
                            → 不是"真实但偏轴"，本来就是假读数
候选 2（SCAN 前向 1.36 m）：A 世界点 (-6.2,+17.9,+1.3)@3.04 m
                            B (-6.1,+18.1,+1.4)@1.63 m / C (-6.4,+21.5,+1.7)@5.08 m
                            → 世界点漂 3.49 m、距离差 3.45 m → 非同一稳定结构
```

- 结论：本场景的失败原因**不是"被转出窄锥"，而是"读数本身不稳定"**。单测 61 passed；
  累计（t11–t16 / esc2 / esc3 / a1 / a2）0 次误交权、0 次误触发围栏。

### 已知问题

- 命中率累计 **1/9**；已向服务器侧建议切 C（接受命中率、主线转 ②仲裁实飞 + 已知点位做可闭合专项），
  待其确认。
- `nearest_*` 是"锥内最近点"而非"被跟踪目标"：对大面积平面，锥一转最近点会在面上滑移，
  对"世界点一致"判据天然偏严；若后续仍要用，建议改为"锥内聚类质心"。

---

## [2026-10-14] 选 B：交权前 ±15° 微转复验 + 位姿质量门；脱困上限 B 实飞验证不再越界

### 新增

- **交权前 ±rot_deg 微转复验**（`vlm_closed_loop.py`，服务器侧 2026-10-14 选 B）：
  SCAN 发现候选 → 静置采样 A → 左转 +15° 采样 B → 转到 −15° 采样 C → 判定
  "**世界点偏差 ≤1.0 m 且距离差 ≤1.0 m**"才算真结构。真结构在世界系固定，拖影会随姿态变；
  微转本身还把"到位后不在前向锥里"的结构扫进锥内（顺带修 Form A）。
  - 新增参数：`rot_verify/rot_deg/rot_rate_deg_s/rot_settle_s/rot_timeout_s/rot_tol_m/`
    `rot_dist_tol_m/rot_pose_age_max_s`；位姿不新鲜时**暂缓判定**（等待，不判拖影）。
  - 新 helper：`heading_enu_unwrapped_deg`、`_rot_sample()`、`_fmt_world()`；
    `_on_map_status` 解析 `nearest_fwd=(dx,dy,dz)@d`（世界系相对坐标）。
- **位姿质量门**（`vlm_navigator` + `safety` + `px4_interface.state`，服务器侧第三节）：
  `state.position_age_s()` + 由航向差估计角速度 → 位置陈旧（>0.5 s）或 |yaw_rate|>0.6 rad/s 时
  **defer（暂缓判定）**：已在保持则维持保持（不动 `stop_streak`、guard 仍 `obstacle`），
  否则只压速到 stale 级、既不累计也不清零 `stop_streak`。每帧 `guard_detail` 落盘
  `pose_age_s` 与 `yaw_rate`。参数 `pose_quality_gate/pose_age_max_s/pose_max_yaw_rate`。

### 修复

- **微转复验的符号 bug**：首版用了 NED 解卷绕航向，而 `publish_yaw_rate` 是 ENU 角速度，
  符号相反 → 越转越远（t15 误差涨到 +193° 后按超时安全退出，未误交权）。
  改用 ENU 口径（`heading_enu_unwrapped_deg`）后，t16 实测 `+396.5°→+414.7°→+379.6°`（±15–18°）。

### 验证

- 单元测试：`colcon test` **61 passed**。
- **脱困上限 B 实飞验证（esc3，s1014-205929）**：两次脱困均 **0.91 m / 9.1 s ≤ 1.00 m / 10 s**
  （旧逻辑为 1.01 m 越界）→ 重试用尽后放弃 → 注入解除后 episode 闭合 1/1。
- **位姿质量门实飞数据（esc3 交权后 337 帧）**：`pose_age` 最大 0.02 s、
  `|yaw_rate|` 最大 0.59 rad/s、**`pose_defer` 触发 0 次**（无误触发）。
- **微转复验（t16，s1014-205602）**：候选前向 1.41 m（世界点 (+0.8,+11.0,+1.4)）→
  ±15° 两姿态下前向锥内均无点 → 判不通过（方向相关读数）→ 回扫描 → 收工降落。
  说明新复验比旧的"静止再看一次"更严（旧口径下这类候选曾交权）。

### 已知问题

- **寻障命中率仍是瓶颈**（t15/t16 无 episode）：绝大多数候选是"转一下就消失"的近场读数。
  已请服务器侧在 A（±10°/放宽锥）/ B（RUN 偏航降到 0.3 的对照轮）/ C（转主线）中定；
  我倾向先跑 B 对照轮。
- 位姿门限 0.6 rad/s 与实测峰值 0.59 rad/s 很接近，是否下调待定。

---

## [2026-10-14] 脱困上限改 B（预判式不越界）；仲裁器落盘归因 + 提到 20 Hz + SITL 冒烟；寻障全表与命中率诊断

### 变更

- **脱困上限改 B**（服务器侧 2026-10-14 建议）：`vlm_navigator` 的脱困结束判定改为**预判式**——
  把"本周期（`command.duration_s`）还会累积的后退"计入，一旦会越过 `escape_max_s` / `escape_max_m`
  就**本帧不再发后退**。边界与 `escape_back_mps`、控制周期无关（调速度也不会越界）。
  日志改为"已后退 X m / Y s；上限…，预判本周期会越界"。
- **仲裁器归因**（服务器侧对仲裁器的第 1 点）：`velocity_arbiter` 新增 `log_csv` 参数，落盘
  `t,winner,age_avoid_s,age_vlm_s,forward,left,up,yaw_rate`（换赢家时必写、其余 ≤2 Hz）；
  `publish_rate_hz` 默认 **10 → 20 Hz**（与 `offboard_bridge` 同频，过期判定跑在控制率上）。
- **寻障全表**（服务器侧第五节建议）：`vlm_closed_loop.py` 的 SEEK 统计由"前 6 个扇区"改为
  **打印全部扇区**，并逐项标注不合格原因（`点<40`/`跨度<6°`/`MAD>1.0`/`中位<6 m`）。

### 验证

- 单元测试：`colcon test` **61 passed**（vlm 41 + px4_interface 20）。
- **仲裁器 SITL 冒烟**（真实栈、输出重定向到 `/arbiter/test_out` 不动飞机）：

```text
两路同喂   → 赢家 avoid（age 0.025/0.005，vy=-0.50）
停掉 avoid → 赢家 vlm（age_avoid 0.525 > 0.5 超时，vx=1.00）
两路全停   → 赢家 none（age 4.975/1.004，不发指令）
CSV 每行含赢家与两路 age（可事后归因）
```

- **t12–t14 连跑**（各自全新 session + preflight 全过 + 安全落地）：命中 **0/3**、无 episode 3/3；
  但全扇区表显示**门限不是瓶颈**——每轮各筛出 4/8/6 个合格大结构（如 t12 扇区 -120° 中位 22.6 m、
  MAD 0.77；t14 扇区 90° 中位 18.5 m、MAD 0.05）。卡点在下游：
  - 形态 A（多数）：扫描期近障碍 0.45–2.6 m，**静止复验第 2 轮变 inf/4.9 m** → 判拖影；
  - 形态 B（仅 t14）：静态复验 2/2 通过（2.35→0.62 m）并交权，但 RUN 阶段航向 −922.9° 后近场读数
    世界系逐帧跳变 → 判拖影 → guard 全程 `ok/duration`，**从未进保持**。

### 已知问题

- 寻障命中率仍低（本批 0/3，累计 1/8）；已把三种改法（A 放宽世界系一致性 / B 原地微转复验 /
  C 换场景）提给服务器侧定方向——**这一条会改变实验设计，未擅自改飞行行为**。
- 仲裁器仍未在"两回路同时真飞"下验证（仅 SITL 冒烟 + 单测）。

---

## [2026-10-14] 按服务器侧口径落地 ①寻障排序 / ②双指标 / ④脱困有界；脱困**实飞验证通过**

### 新增

- `scripts/inject_clearance.py`：新增 `escape`（地面）与 `escape_flight`（飞行中）两个注入场景——
  "两侧都紧的墙"（`f=0.45, l=r=0.4`），用于逼出保持超时脱困。
- `scripts/run_escape_trial.sh`：**脱困的飞行中验证脚本**。只爬升 2 m 即交权，交权瞬间开始注入
  障碍，跑完自动打分。
  - 缘由（重要更正）：安全层只在 **armed + OFFBOARD** 时生效，地面注入被 `vlm_navigator`
    短路成 `guard=idle`（实测 clearance 已收到但 guard 恒为 idle）——所以脱困**必须真飞**才能验证。

### 变更

- **① 寻障排序**（服务器侧 2026-10-14 ①）：`vlm_closed_loop.py` 的 SEEK 合格判据加
  **中位距离下限** `seek_min_median_m=6.0`，`seek_max_mad_m` 由 1.5 **收紧到 1.0**；
  合格扇区里仍取中位距离最小者（"最近的合格大结构"）。
- **② 双指标**（服务器侧 ②）：`scripts/analyze_detour.py` 改为**多轮聚合**并输出
  `绕行闭合率`（闭合 episode / 有 episode 轮次）、`寻障命中率`（有 episode 轮次 / 总轮次）、
  `无 episode 轮次`（单独计数，既不算 FAIL 也不隐藏）；episode 判据加入 `obstacle_escape`。
- **④ 脱困有界 + 有限重试**（服务器侧 ④）：`vlm_navigator` 新增 `escape_max_s=10.0`、
  `escape_max_m=1.0`、`escape_max_retries=1`；脱困到时/到距即结束并恢复常规判断，
  同一段障碍最多再重试 1 次（共 2 次），用尽后报 ERROR "不再后退，交由阶段超时 / max_flight_s /
  监督层兜底"；`guard_detail` 记录 `escape_index` 与 `avoid_side`；障碍解除即复位计数。

### 验证

- 单元测试：`colcon test` **61 passed**（vlm 41 + px4_interface 20）。
- t11 真机（自动寻障流程，s1014-195414）：preflight 全过、410 帧、无 episode、全程安全落地
  → 新门限未误触发（尚未验证"门限能选中真结构"）。
- **脱困实飞（esc2，s1014-200658，注入"两侧都紧的墙"）**：

```text
保持 >20 s → 第 1/2 次脱困 → 10.1 s / 1.01 m 到时结束（有界生效）
再保持 20 s → 第 2/2 次脱困 → 10.0 s / 1.00 m 到时结束
→ 重试用尽：不再后退（交监督层兜底）
注入解除 → 恢复前进 → episode 闭合 1/1 = 100%
44 帧 obstacle_escape：forward 全为 -0.100 m/s；escape_index 1:22 / 2:22；avoid_side=left
guard 分布：stale 140 / obstacle 119 / dry_run 63 / obstacle_escape 44 / ok 24 / duration 13
落地：arming_state=1、z≈-0.02 m（已落地并上锁）
```

- 双指标回算 t6–t10：闭合率 0/1、寻障命中率 1/5、无 episode 4/5（与手算一致）。

### 已知问题

- `escape_max_m` 实测超出上限 1 cm（10.1 s / 1.01 m）：时间×速度的量化误差。已请服务器侧在
  "允许 1 帧量化超出"（A）与"距离一到即归零"（B）之间选一个。
- **寻障命中率 1/5 仍是主瓶颈**：新门限只是"不误触发"，能否提升命中待多轮统计。
- 速度仲裁（`velocity_arbiter`）**尚未与 AirSim/PX4 一起实飞**。

---

## [2026-10-14] 新增速度指令仲裁（②）：`local_planner` 与 `vlm_navigator` 不再抢同一话题

### 新增

- `px4_interface` 新增 **速度仲裁节点** `velocity_arbiter`（`px4_interface/velocity_arbiter.py`，
  入口 `ros2 run px4_interface velocity_arbiter`）。背景：`local_planner`（几何避障）与
  `vlm_navigator`（语义导航）**都**发布 `/offboard_bridge/velocity_setpoint`，同时运行会互相覆盖。
  - 规则：每个来源有独立新鲜度超时（默认 `avoid=0.5 s`、`vlm=1.0 s`）；在**未过期**的来源里按
    `priority`（默认 **`avoid,vlm`**，即几何避障优先）取第一个转发；**全部过期则不发布**，
    交由 `offboard_bridge` 的速度超时保持位置（保守，不把过期指令当新指令）。
  - 纯函数 `select_command()` 与 ROS 节点分离，便于单测。
  - 只有一路来源在发时，行为与"直连"等价（透传）。
- `bringup/sim_stack.launch.py`：新增 `with_arbiter`（默认 `true`）并接入仲裁器——
  `local_planner`/`vlm_navigator` 的 `velocity_topic` 经 `PythonExpression` 解析为
  仲裁器输入（`/velocity_arbiter/{avoid,vlm}_cmd`）；`with_arbiter:=false` 时回到直连旧行为。

### 验证

- 单元测试：`ros2_px4_ws/src/px4_interface/test/test_velocity_arbiter.py`（8 个用例，含优先级、
  逐级回落、全部过期、超时边界、超时覆盖）→ **colcon test：20 tests, 0 failures**。
- 运行时冒烟（无硬件）：起仲裁节点 + 两路 `ros2 topic pub`，节点日志实测
  `速度控制权 → avoid` → （停掉 avoid）`→ vlm` → （全部停）`所有速度来源均已过期 → 停止发布`。
- 启动栈：`ros2 launch bringup sim_stack.launch.py --show-args` 通过；`with_arbiter` 取值
  `true/false` 时 `local_planner.velocity_topic` 分别解析为 `/velocity_arbiter/avoid_cmd`
  与 `/offboard_bridge/velocity_setpoint`（已用 `PythonExpression.perform` 实测）。

### 已知问题

- **未实飞验证**：仲裁节点尚未在带 AirSim/PX4 的完整链路上飞过；沙箱内 DDS 组网受限，
  仅验证到"节点启动 + 逻辑正确"。带硬件联调时需确认 `local_planner` 与 `vlm_navigator` 同时
  运行时只有一个来源在输出。

---

## [2026-10-14] 修复后批次 t6–t10 统计；保持超时"脱困"行为（`hold_max_s`/`escape_back_mps`）

### 新增

- `vlm_navigator` 新增 **保持超时脱困**：`hold_max_s`（默认 **20 s**）+ `escape_back_mps`（默认 **0.10 m/s**）。
  连续处于 `obstacle` 保持超过 `hold_max_s` 仍未释放时（常见于**两侧都紧的大墙面**——释放条件要求
  所选侧 ≥ `obstacle_side_release_m`=1.2 m，贴墙时永不满足），改为**慢速后退 + 朝宽侧偏航**
  （guard 记为 `obstacle_escape`），让障碍重新进入视野/脱困，而不是贴到窗口结束。
  - 位置：`ros2_px4_ws/src/vlm/vlm/vlm_navigator.py`；命令复用同一 `BodyVelocity`，`duration_s` 保留。
  - 触发仅限 `map_guard == 'obstacle'`；离开 obstacle 即清零 `_hold_started_at`。

### 验证

- 单元测试：build + `colcon test`（vlm 包）**41 passed**（含上一条新增的阶段状态机静态单测）。
- 修复后批次 **③ 连跑 5 次（t6–t10，全新 session + preflight 全通过）**：

```text
t6  s1014-185233  无 episode（候选复验未过 → 未交权）
t7  s1014-185519  无 episode
t8  s1014-185813  episode 1 个：208 帧(≈90 s)持续保持、0 帧恢复 → 0/1 ❌ FAIL
t9  s1014-192356  无 episode
t10 s1014-192652  无 episode（另有 1 个单帧闪现，单独计数）
```

  5 轮全部 preflight 通过、**0 次盲交权、0 次误触发围栏、全部安全落地**。
  与修复前（t1–t5：轮级 4/5、episode 6 个）对比，**仅 1/5 轮真正交权**——正是"**无确认不交权**"生效的结果。

### 已知问题

- `hold_max_s`/`escape_back_mps` 行为**尚未实飞验证**（已实现、已过单测；t8 用例即为其目标场景）。
- **寻障排序**现按"中位距离最小"（t7 最佳扇区 3.8 m / MAD 1.49，紧贴阈值），可能偏向近处杂波；
  待服务器侧定口径后决定是否改为"结构质量优先（低 MAD）+ 中位距离下限"。
- "无 episode"轮次是否计入成功率**尚无定论**，建议同时报"绕行闭合率"与"寻障命中率"。

### 变更

- `docs/agent_comm/`：归档服务器侧来信 `65_reply_from_server_regression_ok_and_phase_advice.md`
  （回归对账干净、认可两 bug 定位、建议按阶段分配超时 + 阶段状态机单测，均已实现）；
  新增待转达回信 `64_reply_conservative_batch_and_hold_escape.md`；索引与进度速览同步。

---

## [2026-10-14] 按阶段分配卡死预算 + 阶段状态机静态单测（含有效性验证）；"作废换点"分支已实飞覆盖

### 变更（按服务器侧 2026-10-14 两条建议）

- 监督层卡死预算**按阶段分配**（不再统一 180 s）：新增参数
  `phase_budgets='CLIMB:30|SEEK:45|TRANSIT:90|DESCEND:30|SCAN:35|VERIFY:15|ALIGN:15|APPROACH:45|RUN:150'`，
  未列出的阶段回落到 `phase_stuck_s`；新增纯函数 `parse_phase_budgets()`（便于单测）。
  → ALIGN 真卡 15 s 就兜住（原先要等 180 s），长扫描也不会被误判。
- 新增 `ros2_px4_ws/src/vlm/test/test_phase_machine_static.py`：**阶段状态机静态检查**
  1) 同一语句块里 `return` 之后不得再有语句（死代码检测）；
  2) `on_phase` 每个 `if self.phase ...` 分支必须含 `enter/finish/fail/send('land_fast')`（分支必须能推进）。
  **有效性已用最小复现验证**：死代码用例被抓、无转移分支用例被抓。
  → vlm 测试 39 → **41 passed**。

### 实飞：覆盖"逼近超时/净空回升 → 作废换点"分支（`regE184900`）

```text
候选 1、2：复验未通过 → 自动换点
候选 3：二次复验通过（4.2 m）→ 对准 → 0.03 m/s 逼近 → 净空回升到 inf
        → 重新对准 2 次达上限 → **"该候选点作废"** → **"候选点用尽 → 收工降落"**
记录：VLM 332 帧全部 guard=dry_run；**0 次 enable**（没有交权）；已落地并上锁
→ 服务器侧第 61 封建议的"re-align 达上限就放弃该点"这条**已在实飞中生效**
```

### 附带发现（纪律）

同一轮里还踩了一次**时序型并发**：上一轮 VLM 节点（timeout 未到）仍在运行时就启动了新节点
→ 两个发送者抢同一个推理槽 → 新节点全程 HTTP 503（`server busy: inference slot occupied`）
→ 我方看门狗连续 3 次失败即停止下发（正确）→ 该轮无数据。
**教训**：`preflight.sh` 必须在**启动新节点之后**跑（`run_detour_trial.sh` 的顺序是对的：
清节点 → 起节点 → preflight → 飞）。

---

## [2026-10-14] 回归抓到并修掉两个真 bug（ALIGN 缩进死代码把驱动卡死 11 分钟 / 缺 import time）+ 卡阶段兜底

### 🐞 回归 A 抓到：驱动卡在 ALIGN 11 分钟（心跳照发，监督层兜不住）

```text
现象  驱动日志停在 "→ 阶段 ALIGN" 后再无输出；但 /flight_heartbeat 仍在 2 Hz 发 "ALIGN t=658.4"
      → 12 分钟里飞机一直悬停（期间还漂到 3.58 m），监督层因为"心跳正常"没有兜底
根因  上一轮加"无确认不交权"时把 ALIGN 分支的缩进改坏了：
      `if side_max < closable:` 内层 `return` 之后紧跟"对准完成 → enter('APPROACH')"三行，
      **变成死代码**；于是"对准成功且一侧开阔"时无条件 return，阶段永远停在 ALIGN
修法  恢复缩进（三行与 `if side_max` 同级），并加注释标注该陷阱
```

同轮还修掉：**驱动缺 `import time`**（新加的全局飞行时限用了 `time.monotonic()` → 一启动即
`NameError` 崩溃，飞机根本没起飞）。**preflight 正确抓到监督层有 2 个实例**（fail-closed 生效）。

### 新增兜底（针对"卡住但心跳正常"这一类）

- 监督层 `phase_stuck_s`（默认 180 s）：解析心跳里的阶段名，**同一阶段持续过久 → 下发 land_fast**
  （今天的 ALIGN 卡死就是这种形态，仅靠"心跳停"判据兜不住）。
- 驱动 `max_flight_s`（默认 600 s）：**全局飞行时限**，任何阶段到点即收工降落。

### 回归 C 验证（修复后，自动寻障 → 三个候选点均未找到可复验障碍）

```text
0 次 "收到 enable"（**没有交权**，符合"无确认不交权"）
VLM 记录 375 帧全部 guard=dry_run（**dry_run 期间不再按时长/围栏执法**，记录不被起降点旧原点污染）
三个候选点扫描+二次复验均未通过 → "全部候选点扫描完毕仍未找到可复验障碍 → 按多点预算收工降落"
落地并上锁（arming_state=1）
```

→ 修 1/3 与"无确认不交权"的**兜底路径**已验证；"逼近超时→作废换点"这一分支还需专门构造
（候选要通过二次复验后逼近又停住）才能覆盖。

### 验证

```text
colcon build      ✅（本轮只改脚本，无需重建）
脚本语法/导入      ✅ vlm_closed_loop / flight_supervisor
preflight        ✅ 抓到重复监督层；清理后全 OK
实飞              ✅ 回归 C：无交权、dry_run 记录干净、按预算收工降落
```

---

## [2026-10-14] 修服务器侧指出的三条（enable 原子重置围栏 / 无确认不交权 / dry_run 不执法）+ 暂停收尾

### 修复

```text
1) enable **原子**重置围栏原点：`_on_enable` 直接用"enable 那一刻"的 PX4 位姿写入 `_guard_origin`
   （不再置空等下一帧），同一锁内重置 `_guard_started_at`；日志打印"围栏原点原子重置为当前位姿 (x,y) NED"。
   → 消除 trial4 那种"首个 post-enable 帧按旧原点判 geofence"的窗口。
2) **无确认不交权**：逼近超时且从未到达停止阈值（没有二次复验确认的障碍）→ **不交权**，
   作废该候选点 → 回 SEEK 取下一个候选；候选用尽则收工降落。
   （此前"超时仍交权"会让 VLM 对着空处绕行又撞围栏，双重失败。）
3) dry_run 期间**围栏/时长包络不生效**：`_guard_command` 开头 dry_run 短路（guard='dry_run'），
   此时由摆位脚本控制飞机，记录不再被"起降点旧原点"污染。
```

### 服务器侧两项改动（已知晓，恢复时生效）

```text
· W1 首次真实告警（gpu_mem +4132 MiB）：每趟新 session 是对的，但服务端**旧会话未回收**导致堆积；
  已加会话回收 SESSION_TTL_S=900 / MAX_SESSIONS=8（LRU），下轮生效；
· VLN_EVAL_TRAJ_TOP1=1：下次启动带上，从源头消除 0.1% 的畸形 500（nas token + 缺 act_l0）。
```

### 暂停与环境收尾（用户暂离）

```text
已停：VLM 节点、flight_supervisor、ROS 栈、PX4、Micro XRCE-DDS Agent、SSH 隧道（逐项核对）
未动：Windows 侧 UE4/AirSim
接续清单已更新 docs/next-session-handoff.md（含恢复命令、preflight 纪律、下一轮第一件事）
```

### 验证

```text
colcon build vlm  ✅ 成功
单元测试          ✅ vlm 39 passed
本轮未飞（服务器侧服务已停）
```

---

## [2026-10-14] 打分口径定稿 + ③ 连跑 5 次（4/5 PASS）+ 试验脚本化 + trial4 失败签名

### 打分口径（服务器侧 2026-10-14 定稿，已固化进 `analyze_detour.py`）

```text
episode = 连续 ≥3 帧 **且至少含 1 帧 obstacle（保持态）**
closed  = episode 结束后 20 帧内出现 ≥5 帧前向非零
成功率  = closed / episode，要求 100%；单帧闪现与"仅慢速"区间单独计数报告（不计分、不隐藏）
复核：m4 = 3/3 = 100%；s1014-131724 = 3/3 = 100%
```

### 新增：`scripts/run_detour_trial.sh`（把纪律脚本化）

一次试验 = 按 PID 清理 VLM 节点 → **全新 session id** → 启动唯一一组 VLM（dry_run+timeout）
→ `scripts/preflight.sh` **fail-closed**（不合格直接不飞）→ 自动寻障+完整绕行 → `analyze_detour.py` 打分。

### ③ 连跑 5 次结果（每轮 preflight 全 OK，服务端 0 新增 503 / 0 重置）

```text
trial1 s1014-132843  episode 1（13 帧,left）        → 1/1 ✅
trial2 s1014-133329  episode 2（12 帧 left,5 帧 right）→ 2/2 ✅
trial3 s1014-133723  episode 1（10 帧,left）        → 1/1 ✅
trial4 s1014-134227  episode 1（14 帧,left+right）  → **0/1 ❌**
trial5 s1014-134634  episode 1（14 帧,left）        → 1/1 ✅
轮级 4/5 = 80%；episode 级 5/6 = 83%
```

### trial4 失败签名（下一步修）

```text
该轮摆位"逼近超时"（净空 3.4~3.8 m 反复）后仍交权；交权前后 guard 长时间停在 **geofence**
（seq61 13:42:58 → seq256 13:47:07），即 **enable 之后围栏未按预期重新零点** →
下一步：把 enable 的围栏重置做成原子操作并打印"新原点 + 当帧漂移"，再重跑回归。
```

### 其它

- 服务器侧建议开 `VLN_EVAL_TRAJ_TOP1=1`（约束生成到动作词表，消除本轮的 1 次畸形 500：
  `nas` 文本 token + 缺 `<act_l0_*>`）；**我们已回复同意**（重启代价为零，每趟都用新 session）。

---

## [2026-10-14] 稳健化 SEEK（四重门限 + 多候选 + 作废换点）：自动链路一趟跑出三段闭合绕行

### 变更（按服务器侧 2026-10-14 的两条判别 + 一条流程）

- **SEEK 选点稳健化**：按扇区统计并用四重门限筛选——
  `seek_min_points≥40`、`seek_min_span_deg≥6°`（垂直角跨度，墙/建筑跨多行）、
  `seek_max_mad_m≤1.5`（深度 MAD，大平面离散小）、再按**中位距离**排序；
  产出 `seek_max_candidates`（默认 3）个候选转场点，按中位距离升序。
- **re-align 达上限 → 该候选点作废、换下一个合格扇区**（不再硬交权），
  使每轮 episode 都是"确认过的障碍"。
- `scripts/analyze_detour.py` 增加"episode 最少帧数"参数（默认 3），并同时给出严格口径。

### 实跑（新 session `s1014-131724`，preflight 全 OK，单组 VLM）

```text
SEEK 统计：-90° 3.9 m/5953 点/MAD 1.62 ❌ | -60° 4.0/5160/2.11 ❌ | -30° 4.1/5872/1.79 ❌
           0° 5.1/1167/2.01 ❌ | **-180° 30.2/1044/0.72 ✅** | **210° 30.4/695/0.42 ✅**
→ 生成 3 个候选点；第 1 个点扫描 25 s 无复验障碍 → 自动换点；第 2 点二次复验通过（1.75→1.24 m）
→ 对准 → 0.15 m/s 逼近 → **0.83 m 交权** → 95 s 闭环
打分：episode 1（8 帧，left）✅ / episode 2（11 帧，left+right）✅ / episode 3（**1 帧闪现**）❌
      / episode 4（16 帧，left）✅
→ 严格口径 3/4 = 75% FAIL；**episode≥3 帧口径 3/3 = 100% PASS**（口径待服务器侧确认，不擅自放宽）
```

### 结论

- **自动链路（寻障 → 转场 → 复验 → 对准 → 逼近 → 交权 → 闭环）一趟跑出三段闭合绕行**；
- "垂直跨度 + MAD"两个判别确实把近处杂乱近场（点数多但 MAD 1.6–2.1）与 30 m 外的真实大结构
  （点数少但 MAD 0.4–0.7）分开，证明该判据有效。

### 下一步

口径确定后按 ③ 连跑 ≥5 次统计成功率（每趟：`scripts/preflight.sh` + 全新 session id）；最后做 ② 仲裁。

---

## [2026-10-14] 几何去地面 SEEK + fail-closed 起飞前检查；自动寻障距离估计待稳健化（本轮打分 1/3）

### 新增

- **几何去地面**（替换行带比例，服务器侧建议）：SEEK 阶段把深度像素按 HFOV + 宽高比
  反投影成机体 FLU 点，只保留 `up > -(高度 − seek_ground_margin_m=0.5)` 的点
  （地面被精确剔除，随俯仰自适应），再取保留点中的最近者 → 记录"世界方位扇区 ← 距离"。
- `scripts/preflight.sh`：**fail-closed 起飞前检查**（并发防护脚本化）——
  逐项核对 VLM/offboard_bridge/airsim_camera/local_map/flight_supervisor/PX4/Agent **各 1**、
  `/fmu/out` 可读、地图 `frames>0`、远端 `/health` ready；**任一项不合格 exit 1 不飞**；
  `--session` 顺便生成**全新 session id**（本轮起每趟都用）。
- `scripts/analyze_detour.py`：绕行成功率打分（判据写死；m4 = 4/4 = 100% PASS）。

### 实跑（新 session `s1014-130455`，单节点 + preflight 全 OK）

```text
深度寻障：扇区 −90° 最近 2.6 m（次近 90° 13.4 / 60° 15.0 / −240° 17.7）→ 转场点 6.0 m
摆位：候选 0.75 m → 二次复验 2/2 通过（3.64→1.67）→ 真障碍 → 对准 → 逼近
     逼近中净空多次回升 inf → 重新对准 3 次（上限）→ 35 s 超时后交权
实飞：594 帧；航向净转 −409.5°；打分 **1/3 = 33% FAIL**
```

### 诊断与下一步

```text
问题：自动寻障现在"取保留点里水平最近的一个点"定方位/距离 → 容易锁到近处小物/边缘（本轮 2.6 m），
      于是转场只飞 6 m，交权后遇到的仍是小/瞬态目标 → 绕行不闭合。
改法：a) 按扇区取**中位距离**（对离群点稳健）；b) 要求该扇区**有效点数/占比 ≥ 阈值**（结构要有跨度）；
      c) 在满足 a+b 的扇区里取中位距离最小者，转场距离 = 中位 − 余量。→ 下次跑前实现并验证。
```

### 规矩（已脚本化）

每趟飞行前一律 `scripts/preflight.sh`（fail-closed）+ **全新 session id**；
本轮 236×503 与两次会话重置全部归因于 m1/m2/m3/m5 的节点复用/重叠——m4/m5 之后的数据以
preflight 通过 + 新 session 的轮次为准。

---

## [2026-10-14] 自动寻障（同趟飞行 SEEK）+ 绕行打分校准器；发现深度距离混入地面

### 新增

- `vlm_closed_loop.py` 新增 **SEEK 阶段**（服务器侧建议 ①"把交接写死"）：慢速偏航 360°
  逐帧按 30° 扇区累计"上 55% 行 + 中心列带"的有效深度 → 取**中位深度最小**的扇区 →
  转场点 = 距离×方位（距离 = 中位深度 − `seek_margin_m`，夹在 `seek_min_m`~`seek_max_m`）
  → 自动 TRANSIT → 既有"扫描/二次复验/对准/逼近/交权"流程。
- `scripts/analyze_detour.py`：**绕行成功率打分**（判据按服务器侧口径写死）——
  每个 obstacle episode 结束后 20 帧内出现 ≥5 帧前向非零即"闭合"，要求 100%。
  **m4 打分：4/4 episode 全部闭合 = 100% PASS**。

### 实跑（`session=m5`，新 id、单节点）

```text
深度寻障：扇区 90° 中位 10.9 m（次近 −90° 11.6、60° 12.2、120° 13.4）
→ 转场点 (东 −2.1, 北 +7.7) 距离 8.0 m（被 seek_min_m 夹住）→ 到位后扫描 25 s 未找到可复验障碍 → 收工降落
```

### 发现与修复：深度距离估计混入地面

```text
同一扇区 90°：独立 depth_seek 工具（上 55% 行、不过滤哨兵）= 23.6 m；
              本次飞行内 SEEK（上 55% 行、过滤哨兵）= 10.9 m
原因  上 55% 行里仍含地面（相机有俯仰），地面把中位深度压低 → 转场点算成 8 m，
      而障碍在 ~20 m 外，仍在净空锥 6 m 之外 → 空扫
修法  新增 `seek_row_fraction`（默认 **0.35**）：只取**地平线以上**行带，先排除地面
```

### 规矩（再次验证必要性）

```text
m4 的 VLM 节点（timeout 700 s 未到）在起 m5 时仍活着且已 enable → 抢占位置模式 → m5 爬升超时；
按 PID 清理后重跑成功。→ "每趟飞行前：按 PID 核对 VLM 只有一组 + 每次都换新 session_id"。
```

### 下一步

① 用修正后的行带重跑自动寻障（验证转场点 ≈ 真实结构距离 − 余量）并走完整绕行；
③ 连跑 ≥5 次统计成功率（打分校准器已就绪）；② VLM 与 local_planner 仲裁（安全层覆盖 VLM、同帧同坐标系）。

---

## [2026-10-14] 完整绕行跑通（同一次飞行两轮）：深度图定方位 → 转场 → 二次复验真障碍 → 保持/转向/恢复

### 新增：`scripts/depth_seek.py`（深度图直接找障碍，服务器侧建议 ②）

```text
悬停 2 m、慢速偏航 360°，逐帧取"上 55% 行 + 中心列带"的有效深度，
按 30° 扇区汇总中位深度并排序 → 直接读出"哪个方位有结构、多远"。
实测（真转满 −392°）：
  90° 23.6 m | 60° 24.4 m | −60° 40.7 m | −150° 40.8 m | 150° 54.4 m | 30° 59.5 m
  其余扇区 ≈14000（无回波哨兵值）
→ **结构在正北/东北 23–24 m**：远超本机地图 ±8 m 与净空锥 6 m，故此前 6 m 内扫不到障碍
```

### 完整绕行（`m4`）：真实障碍 + 95 s 闭环 + 交权点 0.47 m

```text
摆位   扫描候选 1.74 m → 第 1 轮复验 1.44×3 通过 → 第 2 轮 0.47×3 通过 → 真障碍
       对准（误差 0.0°）→ 0.15 m/s 逼近 → 交权（净空 0.47 m），enable 已发
实飞   469 帧；guard: ok 236 | duration 156 | obstacle **14** | obstacle_slow 11 | geofence 6
第 1 轮 seq39–41  obstacle_slow(0.52/0.51m,streak1→2) → **obstacle**(streak3,avoid_side=left,yaw+0.15,creep0.05)
        seq49      ok(1.35m) → 释放 → **前向恢复 0.12 m/s**
第 2 轮 seq91–93  obstacle_slow(0.50/0.47m,streak1→2,yaw−0.50) → **obstacle**(streak3,avoid_side=right,yaw−0.15)
        seq97–98   **obstacle**(avoid_side 翻到 left，yaw+0.15)
        seq99      ok(inf) → 释放 → **前向恢复**
收尾   seq306 duration（95 s 到点归零）→ landed=true、已上锁
驱动   353 帧、航向净转 +89.3°、位移最大 8.48 m
```

**结论：在真实、双轮复验确认的障碍上，同一次飞行完整跑通两轮
"慢速区 → 三帧确认 → 保持 + 朝宽侧转向 + 小前向绕行 → 视野打开 → 恢复前进"。**

### 修复/规矩

```text
1) 并发复现（我的锅）：m2 的 VLM 节点（已 enable）未退出，在 m3 爬升时继续发速度指令 → 抢占位置模式、
   爬升超时。改为**按 PID 清理**并规定：每次飞行前核对 VLM 只有一组；重启节点必须换新 session_id
   （m1 那次 frame_seq 136→1 的会话重置就是复用 session_id 造成的）。
2) depth_seek 补心跳：否则监督层 11 s 后误发 land_fast 抢占偏航（本轮实测踩到并修复）。
```

### 下一步

① 把"depth_seek 定方位 → 自动转场 → 复验 → 交权闭环"固化为脚本任务；
③ 连跑多次统计绕行闭合成功率；② VLM 与 local_planner 的速度话题仲裁（集成阶段）。

---

## [2026-10-14] 恢复开发：多点扫描实飞（二次复验拦下瞬态）+ 修 `_world_consistent` 落盘 bug

### 环境恢复

```text
服务器 /health ready:true，基线显存回到 11579 MiB（第三方 UnrealEditor 已退出，读数干净）
WSL：隧道 → Agent → PX4（电池阈值清零）→ ROS 栈（实例 1/1/1、/fmu/out 50 Hz、地图 frames>0）→ 监督层
```

### 新增（按服务器侧四点建议）

- `scan_points` 多点扫描（分隔符 `|`：**`;` 会被 ROS 参数吃掉**，实测踩到并把解析改成多分隔符兼容）；
- `verify_rounds=2` **二次复验硬门限**：同一候选需连续两轮静止窗口都稳定；
- `closable_side_m=2.5` **可闭合门限**：ALIGN 后若两侧都紧 → 直接换下一个候选点（避免"保持不闭合"）；
- 每点 `enable` 时重置安全包络与围栏原点（转场后重新零点）。

### 实飞结果（`scan_points="0,15|12,0|-12,0"`）

```text
北 15 m：扫描候选 2.66 m → 第 1 轮静止复验通过（0.75/0.75/0.53）→ **第 2 轮未通过（4.67×3）**
        → 判瞬态拖影 → 25 s 预算用尽转下一点
东 12 m / 西 −12 m：25 s 内无候选通过第 1 轮 → 转下一点
候选点用尽 → 按多点预算收工降落（landed=true、已上锁）
结论：**起飞点 15 m 范围内的北/东/西三个方向都没有可复现障碍**；
      二次复验是决定性的——c21 只测一轮就放行的那个"真障碍"很可能是同类瞬态
```

### 修复

```text
bug: 地面段（未解锁）所有决策记录写盘失败：AttributeError '_world_consistent'
根因: 该属性只在 `_guard_command` 的地图安全层段赋值，"未解锁/慢响应"提前返回路径不经过
修法: 在 __init__ 初始化；修后 538 帧记录全部写入、0 异常（vlm 39 passed）
```

### 下一步（待服务器侧选）

① 候选点推远/加密（25–35 m 八方向）；② **用深度图直接找障碍**（列方向持续性/最大深度跳变 →
障碍方位+距离，再转场对准交权）；③ 场景侧把起降点放到建筑附近。我们倾向 ②+①。

---

## [2026-10-13] 多点扫描就绪（未飞）+ 暂停飞行实验、环境收尾与接续清单

### 新增（按服务器侧 ① 的四点建议，**本轮未飞**）

```text
① 静止复验升级为"**二次通过**"硬门限：`verify_rounds`（默认 2）——同一候选需要连续两轮
   静止窗口都稳定，避免小/薄结构一次性虚触发
② 多点预算：`scan_points`（形如 "0,15;12,0;0,-15;-12,0"）——每点"扫描+复验"，
   该点失败自动转场下一点；**全部点用尽则收工降落**，不无限扫
③ 目标障碍以"二次复验通过"作为"可复现"判据（转角类更易通过）
④ 每点转场走高 3.5 m；每点交权时 `enable` 都会重置安全包络**与围栏原点**
```

### c21 标注（按服务器侧口径）

`c21` 标为"**单侧可用：保持+转向已验证；完整绕行待复现**"；
`c6–c9` 的净空/摆位数据作废（多栈污染）；`c22` 干净（seq 1..878 无缺号、0 新增 503）。

### 暂停与环境收尾（用户暂离开发）

```text
已停：VLM 节点、flight_supervisor、ROS 栈（stop_stack.sh 父子同停）、PX4、Micro XRCE-DDS Agent、SSH 隧道
核对：px4=0 / agent=0 / vlm=0 / supervisor=0；18080 无监听；/health 不可达（符合暂停预期）
未动：Windows 侧 UE4/AirSim（由用户掌握）
恢复步骤与起飞前检查清单：docs/next-session-handoff.md（含服务端恢复命令）
```

### 验证

```text
脚本语法/导入     ✅ vlm_closed_loop.py（含多点解析：'0,15;12,0;0,-15;-12,0' → 4 个点）
单元测试          ✅ vlm 39 passed
本轮未做飞行验证    （服务器侧服务与监控停止，按约定暂停）
```

---

## [2026-10-13] 503 根因（我方节点无超时）+ 对准/持续复验/围栏放宽；c22 未能复现障碍

### 事故：794×503（服务器侧 W2 真实生效）

```text
根因    c20 的 VLM 节点用 setsid+nohup 启动且**没有 timeout** → 一直存活（服务器侧查到 22:44 仍在发）
        c21 随后又起第二个节点 → **两个并发发送者** → 服务器侧 794 次 503（单顺序客户端不会 503）
处置    清掉全部 VLM 节点（计数 0）；**所有 VLM 节点启动一律带 timeout**；
        起飞前 checklist 增加"仅一个 VLM 节点且带超时"
数据核对 c21 本地记录 seq 38→514 **缺号 0 个**，与服务器侧 c21 n=514 吻合
        → 503 全部落在 c20 残留流；**c21 的避障序列不受影响**（如服务器侧要求保守，可标"待复现"）
```

### 变更（按服务器侧两条建议 + 一条放宽）

- **逼近前先对准**（新 `ALIGN` 阶段）：记录扫描确认方位为目标航向，转向到位（容差
  `align_tolerance_deg=8°`、超时 `align_timeout_s=10`）再前进；逼近速度 0.2 → **0.1 m/s**。
- **逼近中持续复验**：净空回升到 ≥ `scan_found`（障碍脱离视野）→ **停下重新对准**
  （最多 `realign_max=3` 次），不再盲走。
- 交权点围栏放宽：6 → **12 m**（配合对准，让"视野打开→恢复前进"有机会跑出来）。

### 实飞（c22，严格单节点、同一位置北 15 m）

```text
扫描候选 1.05 m → 静止复验 **未通过**（停下后 4.43/4.43/4.85 m）→ 判拖影、继续扫描
32 s 用尽未再出现可复验候选 → 按时间预算收工降落
→ **c21 遇到的真障碍在该点不可复现**（疑为小/薄结构，仅在特定视角出现）
```

### 下一步（待服务器侧选）

① 加"多点自动扫描"（北/东/南/西各 15 m 依次扫描+复验，命中第一个可复现障碍后自动逼近交权）；
② 以 c21 阶段性收尾，完整绕行留到集成阶段；③ 场景侧建议。

---

## [2026-10-13] 真实障碍上的完整避障序列跑通（新增 enable 机制 + 转场扫描）

### 新增：`enable` 机制（消除"节点启动 ~30 s"时间差）

- `vlm_navigator` 新增 `enable_topic`（默认 `~/enable`）：收到 `enable` 时**解除 dry_run**，
  并**重置安全包络起点与围栏原点**到交权点（服务器侧建议的"到点后重新零点"）。
- `vlm_closed_loop.py` 进入 RUN（交权）时自动向该话题发 `enable`；
  推荐用法：VLM 节点**先以 `dry_run:=true` 启动**，摆位脚本做完转场/扫描/逼近后再交权。
- 新增转场阶段：`transit_enabled` / `transit_east_m` / `transit_north_m` /
  `transit_altitude_m`(3.5) / `transit_timeout_s`；到点后降到扫描高度再扫描。
- 新增扫描阶段的**静止复验**（`verify_s`，默认 2.5 s）：候选停下后仍在阈值内才算真障碍。

### 实飞结果

```text
c20（东 12 m）：扫描候选 1.60 m → 停下后 5.51 m → **判定拖影，继续扫描**（复验有效）
c21（北 15 m）：扫描候选 2.43 m → 静止 2.5 s 保持 1.62 m → **判定真障碍** → 逼近 → 交权
  VLM 记录（288 帧）：ok 101 | duration 90 | geofence 88 | obstacle 7 | obstacle_slow 2
  seq38–39 obstacle_slow（0.47 m，streak 1→2，前向→0）
  seq40–48 **obstacle**（streak=3，`avoid_side=left`，yaw +0.15，creep 0.05）
  seq49+   geofence（距**交权点** 6 m，证明围栏原点已按建议重置）
结论：**"真实障碍 → 压速 → 三帧确认 → 保持 + 朝宽侧转向 + 小前向绕行"在真实地图数据上完整触发**；
      "视野打开 → 恢复前进"被外层 6 m 围栏先按住（设计行为，待放宽围栏或先对准后再验证）
取证 logs/2026-10-13/c19_detour.* / c20_* / c21_*
```

### 观察（诚实记录）

逼近段（0.2 m/s 前向）里，静止时稳定 2.5 s 的 1.62 m 读数**在移动中又消失**。
推测是逼近方向与障碍方位有偏差、视野离开目标（而非深度不稳定）。下次可先"对准再逼近"。

### 验证

```text
单元测试          ✅ vlm 39 passed
colcon build vlm  ✅ 成功
实飞              ✅ 真实障碍上完整序列；停下复验两向验证；围栏原点重置生效；落地并上锁
```

---

## [2026-10-13] 修掉监督层误触发（爬升超时真因）；静止复验证明近场读数=自运动拖影

### 修复：监督层在起飞瞬间误发 land_fast（这才是"爬升超时"的真因）

```text
证据    supervisor: 1791900848.338 "心跳已停 712.9 s（>5 s）且飞机已解锁 → 下发 land_fast"
        驱动:       1791900848.2   ARM → OFFBOARD（起飞瞬间）
        桥接:       1791900848.371 控制模式切换：position → velocity
        → land_fast 触发快速降落通道（速度模式向下），与爬升位置目标冲突 → 40 s 爬升超时
根因    监督层的 last_heartbeat 是**上一轮遗留的旧时间戳**；驱动新建节点到首条心跳约 1 s 延迟，
        监督层 1 Hz 定时器正好落在这个窗口 → 按"停了 712.9 s"发指令
修法    新增 arm_grace_s（默认 6 s）：解锁后先给驱动宽限期，期内不判心跳；
        宽限期后仍无心跳时基准取"解锁时刻"而非旧值。修后本轮飞行 **0 次误触发**
        （此前把该现象误判为"地面态残留"，现更正并写进 CHANGELOG）
```

### 新增：扫描变体的"静止复验"（并据此给近场疑点定性）

- `vlm_closed_loop.py` 增加 `VERIFY` 阶段：扫描发现候选后**停下等 `verify_s`（默认 2.5 s）**，
  净空仍 ≤ 阈值才算真障碍（自运动拖影停下即消失）。
- 实测（c18）：2 m 悬停、15°/s 扫满 32 s（≈480°）——**没有任何候选通过静止复验**，
  每个候选停下后净空都回到 `inf`。按时间预算收工降落。

```text
结论：起降点**方圆 6 m 内没有真实障碍**；此前 0.45~0.91 m 的近场读数**全部是自运动拖影**
      （移动/偏航时相机把地面/自身扫进前向锥，停下后随 5 s 衰减消失）。
      与"静止偏航 142 帧从未出现"完全一致；世界系 track 判据本来就拦这类点，这次独立验证同一结论。
取证  logs/2026-10-13/c18_driver.log（扫描/复验日志）、c17_detour.csv、c18_detour.csv
```

### 下一步（待服务器侧选）

完整"保持→绕行→恢复"需真实障碍：① 先用位置控制平移 10–20 m 到场景另一处重做扫描+复验；
② 留到 ③ 集成阶段自然触发；③ 换/重建场景。我们倾向 ①。

### 验证

```text
实飞              ✅ 监督层 0 次误触发；扫描+静止复验按预期收工；落地并上锁
单元测试          ✅ vlm 39 passed
```

---

## [2026-10-13] track（质心）取代链式比较 + 几何门限；查明 AirSim 与 PX4 本地系存在固定偏移

### 变更（按服务器侧两点补充）

- `vlm/safety.py` 新增 `ObstacleTrack`：维护最近 N 帧（默认 3）**世界系候选点的质心**，
  新点必须落在质心容差（`obstacle_world_tolerance_m=0.4`）内才算同一障碍。
  取代"只和上一帧链式比较"——链式会随帧漂移、丢帧即清零。
- `vlm_navigator` 增加**几何门限** `obstacle_candidate_max_m`（默认 4.0 m）：
  只有"前向锥内且距离 ≤ 该值"的点才进入候选/track，避免远处噪声当候选；
  参数 `obstacle_track_size`。记录 `obstacle_world` / `world_consistent` 不变。
- 单测新增 2 项（稳定点连续通过、跳变点被拒、质心只统计最近 N 帧）→ vlm **39 passed**。

### 用 AirSim 真值选障碍：发现两个前提问题

新增 `scripts/list_scene_objects.py`（`simListSceneObjects` + `simGetObjectPose`），实测：

```text
场景物体 211 个、≤60 m 的 107 个，但几乎全是引擎/相机/GameMode/Volume 类 actor
→ **建筑不是独立 actor**（属关卡静态网格），拿不到"有限跨度障碍"的坐标
⚠️ AirSim 的 Drone1 位姿与本机 PX4 本地原点相差约 (东 3.4, 北 −6.9, 上 −0.9) m
   → **AirSim 世界系与 PX4 本地 ENU 存在固定偏移**；地图/规划/避障都在后者，
     若要按 AirSim 物体坐标瞄准，必须先标定并换算该偏移
```

### 建议的替代路径（已在 38 号信里请示）

改走"**扫描变体**"：悬停 2 m → 缓慢偏航 360° 边转边看本机地图净空 → 找到净空较小的方位后
对准并慢速逼近到 ~1.2 m → 交权（`yaw_lock_s=0`）观察"保持→绕行→恢复"。
全程只用自己的坐标系，无需跨系标定。

### 取证

`logs/2026-10-13/c15_detour.csv`（开阔区结论）、`scripts/list_scene_objects.py` 输出（本轮对话）。

---

## [2026-10-13] 世界系一致性判据 + 逐帧速度/俯仰落盘；c15 显示起飞点前方开阔

### 变更（按服务器侧两条判据）

- **逐帧速度/俯仰**：`Px4StateMonitor` 扩展 `vx/vy/vz`（vehicle_local_position）与
  `pitch`（vehicle_attitude 四元数）；决策记录新增 `px4_velocity` / `pitch_deg`，
  与 `map_status`（含最近点相对坐标）同帧落盘，可直接分析"伪点是否与运动/俯仰相关"。
- **世界系持续性判据**：`apply_map_safety(..., world_consistent=)` ——
  上层把地图最近点换算成**世界系 ENU 点**并与上一帧候选点比较（容差
  `obstacle_world_tolerance_m=0.4`）；保持的 streak **只在与上一帧世界坐标一致时累计**，
  世界系里逐帧跳变（自运动拖影）的帧不累计、重新观察。决策记录新增
  `obstacle_world` / `world_consistent` 便于归因。
- 新增单测：`world_consistent=False` 不累计 streak；为 True 时正常进保持（vlm 37 passed）。

### 实飞（c15，干净单栈，`yaw_lock_s=0`，45 s 窗口）

```text
guard：ok 99 | duration 74（**无 obstacle / obstacle_slow**）
前向净空：最小 5.40 m，其余 inf；<1.2 m 帧数 0
摆位：0.25 m/s 前进 40 s（≈10 m）未遇任何障碍 → 超时后原地交权
结论：今天这个起飞点**正前方是开阔区**（>6 m 无障碍），拿不到"保持→绕行"完整序列；
      同时本轮**未再出现瞬态近场读数**（与"多栈"结论一致）
取证：logs/2026-10-13/c15_detour.csv、c15_driver.log、c15_vlm.log
```

### 环境小坑（记录）

```text
连续多轮 land_fast 后出现"已解锁 + OFFBOARD 但 40 s 不爬升"（爬升超时）；
重启 PX4 即恢复（地面态残留，非代码问题）→ 已把"连续实验间隔重启 PX4"加入起飞前清单。
```

### 下一步（待服务器侧选）

要拿到完整"保持→绕行→恢复"，需先"逼近真实障碍"：① 摆位阶段加转向/走航点能力；
② 由服务器侧指定起点→障碍的场景位姿；③ 留到 ③ 集成阶段自然遇到障碍。

---

## [2026-10-13] 落地保护解耦（独立监督层，实测触发）+ 场景②完整链路通过

### 背景（服务器侧安全建议）

`land_fast` 原先写在驱动脚本收尾里，**脚本一崩保护也没了**（判别脚本汇总崩溃那次就是）。
服务器侧建议解耦，三选一或叠加：① PX4 侧 failsafe ② 独立监督进程 ③ 脚本 try/finally。

### 变更

- 新增 `scripts/flight_supervisor.py`（方案 ②）：订阅 `/flight_heartbeat`（驱动脚本 2 Hz 发布）
  与 `vehicle_status`；**已解锁且心跳停 > `timeout_s`（默认 5 s）** → 下发 `land_fast`
  （每 `repeat_s` 重复）。与驱动脚本**不同进程**，脚本崩溃/被 kill/卡死都能兜住。
- `vlm_closed_loop.py` / `map_selfcheck.py` 增加心跳发布与**顶层异常兜底**
  （异常时先 `land_fast` 再退出；正常收尾走 `SystemExit(0)`，不被误记为异常）。
- 方案 ① 现状保持：offboard/手动源丢失 → **位置保持**（`COM_OBL_RC_ACT=0`，早先已验证），
  不改成自动降落（任务里还有 F9 人工接管路径）；进程崩溃由 ② 覆盖。

### 实测触发（不是纸面功能）

```text
c12 收尾：驱动脚本已退出、飞机仍在降落（尚未上锁），心跳停了 5.4 s / 8.4 s
→ supervisor 两次下发 land_fast（logs/2026-10-13/supervisor.log 有 ERROR 记录）
```

### 场景②完整链路（c13，干净单栈）

```text
摆位      前进 ~2 s，前向净空连续 3 次 0.91 m → 停机交权
闭环      173 帧：ok 94 | duration 74 | obstacle_slow 2 | obstacle 3
关键      seq1–2 obstacle_slow（0.45/0.47 m，前向→0，streak 1→2）
          seq3–5 obstacle（streak=3，avoid_side=right）；yaw_lock_s=10 按设计强制偏航 0
          seq6 起变 inf → 释放 → 恢复前向；45 s duration → 归零悬停 → 落地并上锁
结论      状态机链路完整：慢速区 → 三帧确认 → 保持+选侧 → 释放 → 恢复；
          仅"保持期间转向"被我们自己的 yaw_lock 按住（下一轮置 0 即补上该序列）
```

### 待定性（已埋好取证）

每次"摆位前进后交权"最初 1–2 秒会出现 0.45–0.91 m 的净空读数随后转 inf，
而**静止偏航**实验 142 帧从未出现 → 假设是运动/俯仰变化下进入前向锥的近场回波，
悬停后随 5 s 衰减清除。决策记录已新增 `map_status`（地图状态原文，含最近点相对坐标），
下一轮可直接判定"相对机体还是相对世界"。

### 另外

- 隧道再次断开（c12 时 VLM 报 Connection refused）→ 机载看门狗按设计连续 3 次失败即
  **停止下发运动**（日志 "连续 3 / 连续 14 次未取得有效指令"），第 ④ 条闸的真实生效记录；
  隧道已用 `scripts/vlm_tunnel.sh` 恢复。
- `except BaseException` → `except Exception`：避免正常 `SystemExit(0)` 被误记为异常。

### 验证

```text
单元测试          ✅ vlm 36 passed
colcon build vlm  ✅ 成功
实飞              ✅ c13 状态机全链路；supervisor 实测触发；落地并上锁
```

---

## [2026-10-13] 近场伪点定性：干净环境下不存在；过滤改"机体原点系 + 体素中心"

### 变更（按服务器侧 2026-10-13 建议）

- **过滤判据搬到机体原点系且落在体素中心**：点级过滤挡不住量化——离原点 0.31 m 的点
  可能落进中心距 0.22 m 的体素（实测 `nearest_forward=0.23 m` 却已设 0.3 m 过滤）。
  `VoxelGrid.insert(..., min_range_m=)` 新增该判据；`local_map.min_range_m` 默认 0.45 m
  （仍远低于 0.6 m 保持阈值）。
- 新增 `scripts/map_selfcheck.py`：**运动判别**——起飞 2 m → 慢速偏航 90°（7.5°/s × 12 s）
  → 逐帧记录 `nearest_fwd` 的世界系(ENU)与机体系偏移 → 打印两种离散度并落盘 CSV
  （机体系更稳=自身回波；世界系更稳=真实障碍）。

### 判别结果：干净单栈下**没有近场伪点**

```text
飞行样本 142（慢速偏航 90°）：nearest_forward_m 142/142 = inf；dropped_near 142/142 = 0
→ 前向锥内既无真障碍也无自身回波
→ 之前的 0.09/0.22/0.92 m 跳变读数来自**多栈并存环境**（旧实例未退、新旧代码混杂），
  不是本机地图的固有缺陷；此疑点收尾
取证 logs/2026-10-13/map_selfcheck2.csv、map_selfcheck_run2.log
```

### 今日另外修掉的三处环境问题

```text
1) PX4 进程已死（地图 frames=0、无位置可用）→ 重启并重新清零电池阈值
2) PX4↔Agent 的 DDS 链路中断（/fmu/out 完全无数据，两个进程却都在）→ 重启 Agent 恢复，
   验证 /fmu/out/vehicle_local_position_v1 稳定 50 Hz
3) 多栈并存（3×bridge/camera、2~3×map）→ scripts/stop_stack.sh 同时停父子节点；
   起飞前检查清单：实例 1/1/1/1 + /fmu/out 有数据 + 地图 frames>0
```

### 修复（自身脚本）

- `map_selfcheck.py` 汇总时对 `None` 做统计导致崩溃，且**崩溃发生在自动降落之前**
  （那轮未自动落地，已手动 `land_fast` 并确认上锁）；现改为区分"没数据/没障碍"，
  并把原始状态串一并落盘。
- `map_selfcheck.py` 的 `_on_status` 与基类订阅 `VehicleStatus` 的同名回调冲突
  （导致把 VehicleStatus 当 String 解析）→ 更名 `_on_map_status`。

### 验证

```text
单元测试          ✅ vlm 36 passed、mapping 13 passed
colcon build      ✅ mapping 成功（含体素中心过滤）
实飞              ✅ 判别飞行 142 帧全 inf；落地并上锁
```

### 下一步

干净单栈上重做场景②完整链路（摆位 ~1 m + 锁偏航 10 s + 保持三帧确认），
目标是拿到完整"保持 → 朝宽侧转向 → 视野打开 → 恢复前进"序列；之后做 ③ 集成验证。

---

## [2026-10-13] 解"过滤/阈值死结"（保持态恢复可达）+ 查出多栈并存（读数矛盾的真正来源）

### 服务器侧指出的死结（成立，已修）

```text
问题    近场过滤下限取了 0.6 m = 保持阈值本身 → nearest_forward 的下限被钉在 ~0.6 m
        → 满足 "<0.6 m 进入保持" 的数据被过滤掉，guard='obstacle' 几乎不可达
修法    ① 过滤下限 0.6 → 0.3 m（远低于 0.6 m 阈值）
        ② 进入保持改为**连续 stop_confirm_frames（默认 3）帧**确认——时间维滤伪点
        ③ 净空竖直窗由对称 ±0.6 m 改为**下窄上宽**（下 0.25 / 上 0.6），排除光轴下方地面
        ④ 地图状态新增诊断字段：nearest_fwd=(dx,dy,dz)@距离、dropped_near、dropped_min_m
```

### 实飞验证（c11，单栈）

```text
摆位      前进 2 s、净空连续 3 次 ≤1.0 m 后交权
闭环      ok 83 | duration 40 | obstacle_slow 2 | **obstacle 3**
关键      seq1–2 obstacle_slow（0.22/0.09 m，streak 1→2，前向=0）
          seq3–5 **obstacle**（streak=3 触发，avoid_side=right）；本轮 yaw_lock_s=10 → yaw 强制 0
          seq6 起净空跳回 inf → 恢复 ok
结论      **保持态（obstacle + 侧向选择）已可达且有落盘证据**，死结解开
取证      logs/2026-10-13/c11_detour.csv、c11_driver.log、c11_vlm.log
```

### 重大环境问题：多栈并存（今天读数反复矛盾的真正来源）

```text
现象    /local_map/status 的 frames 在 517/1777/2868 间跳变、部分行缺少新字段
根因    pkill 只杀 ros2 launch 父进程时子节点存活；今天反复重启累积出
        **3 个 offboard_bridge + 3 个 airsim_camera + 2~3 个 local_map**
        → 同一话题多个发布者、新旧代码混杂（也解释了更早的一次"爬升超时"）
处置    新增 scripts/stop_stack.sh：同时停父进程与四类子节点并打印剩余计数；
        已恢复到 bridge=1 / cam=1 / map=1 / launch=1 并校验
影响    **c6–c9 的净空/摆位数据作废**（多栈污染）；c10 起为单栈数据，以 c11 为准
教训    每次"重启栈"后必须校验实例计数（这也是服务器侧反复提醒的"确认只有一个节点"）
```

### 仍待解

单栈下仍见短时跳变的近场读数（0.09/0.22/0.92 m，1–2 帧后变 inf）。已具备 `nearest_fwd`
相对坐标诊断；下一步先定性（自身回波 vs 真实薄结构），再决定是否把过滤挪到机体原点系
（0.4–0.5 m，仍远低于 0.6 m 阈值），之后再做完整"保持→绕行→恢复"演示。

### 验证

```text
单元测试          ✅ vlm 36 passed（新增"单帧近读数不立刻进保持"）
                  ✅ mapping 13 passed（新增地面排除、低空边界、最小距离过滤）
colcon build      ✅ vlm / mapping 成功
实飞              ✅ c11 保持态可达；单栈校验通过
```

---

## [2026-10-13] 场景②（自动摆位到障碍前）+ 修掉地图近场伪点（0.6 m 最小距离过滤）

### 新增

- `scripts/vlm_closed_loop.py` 增加**摆位阶段** `pre_position`：
  起飞后按当前机头方向以 `pre_speed_mps`（默认 0.2）慢速前进，直到前向净空
  **连续 `pre_stable_samples` 次（默认 3）** ≤ `pre_stop_clearance_m`（默认 2.0 m）才停下交权。
  要求"连续"是因为实测遇到**单帧瞬时读数**（10 次采样全 `inf`，只有触发帧读到 1.91 m）——
  那是噪声不是障碍（第一次实现就被它骗停过）。
- `scripts/vlm_closed_loop.py` 输出改**行缓冲**（前面几次外部按日志同步阶段失效的原因）；
  摆位行单独记录（`phase=APPROACH`，含 `forward_clear_m`）。

### 修复：地图近场伪点（c4 起飞后立刻"obstacle hold"的真因）

```text
证据    地面静止：/local_map/status 持续 nearest_forward_m=0.17~0.20 m（本应是 inf）
        2 m 交权瞬间：三向同时 0.40~0.53 m，几帧后变 inf
判断    相机近场回波（机身/起落架/起飞残点）被当成障碍 → 无谓慢速/保持
修法    voxel_grid.filter_by_min_range()：丢弃机体坐标系下 <0.6 m 的点（= 避障停止距离，
        更近的信息对决策无价值却最易被自身点污染）；local_map 新增参数 min_range_m 默认 0.6
实测    每帧丢弃 7249~7475 个近场点；体素数 350 → 165；地面读数 0.17 → 0.55 m（真实地面）
测试    新增 2 项（阈值边界保留 / 关闭过滤），mapping 包 11 passed
```

### 场景②实飞（带过滤）

```text
摆位      前进 1.3 s 后前向净空连续 3 次 1.51 m → 停机交权
闭环      119 帧：stale 1 | obstacle_slow 4 | ok 61 | duration 53
关键      seq2–5 净空 0.61/0.63 m → 慢速区把前向压到 ≈0（scale≈0.017）；
          seq6 起净空回到 2.17 m（模型偏航打开视野）→ 恢复前向；30 s duration 后归零悬停
收尾      land_fast 落地并上锁
取证      logs/2026-10-13/c8_detour.csv、c8_driver.log、c8_vlm.log
```

结论：**"障碍 → 压速/近乎停住 → 偏航打开视野 → 恢复前进"已跑通**；
但未进入 `<0.6 m` 的"保持 + 转向"状态（模型一转视野即开）。

### 服务器侧经验（已记入）

任何"数据新鲜度"判据都应取**采样周期的高倍（≥3×）**并对进入/退出用不同门限，
否则会出现 ok/stale 逐帧交替（本轮踩到并已修，见上一条）。

### 下一步（待服务器侧选）

① 摆位阈值 2.0 → 1.0 m；② 交权后先锁偏航 10 s 逼出保持状态；③ 换"有限跨度"障碍做集成验证。

---

## [2026-10-12] C3 第二步（真实避障）：真实障碍触发保持已验证；修掉"陈旧阈值"实践 bug

### 结果

```text
c4（陈旧阈值 0.5 s）
  起飞后本机地图立刻报告很近净空：前/左/右 = 0.39/0.36/0.38 → 0.15/0.24/0.18 m
  guard='obstacle' 4 帧：yaw ±0.15（左/右两个方向都出现）、forward=0.00 → 归零保持
  净空随后恢复（5.19/inf）→ guard='ok' 正常前飞
  118 帧；航向 −156.8°；位移最大 2.16 m；30 s 到点 duration；land_fast 落地并上锁

c5（陈旧阈值改 1.5 s 后复跑）
  guard = ok 66 帧 + duration 52 帧，**无 stale/ok 抖动**
  前向净空最小 1.86 m（本轮 5 m 窗口内无近距离障碍，未触发 obstacle）
  118 帧；航向 −143.7°；位移最大 2.31 m；30 s 到点归零悬停；落地并上锁
```

### 修复：陈旧阈值必须大于地图周期的 3 倍

```text
现象    c4 里 guard 在 ok / stale 之间逐帧交替，前向速度被反复钳位 0.1/0.12 m/s
根因    local_map 跟随相机约 2.25 Hz（周期 0.44 s），而陈旧阈值取了 0.5 s；
        两个自由运行循环每两帧就有一次年龄超阈值
修法    MapSafetyLimits.stale_s 默认 0.5 → 1.5 s（≈允许连丢 3 帧），ROS 参数同步改默认；
        新增单元测试 test_not_stale_within_three_map_periods 锁住语义
fail-safe 不变：地图真断时 1.5 s 内发现，0.3 m/s × 1.5 s ≈ 0.45 m 额外位移
```

### 记录：服务器侧的 W2 真实生效

服务器侧审计发现 c3c 时段出现 **117 次 HTTP 503**，归因是"上一轮 VLM 节点未退出、
两个节点并发请求"→ W2（过载快速失败）把多余请求直接拒掉，而不是静默排队。
这是 W2 的一次真实生效记录；本轮跑前已确认单节点，不再出现 503。

### 验证

```text
单元测试          ✅ 35 passed（新增"1.0 s 年龄不判陈旧"）
colcon build vlm  ✅ 成功
实飞              ✅ 真实障碍触发保持（c4）；修正后无抖动（c5）
取证              logs/2026-10-12/c4_avoid.csv、c5_avoid.csv、c4_vlm.log、c5_vlm.log
```

### 下一步（待服务器侧选场景）

完整绕行（持续障碍 → 绕开 → 恢复）尚未演示：① 窗口放长到 60 s、围栏提到 8–10 m；
② 先用航点层把飞机摆到离墙 2–3 m 再交给 VLM + 避障层。

---

## [2026-10-12] C3 追加三规则（转角上限/释放迟滞/宽侧小前向）实飞验证通过

### 变更（按服务器侧 2026-10-12 建议）

- `vlm/safety.py`：避让偏航默认 0.3 → **0.15 rad/s**；新增
  `side_release_m`（释放要求所选侧也恢复到 1.2 m）、`max_turn_deg`（累积转角上限 60°）、
  `creep_mps` / `creep_open_side_m`（较宽侧 ≥1.5 m 时给 0.1 m/s 小前向绕行；两侧都紧则归零）。
- `vlm_navigator`：按航向累积"进入避障后的转角"并传入安全层；
  `scripts/inject_clearance.py` 增加 `scenario:=turn`（覆盖上面三条）；
  `scripts/vlm_closed_loop.py` 输出改**行缓冲**（此前块缓冲导致外部按日志同步阶段失效）。

### 实飞验证（悬停 2 m，431 帧；`session_id=c3e`）

```text
规则                     期望              实测
宽侧小前向                宽侧≥1.5m→≤0.1     ✅ 49 帧 forward=0.10（同时 yaw=+0.15 左转）
两侧都紧就归零            两侧<1.5m→0         ✅ 22 帧 forward=0.00
转角上限（防死角打转）     累积≤60°            ✅ 55 帧 yaw_capped，首次 turn_deg=-60.1° 后 yaw=0.00
释放迟滞（侧向门槛）       侧向<1.2m 不释放    ✅ f=0.95 l=1 r=1 仍 obstacle；l=r=2 才释放→obstacle_slow(0.07)
避让偏航                  ±0.15 rad/s        ✅ 实测下发即为 ±0.15

guard 分布：geofence 109 | stale 85 | duration 75 | obstacle 71 | obstacle_slow 22 | ok 21
→ 5 m 围栏与 140 s 时长上限均按预期触发（小前向+陈旧期 0.1 m/s 累积位移到 5.11 m），
  内层地图安全层与外层包络叠加工作无冲突。
取证：logs/2026-10-12/c3_turn3.csv、c3_driver5.log、c3_vlm5.log、c3_injector5.log
```

### 执行问题（已修，不影响结论）

- 前两次重跑失败：①观察窗太短（注入未跑完就降落）；②**上一轮的 VLM 节点未退出**，
  在驱动爬升期间继续发速度指令（速度模式优先）→ 爬升超时。改为"先清理残留节点 +
  按日志/文件事件同步"，并给闭环驱动加行缓冲。
- 小前向绕行会累积位移（本轮 5.11 m，触发围栏）→ 真实避障轮将收短观察窗或把 creep 降到 0.05 m/s。

### 验证

```text
单元测试          ✅ 34 passed（新增转角上限、侧向释放迟滞、两侧紧归零等用例）
colcon build vlm  ✅ 成功
实飞              ✅ 三条规则逐条命中（见上）
```

---

## [2026-10-12] Step C3 地图安全层 + 注入式验证通过（四态逐段触发，绕行方向两个方向都对）

### 新增

- `vlm/safety.py`：地图安全层（纯函数，可离线单测）

  ```text
  parse_clearance_status()   解析 /local_map/status 的 nearest_forward_m/left_m/right_m
  apply_map_safety()         fail-safe 陈旧处理 + 压速 + 障碍保持（含迟滞）+ 偏航覆盖
  ```

  阈值（全部可参数化）：陈旧 0.5 s → 前向压到 0.1 m/s；净空 <1.2 m 起线性压速、<0.6 m 归零保持、
  >0.9 m 才释放；保持时朝较宽一侧偏航 0.3 rad/s 并**覆盖** VLM 的 yaw。
- `vlm_navigator` 接入：订阅 `map_status_topic`（默认 `/local_map/status`），
  每帧记录 `guard / guard_detail / clearance / pre_filter / post_filter`；
  参数 `avoidance_enabled`、`map_stale_s`、`stale_forward_limit_mps`、
  `obstacle_slow_start_m`、`obstacle_stop_m`、`obstacle_release_m`、`obstacle_avoid_yaw_rate`。
- `scripts/inject_clearance.py`：C3 确定性注入场景（独立话题，避免被真实 `local_map` 覆盖）：
  不发布 → `ok` → 压速 → 左绕 → 右绕 → 不发布。

### 实飞验证（悬停 2 m，125 s 观察窗，`session_id=c3b`）

```text
注入时段                 期望            实测（帧数 / 下发）
0–8 s   不发布           stale           ✅ stale（前向压到 0.1 m/s）
8–18 s  f=3.0 l=3 r=3    ok              ✅ ok（22 帧）
18–26 s f=0.9 l=2 r=2    obstacle_slow   ✅ obstacle_slow（17 帧，scale=0.5）
26–36 s f=0.4 l=3 r=1    obstacle+左转    ✅ avoid_side=left（23 帧，yaw +0.30 → 航向 −169.9°）
36–46 s f=0.4 l=1 r=3    obstacle+右转    ✅ avoid_side=right（23 帧，yaw −0.30 → 航向 +167.2°）
46–52 s 停止发布          stale           ✅ stale

驱动 CSV guard 分布：stale 97 / obstacle 46 / ok 22 / obstacle_slow 17（与场景逐段吻合）
取证：logs/2026-10-12/c3_injection2.csv、c3_driver2.log、c3_vlm2.log、c3_injector2.log
```

**绕行方向两个方向都做了物理验证**（左更宽→左转、右更宽→右转），
即 `local_map` 的 `left_m/right_m` 与 LightNav `lateral(+ = 左)` 同一套符号。

### 修复

- 逐帧 JSON 记录漏了 `pre_filter/post_filter`（此前只写进发布的话题消息）→ 已补；
- 第一次注入测试的观察窗太短（VLM 节点启动约 25 s + 注入 52 s 超出 95 s 观察窗，
  只跑到第 2 段就降落）→ 重跑用 125 s 窗口，五段全部覆盖；
- 已知调参项：避让偏航 0.3 rad/s 在 10 s 内转了约 170°，对"擦肩而过"过猛，
  真实避障拟降到 0.15 rad/s 并加"净空恢复即停转"（待服务器侧确认）。

### 验证

```text
单元测试          ✅ 32 passed（新增 9 项 safety 测试：fail-safe/方向/迟滞/压速/关闭开关）
colcon build vlm  ✅ 成功
实飞              ✅ 注入场景五段逐段触发，方向正确
```

---

## [2026-10-12] Step C2（加前进分量）通过：全程受控，30 s 包络到点强制归零

### 结果（`session_id=c2`，观察窗 50 s）

```text
状态动作        rot_left ×2、point ×62（64 帧 / 约 28 s 记录）
偏航指令        正（=左）11 帧 / 负（=右）0 帧
航向总变化      −152.7°（PX4 NED，负 = 左转）→ 符号链继续正确
前进指令        实测上限 0.12 m/s（waypoint.forward≈0.15 → /0.375=0.4 → ×0.3 上限）
水平位移        0.00 → 1.66 m（最大 1.66 m，5 m 围栏未触发）
高度            1.54–1.71 m（波动 0.17 m，无持续下降；高度仍交飞控）
包络            guard='ok' 64 帧 + **guard='duration' 10 帧**（30 s 到点后强制归零并保持）
机载看门狗      慢响应闸未触发（往返 min 154 / mean 213 / max 314 ms）、连续失败 0 次
收尾            land_fast → landed=true、已上锁
取证            logs/2026-10-12/c2_closed_loop.csv、c2_vlm.log、c2_run.log
```

### 变更

- `scripts/vlm_closed_loop_c1.py` → `scripts/vlm_closed_loop.py`（C1 只偏航 / C2 加前进共用），
  并补充**水平位移**记录（`px4_x/px4_y/drift_m`）与小结里的"前进指令最大值、位移末值/最大值"。
- 上一轮新增的"实际下发"日志已验证有效：本轮日志里能直接看到 `source=warmup guard=ok 前 +0.00 …`，
  不会再被"缩放前"的值误导。

### 验证

```text
实飞              ✅ C2 通过（见上）
脚本语法/导入      ✅ 通过（修复改名后 main() 仍引用旧类名的错误）
```

### 下一步（C3）

把本机局部地图的三向净空（`/local_map/status`）插进 VLM 命令通道（安全过滤放在 VLM 节点内部，
避免两个节点抢同一个速度话题）；先做**注入式验证**（人为发布 `nearest_forward_m=0.4` 看是否归零保持），
再做真实避障飞行。阈值初值：1.2 m 开始压速、0.6 m 归零并朝较宽一侧偏航。

---

## [2026-10-12] Step C1（只偏航闭环）通过：方向符号链正确，看门狗与包络实测有效

### 结果

```text
指令来源        waypoint[10]（waypoints[0]）
状态动作分布    rot_left ×3、point ×45（48 帧 / 28 s）
我方偏航指令    正（=左）9 帧 vs 负（=右）2 帧
航向总变化      **−102.8°**（PX4 NED，负 = 左转）→ 与左转指令多数相符
偏航幅值        实测恒为 0.50 rad/s（未超设定上限）
高度            交权时约 1.69 m，28 s 内 1.50–1.69 m（波动 0.19 m，**无持续下降**）
包络/看门狗     guard 全程 'ok'：围栏 5 m 未触发、慢响应闸未触发（往返 mean 206 ms）、
                连续失败闸未触发（0 次停止下发）；30 s 上限因观察窗 45 s 先降落而未跑到
收尾            land_fast → landed=true、已上锁
会话            session_id=c1（新会话，frame_seq 1→118）
取证            logs/2026-10-12/c1_closed_loop.csv、c1_vlm.log、c1_run.log
```

**结论**：服务器侧担心的"PX4 yaw 与 LightNav yaw 反号"**不成立**——
我们的接口层在写 `yawspeed` 时已做 ENU↔NED 翻转，物理验证方向正确，无需再取负。

### 修复（日志误导）

- `vlm_navigator` 的"远端路点…→ 前 X m/s"打印的是**缩放前**的值（`forward_scale=0` 时
  会显示 0.40 m/s 而实际下发 0），已在 guard 之后新增一行 `实际下发：source=... guard=...
  前/左/上/偏航`，避免日志误读。

### 文档

- `docs/agent_comm/19_reply_C1_result_to_server.md`（C1 结果 + 下一步 C2 从 0.3 m/s 起步）、
  README 索引与进度更新。

### 验证

```text
colcon build vlm  ✅ 成功
单元测试          ✅ 23 passed
实飞              ✅ C1 通过（见上）
```

### 下一步

C2：加前进分量（巡航上限 0.3 m/s 起步，仍在开阔区，30 s / 5 m 包络），
观察地速与航向是否收敛、有无来回摆；通过后 C3 把本机局部地图/避障接入 VLM 指令通道。

---

## [2026-10-12] 机载看门狗补齐（服务端明确"不能实时急停"）+ C1 闭环驱动脚本就绪

### 背景（服务器侧的重要更正）

服务器侧更正：他们的监控脚本**只把指标写进日志文件**，既不告警也不打断飞行，
本人也无法实时介入——**"服务端喊停"不构成任何安全机制**，实时安全只能放在机载。

### 变更

- `vlm_navigator` 机载看门狗补齐（新增两条，原三条保留）：

  ```text
  ①（原有）失败/超时 → 丢弃本周期；command_timeout_s=3 s 后停止发布 → 桥接节点位置保持
  ②（原有）连接被重置 → 跳过并重试，不打死工作线程
  ③（原有）vertical_mode='hold' → 竖直恒 0，禁止持续下降
  ④（新增）max_consecutive_errors=3：连续失败即清空待发指令并告警，不必等满 3 s
  ⑤（新增）hold_on_slow_response_ms=800：往返超阈值 → 本周期归零保持（guard='slow'）
  ```

- 新增 `scripts/vlm_closed_loop_c1.py`（Step C1 驱动）：起飞 2 m → **交出控制权**
  （不再发目标点，避免与 VLM 的速度设定点抢模式）→ 记录航向（解卷绕）、高度、
  `apos_state`、`waypoint yaw`、我方指令与 `guard` → 汇总**航向总变化**并自动 `land_fast`。
  判据：`rot_left`/`yaw>0` 为主时应观察到航向**减小**（PX4 NED，负=左转）；若为正则反号。
- 文档：`docs/agent_comm/17_reply_onboard_watchdog_and_W1W2.md`（机载看门狗清单 +
  **W1/W2 要做、W3 不做** + C1 参数确认）、`18_reply_from_server_no_realtime_stop_and_W.md`
  （来信增补版归档）、README 索引。

### 验证

```text
单元测试          ✅ 23 passed
colcon build vlm  ✅ 成功（egg-link 指向 build 目录，已确认新代码生效）
C1 驱动脚本        ✅ 语法与导入通过（未飞行前不构成验证）
```

### 下一步

跑 C1（只偏航 30 s 闭环）：确认符号链方向正确、幅值受限、`guard` 正常；
通过后进 C2（加前进分量，0.3–0.5 m/s 起步），最后 C3 接入本机避障层。

---

## [2026-10-12] 服务器侧数据对账通过 + C1 安全包络落地（只偏航闭环准备就绪）

### 服务器侧补的数据（实验 B 那一半）

```text
状态码        全部 200，无 4xx/5xx
会话重置      new episode = 0（frame_seq 从未回退 → 热身/历史全程未被清空）
服务端延迟    min 110 / mean 232 / p95 251 / max 517 ms（按分钟稳定在 235–243）
显存          真实分辨率推理开始后一次性 +2.7 GB，之后全程持平（无泄漏）；GPU 峰值利用率 43%
```

### 请求数口径差已查清：746（服务端窗口）vs 244（我们的扫场 CSV）vs 920（全会话）

根因：**VLM 节点落地后没有停止发帧**（13:21:11 启动 → 13:28:09 超时退出，共 920 帧），
而扫场脚本的 CSV 只覆盖飞行窗口（244 帧）；服务器侧当时拉到的窗口是 746 帧。

两边对账（这是关键结论）：

```text
                我们 920 帧      服务端 746 帧
rot_left             76              76      ← 逐项一致
rot_right            32              32      ← 逐项一致
stop                 14              14      ← 逐项一致
point               798             624      ← 差 174 = 920 − 746（落地后的地面帧）
frame_seq        1 → 920 无缺号      new episode = 0
```

→ 两侧日志互为印证；**不是多段扫场，也不是会话被重置**。

### 变更

- 新增 `scripts/vlm_session_to_csv.py`：把 `vlm_decisions_*/*.json` 全量导出为逐帧 CSV
  （本次导出 `logs/2026-10-12/drone1_session_920.csv`，920 行，含地面段），
  用于与服务端日志对账；已实测 `frame_seq 1→920` 非 +1 处 0 个。
- `vlm_navigator` 新增 C1/C2 用的安全包络（每帧记录 `guard` 状态）：
  `forward_scale` / `lateral_scale`（C1 取 0 = 只偏航）、`run_duration_s`（30 s，进入
  "已解锁+OFFBOARD"后才计时）、`geofence_max_distance_m`（5 m）；到点/越界只归零悬停，
  不自动降落；`stop/not_visible` 归零保持（原有）。
- 文档：`docs/agent_comm/15_reply_746_vs_244_and_C1_envelope.md`（口径差解释 + 偏航符号链说明 + C1 参数）、
  `16_reply_from_server_expB_data_and_C_advice.md`（来信归档）、README 索引。

### 偏航符号链（回应服务器侧的反号提醒）

```text
LightNav yaw（逆时针为正）→ navigator 发 Twist.angular.z（ENU，逆时针为正）
→ offboard_bridge 写 yawspeed 时 flip_yaw_rate_sign() 取负 → PX4 NED（顺时针为正）
```

即**接口层已经做过 ENU↔NED 翻转**，理论上不需再取负（`test_frames.py` 覆盖该函数）。
但 C1 仍要物理验证：`rot_left`（`yaw>0`）必须让机头**左转**，否则立刻停机改符号。

### 验证

```text
单元测试          ✅ 23 passed
colcon build vlm  ✅ 成功
会话导出           ✅ 920 帧、frame_seq 无缺号
```

---

## [2026-10-12] 实验 B（偏航扫场）完成：模型**非退化**，可进闭环

### 结果（244 帧 / 约 140 s / 一次飞行跑完两轮指令）

```text
流程     起飞 2 m → 预热 → 指令A 双向扫(+15°/s×24s → 停5s → −15°/s×24s)
         → ROS 话题切指令B（不重启节点）→ 预热 → 指令B 双向扫 → land_fast（landed=true、已上锁）
会话     drone1 全程一个；frame_seq 1→244 连续无回退（满足服务端"严格递增"要求）
速率     244 帧 / 140 s ≈ 1.75 Hz
取证     logs/2026-10-12/yaw_sweep.csv（逐帧）+ yaw_sweep_run.log + yaw_sweep_vlm.log
```

### 关键证据：状态动作随视角变化（对照 Step B 静止悬停）

```text
Step B（静止 148 s）      point ×325 | rot_left ×2         ← 几乎不变
SWEEP_A_FWD  52 帧        point 51 | rot_right 1
SWEEP_A_BACK 53 帧        rot_right 13 | rot_left 4 | stop 3 | point 33
SWEEP_B_FWD  53 帧        rot_left 8 | rot_right 5 | stop 1 | point 39
SWEEP_B_BACK 53 帧        rot_right 9 | rot_left 7 | stop 3 | point 34
```

即扫场时模型明确输出**转向/停止**（43 次转向、7 次 `stop`），
`apos_x` 也有 **59/244 帧**偏离默认落点 0.5104（极值 0.0104 贴左、0.9896 贴右）。
→ **模型对场景有响应，不是退化**；悬停时的"恒定"是场景静止 + `apos` 为机体系落点所致
（服务器侧的提醒成立）。

### 结论：相关系数不作硬判据

`corr(航向, apos_x)` 各阶段仅 −0.40~+0.51、`corr(航向, wp_yaw)` −0.64~+0.18——
匀速偏航且场景无单一可跟踪目标，本就不该有干净线性关系。判据改为
**"是否曾偏离默认输出" + "状态动作是否变化"**（服务器侧建议），已在脚本与文档中固化。

### 新增/修复

- `scripts/analyze_yaw_sweep.py`：离线汇总扫场 CSV（分阶段统计 + 状态动作分布 + 相关系数，
  并显式统计"偏离默认落点"的帧数）。
- `scripts/vlm_yaw_sweep.py`：修复小结里 `apos_x` 统计被 NaN 污染的问题
  （状态动作帧无坐标，必须过滤 `isfinite`）。
- 文档：`docs/agent_comm/14_reply_yaw_sweep_result_to_server.md`（结果 + Step C 三步设计）、
  `13_reply_from_server_confirm_and_caveat.md`（来信归档）、README 索引与进度。

### 下一步（Step C，小步走）

```text
C1  固定高度 2 m + 只偏航不前进，30 s：验证 waypoint yaw → 偏航角速度的方向与幅值
C2  C1 通过后加前进分量（waypoint forward 归一化、限速 1 m/s），仍在开阔区
C3  把本机局部地图/避障接入 VLM 指令通道（当前两者尚未打通），做避障闭环
```

---

## [2026-10-12] 采纳服务器侧两条硬要求：运行中换指令（不改 frame_seq）+ 热身提到 8 帧

### 背景

服务器侧同意开始实验 B，但点出一个会毁掉实验的坑：**换指令若靠重启 VLM 节点，
`frame_seq` 会回到 1，服务端会判为新 episode 并清空会话历史**——冷启动默认值会覆盖
我们辛苦预热出来的"按画面决策"状态（详见 09/12 号信）。

### 变更

- `vlm_navigator` 新增指令话题 `~/instruction`（参数 `instruction_topic`）：
  收到新文本后**只改指令、不重启、不动会话**，`frame_seq` 继续递增；
  同时把新文本同步给客户端（`client.instruction`）。这是实验 B 两轮指令的切换方式。
- `warmup_frames` 默认 3 → **6**（服务器侧建议 5–8；扫场脚本用 8）。
- `scripts/vlm_yaw_sweep.py` 重写为"一次飞行跑完两轮指令"：
  `CLIMB → 预热8帧 → 指令A 双向扫（+15°/s×24s → 停5s → −15°/s×24s）→ SWITCH（话题换指令）
  → 预热8帧 → 指令B 双向扫 → land_fast`；
  CSV 增加 `phase` 与 `instruction` 列，小结按四个扫场阶段分别给出
  `apos_x` 变化幅度、`waypoint yaw` 范围与 `corr(航向, apos_x)`。
- 文档：`docs/agent_comm/11_reply_ready_to_fly_with_fixes.md`（确认开始 + 说明两处实现）、
  `12_reply_from_server_go_ahead.md`（来信归档）、README 索引更新。

### 验证

```text
单元测试          ✅ 23 passed
colcon build vlm  ✅ 成功
脚本语法/导入      ✅ python3 -m ast 通过；import vlm_yaw_sweep 通过
```

### 下一步

拉起 Agent + PX4 + ROS 栈（跨天后昨天的进程已结束，AirSim/UE4 仍在），
跑实验 B 一次；跑完把 CSV 与四个阶段的相关系数发服务器侧，
由"是否随航向变化"决定进 Step C 还是回头查 checkpoint。

---

## [2026-10-12] 复核服务器侧 `waypoints`：三条全对；采纳其归一化；实测到会话历史强依赖

### 复核（我方真实帧，新会话 `probe_wp1`，4 次请求）

```text
seq=0 153 ms rot_left  waypoints=10  row0 f=+0.0000 l=+0.0000 yaw=+0.3141
seq=1 124 ms rot_left  waypoints=10  row0 yaw=+0.3141
seq=2 126 ms rot_left  waypoints=10  row0 yaw=+0.3135
seq=3 195 ms point     waypoints=10  row0 f=+0.0001 yaw=+0.3146
行结构（seq=0）：row0 yaw=+0.3141 → row1 +0.6283 → row2 +0.9424（每步 +18°），row3 起平移变大
```

- `waypoints` 确实返回、**10 行/帧**，latency 124–195 ms（含隧道）；
- **符号自洽**：我们真实帧在 `apos_state=rot_left` 时 `row0.yaw=+0.3141` → 正 = 左转，
  与服务器侧的合成帧结论一致（`forward` 前正、`lateral` 左正、`yaw` CCW 正、弧度）；
- **行是累积位姿**：yaw 逐行累加（0.3141/0.6283/0.9424），平移到 row3 才明显 →
  只取 `waypoints[0]`；
- 量级：`row0.forward ≈ 0.000–0.0002 m`（该步纯转向），
  用 0.5 m 当巡航阈值确实触发不了，服务器侧的提醒成立。

### 变更

- `waypoint_to_body_velocity()` 改用服务器侧参考归一化：
  `vx=clip(forward/0.375)`、`vy=clip(lateral/0.25)`、`vyaw=clip(yaw/(π/20))`，
  参数化为 `forward_norm_m` / `lateral_norm_m` / `yaw_norm_rad`（替换原 `full_speed_distance_m`）。
  实测后果：真实帧 `yaw=0.3141` → `vyaw` 归一化 2.0 → **饱和到偏航上限**（"以最大角速度左转"），
  `forward≈0` 故不前进——与 `rot_left` 语义自洽。
- `vlm_navigator` 新增 `warmup_frames`（默认 3）：冷启动前若干帧只记录、下发零速度。
- 新增 `scripts/vlm_tunnel.sh start|stop|status`：一键管理到 LightNav-0 的 SSH 隧道
  （01 号信里承诺过但一直没做；今天隧道跨天消失过一次，客户端只报"连接被拒绝"）。
  实测：`status` 能列出进程并打印 `/health → {"ready":true,...}`。
- 文档：`docs/agent_comm/09_reply_verify_waypoints_and_session_effect.md`（复核结果 + 会话策略）、
  `10_reply_from_server_waypoints_on.md`（服务器侧来信归档）、README 索引与进度更新。

### 观察：输出**强依赖会话历史**（我们自己测出来的新证据）

```text
同一张真实帧：Step B（会话 drone1，已喂多帧） → apos_state=point, apos=(0.5104, 0.9815)
             本次探针（会话 probe_wp1，冷启动）→ 前 3 帧 rot_left，第 4 帧才 point，
                                                  apos=(0.0104, 0.9815)（x 贴左边缘）
```

即冷启动的 `rot_left` + `yaw=+0.314` 是"尚未建立运动线索"的默认输出，与画面内容关系不大。
因此实验 B 的设计调整为：**全程同一会话、不中途重启**，起飞后先预热 3 帧再扫场，
两轮指令之间也不换会话。

### 验证

```text
单元测试          ✅ 23 passed（新增/改写：参考归一化、yaw 饱和用例）
colcon build vlm  ✅ 成功
```

---

## [2026-10-11] 按服务器侧结论改控制映射：竖直不再交给视觉 + 启用 waypoints + 偏航扫场脚本

### 背景（服务器侧的关键澄清）

与服务器侧确认后明确了 `apos` 的语义：它是**可通行落点**（"往哪走"的自由空间点），
**不是目标物体**；户外场景里它落在画面底边中心（x_norm 0.5104 / y_norm 0.9815、
`apos_clamped=true`）就是"向前走"，属**预期行为**。他们用另一张 480×270 合成帧复现了同一坐标，
证明这是模型的默认"向前"单元，不是我们场景的伪影。

因此我们 Step B 看到的"下降 −0.51 m/s"**是我们自己的反投影产物**：把画面底边的地面点做三维
反投影，自然得到朝下的分量——地面机器人这么算等于"前进"，无人机这么算等于"往下扎"。

### 变更

- `pointing_to_body_velocity()` 新增 `vertical_mode`（默认 `'hold'`）与 `max_vertical_mps`：
  `'hold'` 时**竖直分量恒为 0**，高度交飞控保持；`'ray'` 保留旧行为并加限幅，仅作对照。
  同一份 Step B 数据：`up` 由 `-0.54 m/s` 变为 `0.00 m/s`。
- 新增 `Waypoint` / `parse_waypoints()` / `waypoint_to_body_velocity()`：
  解析远端 `waypoints`（`[[forward_m, lateral_m, yaw_rad], ...]` 或同义字典，`null` → 空），
  `yaw` 为弧度（`|yaw|>4π` 时按角度兜底）；映射为"水平速度 + 偏航角速度"，**竖直恒为 0**。
- `vlm_navigator` 指令来源优先级改为：
  `stop/not_visible` → 保持 | `waypoints` 非空 → 路点 | `rot_left/rot_right` → 原地偏航 | 其余 → 指向；
  新增参数 `vertical_mode` / `max_vertical_mps` / `use_waypoints` / `waypoint_full_speed_m` /
  `max_lateral_speed`；决策 JSON 新增 `command_source` 与 `waypoints`（便于事后归因）。
- 新增 `scripts/vlm_yaw_sweep.py`（**实验 B**）：悬停中匀速偏航 ±15 °/s × 24 s、中间停 5 s，
  逐帧记录 `apos.x/y`、`apos_state`、`waypoint`、本机航向与我方指令，输出 CSV 与
  `corr(航向, apos_x)`；判据是"扫一圈后 `apos.x`/`waypoint yaw` 是否随航向变化"。
- 文档：`docs/agent_comm/08_reply_waypoints_and_sweep_spec.md`（同意开 `waypoints`、
  给出实验 B 的具体参数与判据、请其确认 `lateral` 正方向与 `yaw` 单位）；
  `docs/agent_comm/README.md` 补上"`apos` 语义"等双方共识。

### 验证

```text
单元测试          ✅ 22 passed（新增 4 项：竖直归零、竖直限幅、路点解析、路点映射）
colcon build vlm  ✅ 成功
离线对照          ✅ 同一条远程响应：
                    apos → forward 1.00 / left −0.02 / up 0.00 / yaw −0.02
                    waypoint[1.0,0,0.3] → forward 1.00 / left 0.00 / up 0.00 / yaw +0.30
```

### 下一步

待服务器侧开启 `waypoints` 并重启后，跑实验 B（两轮指令，共约 4 分钟）：
若指向/路点随航向变化 → 属控制映射问题，可直接进 Step C 的"固定高度 + 前向/偏航"小范围闭环；
若扫完一圈仍恒定 → 再回头查 checkpoint 与输入格式。

---

## [2026-10-11] Step B 验收：链路/延迟/深度达标，**指向语义不达标**（画面底边饱和）

### 结果（`vlm_decisions_stepB/`，327 条记录 + 327 张帧图）

```text
窗口        21:39:54 → 21:42:23（148 s，实测 2.2 Hz）
请求数      327，异常 0（无解析错误、无超时、无线程异常）
往返 ms     最小 151 / 均值 237 / p95 271 / 最大 383（仅 7 帧 >300）
apos_state  rot_left ×2（首两帧，预期）→ point ×325
opos_state  not_visible ×327
本机深度    327/327 有效，2.88–2.98 m（远端占位恒为 1.00 m，已被替换）
指向像素    325/327 恒为 (653, 706) → x_norm 0.5104 / y_norm 0.9815，远端 apos_clamped=true
指令        前 0.95 m/s、上 −0.51 m/s（99% 帧含下降）、偏航均值 0.10 rad/s
悬停        2 m 保持 240 s（真值 Δz=+0.015 m），落地 landed=true、已上锁
```

### 判定（对照四个通过标准）

```text
① 连续 ≥60 s、≥100 次请求无异常     ✅ 148 s / 327 次
② 指向语义合理                      ❌ 325/327 指向**同一个像素且贴画面底边**（y_norm=0.98、
                                       apos_clamped=true），该点深度落在地面（2.9 m），
                                       指令近乎恒定（前飞 + 下降），与"前往前方倒塌建筑"不符
③ 端到端往返稳定（均值<300 ms）      ✅ 均值 237 ms、无 >1 s 长尾
④ 本机深度采样有效                   ✅ 100% 有效
```

结论：**接口、链路、延迟、深度四项证据充分；唯独指向语义这一条不通过**，
因此不直接进入 Step C 闭环飞行——按现在的输出闭环会"一边前飞一边下降"。
指向不动的直接原因是 148 s 内场景静止（悬停），但**贴边 + 触边标志**说明目标在视野之外
（大概率在画面下方），需要与服务器侧核对后再定处理策略。

### 修复（本轮暴露的第二个必现 bug）

- 状态动作帧（`rot_left/rot_right/stop/not_visible`）没有像素坐标，`x_norm/y_norm` 为 NaN，
  而"发布 + 日志 + 落盘"这段原本在 `try` **之外**：`pixel_from_norm(NaN)` 抛 `ValueError`
  直接打死工作线程。由于**首帧必然是 `rot_left`**，每轮都是"跑了一条就哑掉"（本轮第二次重跑实测 0 条）。
  现在整段收进 `try`，并按 `pointing.has_position` 分流打印；`point_pixel/point_norm` 为
  `null` 表示状态动作。

### 新增

- `scripts/analyze_stepB.py`：汇总决策 JSON（条数 / 延迟分位 / `frame_seq` 连续性 /
  `apos`·`opos` 分布 / 指向像素范围 / 本机深度有效率）。
- 决策与帧图落盘：`vlm_decisions_stepB/`（327 个 JSON + 327 张 JPEG，**未入库**）。
- `docs/agent_comm/07_stepB_result_to_server.md`：本轮结果与指向饱和问题，交服务器侧核对。

### 观察

- `frame_seq` 1→327 无缺号、无重复（服务端要求严格递增，已满足）；文件名毫秒字段有 3 处
  次序异常（WSL 墙钟微调所致），以 `seq` 为准。
- 远端返回的 `apos_px=[653.33, 706.67]` 恰好等于 `frame_size` 的底部边界附近，
  与 `apos_clamped=true` 自洽：模型的指向**被钳在画面下边缘**。
- 本机深度 2.9 m 与"2 m 高度俯视地面"一致，说明该指向对应的物理目标在**地面**上。

### 已知问题（Step C 前置）

1. 指向饱和在画面底边：需判断是"目标确实在视野下方"还是"指令-场景不匹配 / 4 Hz→2.2 Hz 域差异"；
2. 若维持现状进入闭环，必须对**下降分量**做保护（我们已有 `stop_distance_m`，但垂直方向仍需限制）；
3. `opos` 通道恒为 `not_visible`（服务器侧确认该 checkpoint 有此通道但无输出）。

---

## [2026-10-11] 归档与服务器端通信文档：新建 `docs/agent_comm/` + 当前问题请求

### 变更

- 新建 `docs/agent_comm/`，把它此前散落在仓库根目录的 4 份往来文档归档并按时间编号
  （用 `git mv`，保留历史）：

  ```text
  agent_communicate_reply.md       → docs/agent_comm/01_reply_v2_interface_to_server.md
  agent_communicate.md             → docs/agent_comm/02_reply_from_server_stepA.md
  agent_stepB_notice.md            → docs/agent_comm/03_stepB_notice_to_server.md
  agent_communicate_reply_stepB.md → docs/agent_comm/04_reply_stepB_to_server.md
  ```

- 新增 `docs/agent_comm/README.md`：说明文件式通信约定（`NN_<主题>_to_server.md` /
  `NN_<主题>_from_server.md`、来信落到 `agent_communicate.md`）、文件清单与状态、
  以及双方都要知道的关键结论（相机实测速率、不依赖 `opos`、深度由本机补、首帧 `rot_left` 正常）。
- 新增 `docs/agent_comm/05_request_service_down_to_server.md`：给服务器侧 agent 的**当前问题与需求**——
  服务在 Step B 窗口内被正常关闭的证据（时间线 + `server_http.log` 原文 + 我方探测结果）、
  六条需求（重启并保持 / 窗口内不要重启 / 给启动命令与版本标识 / 确认重启语义 / 留意 GPU 与排队 /
  无需额外操作），以及服务恢复后我方的重跑流程与通过标准。
- `README.md` 目录树补上 `docs/agent_comm/`。

### 说明

- 本条目只是**归档与文档**，不改动任何代码；此前提交里出现的旧路径名是历史记录，未回溯修改。
- 服务器侧回信位置随之变化：请写到 `docs/agent_comm/agent_communicate.md`（已在该目录 README 中注明）。

---

## [2026-10-11] Step B 首轮：悬停链路通过，远端服务中途关闭 → 决策数据为空（待重跑）

### 变更（本轮联调暴露的问题，已修）

- **远端抖动会杀死工作线程**：服务器/隧道抖动时 `urllib` 抛 `ConnectionResetError`，
  它不属于原先捕获的 `VlmError/PointingParseError/ValueError`，直接抛穿 `_vlm_loop` 线程——
  节点看起来还在运行，实际**再也不会产生任何决策**（本轮就是这样：240 s 悬停、0 条记录）。
  现在 `RemoteHttpVlmClient.complete()` 把 `http.client.HTTPException / OSError`
  （含连接重置、超时、BrokenPipe）统一转成 `VlmError`；`_run_single_decision()` 再加兜底
  `except Exception`，任何未预期异常只丢弃本周期。
- 新增参数 `remote_timeout_s`（默认 3.0 s）——原先 `create_client` 默认 30 s，一次故障会卡满半个周期。
- 新增参数 `remote_url` / `session_id`；并修正 `create_client()` 不透传指令的问题：
  此前 `provider:=remote_http` 会把**中文提示词模板**当指令发给远端（接口约定是英文自然语言），
  现已按 `instruction` 参数传英文指令。
- 决策记录文件名加毫秒与 `seq`（原按秒命名在 2.25 Hz 下会互相覆盖）；新增 `record_frames`
  把当帧 JPEG 存到 `vlm_decisions*/frames/`，便于"帧图 ↔ 响应"核对。
- `scripts/bringup.sh` 补可执行位（原为 644，`./scripts/bringup.sh` 报 Permission denied）。

### 验证（本轮实测）

```text
PX4 SITL + AirSim      ✅ 连接 TCP 4560、uxrce_dds_client 同步、Ready for takeoff
电池阈值                ✅ 运行中置 BAT_LOW_THR/BAT_CRIT_THR/BAT_EMERGEN_THR=0（param show 确认）
ROS 栈                  ✅ offboard_bridge / airsim_camera / local_map / local_planner 全部在线
相机速率（20 s 窗口）    ⚠ color 1.33 Hz、depth 1.78 Hz（低于早先记录的 2.25 Hz，后续以实测为准）
起飞悬停                ✅ 解锁 → OFFBOARD（nav_state=14）→ 悬停 2 m 共 240 s
                          AirSim 真值 Δz=+0.015 m；PX4 估计在 -2.00 附近 ±0.02 m 振荡（Δ=-0.244 m）
降落                    ✅ land_fast → landed=true，arming_state=1（已上锁）
远端 VLM 决策            ❌ 0 条：服务器侧 server_http.py(pid 1465986) 在窗口内**正常关闭**
                          （日志 "Shutting down / Finished server process [1465986]"），
                          隧道 127.0.0.1:18080 随后不再响应
```

取证日志：`logs/2026-10-11/stepB_hover_attempt1.log`（240 s 悬停高度对比）、
`logs/2026-10-11/stepB_vlm_attempt1.log`（线程被连接重置打断的原始 traceback）。

### 结论与下一步

- 本机侧链路（相机 → VLM 节点 → 隧道）与飞行侧（起飞/悬停/降落）都通过；
  唯一缺口是**服务器服务在联调窗口内不在线**，与模型质量无关。
- 服务恢复后重跑 Step B（约 4 分钟：起飞 + 2–3 分钟记录 + 降落），验收标准仍是
  ≥60 s / ≥100 次请求、往返稳定、本机深度有效。

---

## [2026-10-11] Step B 前置补全：接口 v2 的 `meta`（frame_seq/高度/航向）贯通 + 决策取证

### 变更

- 客户端贯通接口 v2 的 `meta`：`BaseVlmClient.complete(prompt, jpeg, meta=None)`（各 provider 统一签名），
  `RemoteHttpVlmClient` 把 `meta` 放进请求体；`vlm_navigator` 每发一帧自增 `frame_seq`，
  并从 `Px4StateMonitor` 取 `z/heading` 填 `altitude_m`（= `-z`）与 `heading_deg`。
  服务端据此检测客户端重启（`frame_seq` 回退 → 新建会话）。
- 决策取证改进：`_record_decision()` 文件名由"按秒"改为"毫秒 + `seq` 后缀"
  （原命名在 2.25 Hz 下同一秒内会互相覆盖，Step B 要跑 ≥100 次请求会丢数据）；
  记录新增 `frame_seq / latency_ms / apos_state / opos_state`；
  新增参数 `record_frames`（默认 true）把当帧 JPEG 存入 `vlm_decisions/frames/`，
  便于"帧图片 ↔ 响应"一一对照（正是服务器侧想要的 `apos_px`/`opos` 核查材料）。
  文件：`ros2_px4_ws/src/vlm/vlm/vlm_client.py`、`ros2_px4_ws/src/vlm/vlm/vlm_navigator.py`

### 验证

```text
colcon build --packages-select vlm   ✅ 成功（1 min 1 s）
python3 -m pytest test -q            ✅ 18 passed
真实服务 3 次 POST（会话 probe_meta_v2，带 meta）✅ 无 HTTP 错误，RTT 123 / 149 / 159 ms
```

### 观察：`opos_state` 是否可用（回答服务器侧的提醒）

用两张 2026-09-21 实拍帧（`logs/2026-09-21/camera_frame.jpg`、`camera_close_to_target.jpg`）
与 `test_scene.jpg` 各发一次，结果：

```text
camera_frame.jpg           149 ms  apos=rot_left  opos=not_visible
camera_close_to_target.jpg 123 ms  apos=rot_left  opos=not_visible
test_scene.jpg             159 ms  apos=point     opos=not_visible  x=0.5104 y=0.9815
```

`apos` 随画面正常变化（`rot_left`/`point`），`opos` 三帧全为 `not_visible`。
**结论：现阶段不依赖 `opos`**——导航只用 `apos`（指向/可通行方向）+ 本机 32FC1 深度做反投影；
若服务器侧希望启用 `opos`，需要先确认该 checkpoint 的推理入口是否有对象定位输出。
前两张实拍帧返回 `rot_left`（无坐标），与"首帧先转向建立运动线索"的现象一致，属预期。

---

## [2026-10-11] Step B 代码就绪：本机深度替换 + 远端状态动作映射（飞行验证待做）

### 新增

- `vlm_navigator` 支持**在指向像素处采样本机深度**，替换远端返回里的占位深度：
  订阅 `/airsim_camera/depth`（32FC1，1280×720），在 `(x_norm, y_norm)` 处取
  `depth_window_px=3`（7×7）窗口中位数，仅统计 `isfinite 且 > 0.05 m` 的有效像素；
  参数 `depth_topic` / `use_local_depth`（默认 true）/ `depth_window_px`。
  文件：`ros2_px4_ws/src/vlm/vlm/vlm_navigator.py`
- 远端**状态动作映射**：`apos_state` 为 `rot_left/rot_right` → 原地偏航 ±`max_yaw_rate`；
  `stop/not_visible`（无像素坐标）→ 原地保持（零速度）。此前这类响应只会被记成解析异常。
- `agent_stepB_notice.md`：给服务器侧 agent 的 Step B 通知函，需其确认四件事——
  ①期间不重启服务 ②留意 GPU 占用与请求排队 ③重启后同 `session_id` 的会话重建语义
  ④当前服务的模型/代码版本标识。

### 验证

```text
colcon build --packages-select vlm     ✅ 成功（1 min 30 s）
python3 -m pytest test -q（vlm 包）    ✅ 18 passed（仅覆盖 pointing 解析）
import vlm.vlm_navigator               ✅ 通过（新代码可加载）
```

- **飞行验证尚未执行**：Step B 的"起飞悬停 + `dry_run` 记录"待启动，
  本条目在飞行后补实测数值（≥60 s / ≥100 次请求、往返耗时分布、本机深度采样有效率）。

---

## [2026-10-11] Step A 验收通过：真实远端 VLM（LightNav-0）联调成功

### 连接与环境（实测）

```text
SSH          sol@10.255.248.205 密钥登录成功（主机名 ing-server，内网 192.168.31.21）
             → 证实 10.255.248.205 与 192.168.31.21 是同一台机器的不同接口
系统         Ubuntu 22.04.5，系统 Python 3.10.12（服务侧用 venv）
GPU          2 × RTX 4090 D（各 49 GB）
LightNav-0   部署在 /home/sol/lightnav-workspace
服务         127.0.0.1:18080 已在监听（服务侧已就绪）
隧道         ssh -N -L 18080:127.0.0.1:18080 sol@10.255.248.205
```

### 验收结果（按 `agent_communicate_reply.md` 的四条标准）

```text
① GET /health                ✅ {"ready":true,"model":"LightNav-0","rate_hz":4.0,"sessions":1}
② 指向合法（含 state）        ✅ {"pointing":{"x_norm":0.510,"y_norm":0.982,
                                "apos_state":"point","opos_state":"not_visible",
                                "frame_size":[1280,720]}, "latency_ms":152.7}
③ 端到端往返 < 500 ms        ✅ 20 次：最小 153 / 平均 165 / 最大 188 ms
④ 连续 20 次稳定             ✅ 无异常、无丢包；服务端 sessions=1（会话复用正常）
取证                        scripts/vlm_remote_check.py 输出（本轮对话）
```

输入为真实 AirSim 帧（1280×720，45 KB JPEG）+ 英文指令
`"go to the collapsed building ahead"`。

### 修复

- `pointing.parse_pointing()` 对远端格式过于严格：`apos_state` 为
  `rot_left/rot_right/stop/not_visible` 时 `x_norm/y_norm` 为 `null`（这是**预期行为**，
  状态动作本来就没有像素坐标）。改为保留该结果并置坐标为空，
  新增 `Pointing.has_position` 供调用方区分"有坐标的指向"与"状态类动作"。
  修正后同一响应不再报解析错误。

### 观察（供后续判断，不作结论）

- **时序收敛**：连续发送同一帧时，前两次返回 `rot_left`（状态动作、无坐标），
  之后稳定收敛为 `point(0.510, 0.982)`——与服务器侧"模型是时序策略"的说明一致，
  也说明 `session_id` 历史缓冲在起作用。
- **指向偏低**：`y_norm=0.982` 接近画面底部（当前样本是"墙体+地面"的帧），
  语义合理性需要换更有代表性的场景再评估，本次只验证链路。
- 往返 165 ms 远快于本机相机周期（2.25 Hz ≈ 445 ms），**可以做到每帧都决策**，
  原先担心的 0.2–0.5 Hz 低频问题不存在。

### 下一步（Step B）

悬停 + `dry_run` 记录决策：飞机在 OFFBOARD 悬停，把真实相机帧按 2.25 Hz 持续发送、
每次返回的指向与 state 只记录不执行，核对指向语义与延迟稳定性；
同时实现"指向像素处采样本机深度"替换占位深度。通过后再进 Step C 闭环飞行。

---

## [2026-10-11] 远端 VLM 分工确立（LightNav-0）+ 本机客户端就绪

### 背景

用户计划把 VLM 部署在一台 Ubuntu 服务器（ThinLinc 可达），本机负责飞行与实时回路。
与服务器侧 agent 交换意见后（见 `agent_communicate.md` 与 `agent_communicate_reply.md`）确定方案。

### 事实核查

```text
10.255.248.205:22         ✅ SSH 可达（拿到主机密钥指纹
                             ED25519 SHA256:smXunYpCGohmYLRaWJgMjHe7WX4dtOD1uqHeAG2oQlU）
10.255.248.205:80/443     ❌ 未开放
192.168.31.21:22          ❌ 本机路由不可达（服务器 agent 报告的本机网段地址）
结论：10.255.248.205 是同一台服务器在 VPN/ThinLinc 侧的入口，通信走 SSH 隧道
```

### 通信方案（定稿）

```text
服务器：LightNav-0 包装为 HTTP 服务，绑 127.0.0.1:18080（不暴露端口、不改防火墙）
本机：  ssh -N -L 18080:127.0.0.1:18080 <user>@10.255.248.205
        → 本机 http://127.0.0.1:18080 即服务器服务
```

不采用 ROS 2/DDS 跨机（WSL2 NAT + VPN 下多播发现不可靠，且需开放大量端口）。

### 采纳的服务器侧修正（三点，都很关键）

1. **LightNav 是时序策略**（训练时 ~4 Hz、数十帧历史），单帧低频输入是分布外；
   故 Step A **只做"指向"**（单帧感知），本机按 2.25 Hz 持续喂帧并维护 `session_id`。
   注意：本机相机受 AirSim LockStep 限制只有 2.25 Hz，与训练 4 Hz 存在域差异，已记为风险。
2. **指向不含深度** → `depth_m` 由**本机 32FC1 深度图**在指向像素处采样提供。
3. **必须处理 `apos_state`/`opos_state`**（rot_left/rot_right/stop/not_visible 不是坐标）。
   另：联调阶段统一用**英文指令**（模型在英文导航语料上训练）。

### 新增（本机侧）

- `vlm_client.py` 新增 `RemoteHttpVlmClient` 与 provider `remote_http` / `lightnav`：
  POST `{session_id, instruction, image_b64, meta}` → 解析远端 pointing。
- `pointing.py` 支持远端格式（`{"pointing": {x_norm, y_norm, apos_state, opos_state}}`），
  `Pointing` 增加 `apos_state` / `opos_state` 字段。
- `scripts/vlm_remote_check.py`：Step A 验收脚本（/health + 连续 N 次 /navigate + 往返耗时）。
- `scripts/vlm_remote_stub.py`：本地桩服务（实现同一接口，用于服务器就绪前验证客户端）。
- `agent_communicate_reply.md`：给出接口定稿 v2、SSH 公钥与分工。

### 验证（本机侧，用桩服务）

```text
[health] 200 {"ready": true, ...}
4 次 /navigate：往返均 151 ms（桩内模拟推理 150 ms）
解析正确：x_norm/y_norm/apos_state/opos_state 全部按预期读出
请求回显：session=drone1、instruction 正确、image_bytes=8453（8 KB JPEG）
```

### 下一步（等服务器）

1. 服务器侧核对主机密钥指纹、把我的公钥写入 `authorized_keys`；
2. 服务器侧按接口 v2 实现 `server_http.py`（FastAPI，绑 127.0.0.1:18080）；
3. 我建隧道后跑 `scripts/vlm_remote_check.py` 对真实服务做 Step A 验收
   （标准：健康检查通过、指向合法、往返 <500 ms、连续 20 次稳定）。

---

## [2026-10-10] 探索器信息增益评分 + 已访问记忆 + 完成判定验证

### 变更

- `frontier.best_frontier()` 改为**按信息增益评分**：
  `score = 簇规模 / (1 + 0.5 × 距离)`——簇规模约等于未知边界长度，越大越可能通向
  未探索区域；距离做惩罚，避免为远方小边界飞很远。
- 新增**已访问目标记忆**（`visited`，默认 1.5 m 内不重复选择，最多保留 20 个）：
  解决"到达一个 frontier 后又被选回同一处"的原地打转问题。
- 探索器日志新增已访问目标数量。
- 新增 1 个单元测试（已访问目标不被重复选择）。planner 测试 17 → 18，全项目 62。

### 验证：探索完成判定

用参数构造"快速可达完成"的场景（已知半径 12 m 覆盖 ±8 m 局部地图、最大目标距离 5 m），
在飞行中验证完成判定分支：

```text
[frontier_explorer] 探索器启动：… 无 frontier 8 s 判定完成
[frontier_explorer] 探索完成：无可用 frontier（已知 1956 格，障碍 82 格）
取证  logs/2026-10-08/frontier_completion_test.log
```

说明：这是**分支验证**（用参数让 frontier 迅速耗尽），不是"真实探索到覆盖全部区域"的长时验证；
真实完成需要按正常参数长时间飞行，尚未做。

### 已知问题

- 信息增益评分仍是启发式（簇规模 × 距离惩罚），没有遮挡增益/重访代价建模。
- `visited` 只记录目标点而非已探索区域，密集 frontier 场景可能仍会来回切换。

---

## [2026-10-10] 统一 bringup + 阶段 5 自主探索（frontier）

### 新增

- `ros2_px4_ws/src/bringup`（新包）+ `launch/sim_stack.launch.py`：
  **一条命令拉起全栈**（offboard_bridge + airsim_camera + local_map + local_planner），
  用 launch 参数追加任务层/VLM/探索器：`with_mission:=true` / `with_vlm:=true` /
  `with_explorer:=true`，以及 `airsim_host`、`camera_rate_hz` 等。
- `scripts/bringup.sh`：统一启动入口（自动 source 环境）。
- `planner/frontier.py`：frontier 探索纯逻辑——2D 占据/已知栅格、frontier 检测与连通聚类、
  视线遮挡检查、候选选择。
- `planner/frontier_explorer.py`：周期选择 frontier 作为目标发布给局部规划器；
  连续无 frontier 判定探索完成。
- 6 个 frontier 单元测试（planner 11 → 17，全项目 61 个）。

### 验证

```text
统一 bringup   ✅ 一条命令拉起 5 个节点并协同工作
自主探索       ✅ launch 日志：
                 [frontier_explorer] 新探索目标 ENU=(+1.25,+3.75,+2.00)，候选区域数 2
                 [local_planner]    新目标 ENU=(1.25,3.75,2.00)
                 模式切换 → avoid → slow → cruise → arrived（0.43 m）→ 继续下一个目标
```

### 修复

- frontier 目标点选择错误：原实现取**簇质心**，对环形 frontier 会落到圆心
  （实测距机体仅 0.35 m，语义错误）。改为取**簇内离机体最近的 frontier 格子**，并加回归测试。

### 日志整理

- `logs/` 从 701 MB 精简到 188 MB：保留 6 个里程碑 `.ulg` 与全部 46 个文本证据文件。
- 说明：2026-10-08 的 `.ulg` 在清理中被误删（保留列表匹配写错），该日证据仍完整保留在文本日志中。

### 已知问题

- 探索选点只按"距离 + 区域规模"，无信息增益/优先级队列（v1）。
- 探索完成判定（连续 12 s 无 frontier）尚未实测——本次飞行未跑满。
- 探索期间场景较空，规划器多在 cruise 状态。

---

## [2026-10-10] 绕障方向由左右净空决定（含迟滞），替代固定方向

### 新增

- `VoxelGrid.clearance_profile()`：一次给出**前 / 左 / 右**三个方向的最近障碍距离
  （左右按航向 ±90° 锥统计）；`local_map` 的 `~/status` 增加 `left_m=`、`right_m=`。
- `plan_velocity()` 新增 `left_clearance` / `right_clearance` / `previous_avoid_side` /
  `side_switch_margin`：**选择更宽的一侧绕障**；两侧差距小于迟滞阈值时沿用上次方向，
  避免左右摇摆。`AvoidanceCommand` 增加 `avoid_side_used` 记录本次选择。
- `local_planner` 解析三向净空并维护上次绕障方向（迟滞）。
- 新增 4 个单元测试（更宽侧优先、迟滞保持、明确更好时切换、无信息时用默认方向）。

### 验证（飞行实测）

```text
低空 1 m 朝障碍物飞行，规划器日志（含决策依据）：
  avoid（前方净空 0.65 m）向左侧移（左净空 0.71 m / 右净空 1.11 m）
  avoid（前方净空 0.65 m）向右侧移（左净空 inf / 右净空 1.22 m）
  avoid（前方净空 0.67 m）向右侧移（左净空 inf / 右净空 inf）
三向净空随状态实时变化（例：前 0.30 / 左 0.33 / 右 0.34 m，机体处于狭窄角落）
结论：绕障方向不再固定，而是由左右净空决定；两侧接近时保持原方向（迟滞生效）
取证：logs/2026-10-08/{local_planner_leftright.log, local_map_leftright.log}
```

### 已知问题

- 决策证据（左右净空与方向）**打进了日志**，但 `~/status` 里仍是单一 `mode` 字段；
  若要做可视化回放，建议把三向净空也放进状态字符串（本次未做）。
- 在狭小空间（三向净空均 < 1 m）中，飞机表现为贴边滑动而非脱困——符合当前策略能力上限。
- 再次确认：`SIM_*` 与本次的 `BAT_*` 参数**只在内存生效**，PX4 重启后恢复默认，
  因此长会话中会反复触发电池 failsafe（表现为自动 RTL 落地）。做飞行实验前应重设。
- PX4 日志清理会删除构建目录内的旧 `.ulg`（本次删掉了 2026-09-14/16 的原始日志）；
  重要日志务必及时复制到 `logs/`（本次未丢，已在早期归档）。

---

## [2026-10-10] 阶段 4 验证通过：完整避障序列（减速→侧移→恢复→再减速）

### 修复（本轮发现并修掉的真缺陷）

- **净空把地面算成前方障碍**：低空飞行时，30° 锥形区域包含下方地面，
  导致"前方最近障碍"恒为约 0.9 m（实测飞机前进 3.5 m 而该读数不变），
  规划器因此一直处于减速/避让状态而无法正常前进。
  修正：净空只统计与机体**同高度的薄层**（`vertical_tolerance_m`，默认 0.6 m），
  地面点被排除；新增回归测试（地面不构成前方障碍、同高度障碍仍可检出）。

### 验证（1.0 m 低空朝障碍物飞行，默认阈值）

```text
起飞后净空         1.37 m（排除地面后的真实墙体距离）
模式序列           slow（净空 1.37 m）→ avoid（净空 0.74 m ≤ 0.80 m）
                   → slow（净空 1.22 m，侧移后恢复）→ avoid（净空 0.57 m）…
最终位置           NED (-1.00, -2.75)，即向侧向滑动避开，未穿入障碍
过程净空           始终 ≥ 0.5 m（未发生碰撞）
取证               logs/2026-10-08/{local_planner_avoid_sequence.log,
                   local_map_avoid_sequence.log, *.ulg}
```

结论：**深度 → 局部地图 → 局部规划 → PX4 速度控制**的实时避障链路闭环成立，
表现为"减速—侧移脱离—净空恢复—重新接近"的循环，满足"避障不经过 VLM"的架构要求。

### 已知问题

- 侧移方向固定（默认向右），不做左右净空比较，因此面对宽墙时表现为**贴边滑动/绕圈**
  而非找到通路；这是 v1 的已知局限，后续可加左右净空比较或接入更完整的局部规划器。
- 本验证中飞机未能到达目标（被障碍阻挡）——这是当前策略下的**期望安全行为**，
  但"绕障后继续前往目标"尚未验证（需要能绕开的障碍布局）。
- 地面排除使用固定高度薄层，机体大姿态倾斜时薄层与地面可能相交，后续应结合姿态计算。

---

## [2026-10-10] 阶段 4：局部规划器（目标 → 速度 + 避让分支）

### 新增

- `ros2_px4_ws/src/planner`（新包 0.1.0）：
  - `avoidance.py`：纯逻辑决策（可单测）——到达判定、巡航、按净空减速、
    净空不足时**停止前进并侧移**（默认向右，可配）。
  - `local_planner.py` 节点：把目标点（ENU）+ 局部地图净空 → 速度指令发布到
    `/offboard_bridge/velocity_setpoint`；仅当 armed+OFFBOARD 且非 failsafe 时发布；
    地图状态超时（默认 1 s）按“保守减速”处理；**新增偏航控制：机头对准目标方向**，
    使净空锥（相机）与行进方向一致。
- `scripts/send_goal_relative.py`：按当前航向把目标设在前方/侧方，便于复现验证。
- `planner/test/test_avoidance.py`：7 个单元测试。测试总数 43 → **50**。

### 验证

```text
目标跟踪        ✅ 目标设在正前方 4 m，规划器驱动到距目标 0.25 m（巡航/到达分支正常）
避让分支        ✅ 将 stop_distance 临时调大以触发：
                  status: mode=avoid clearance_m=1.79 reason=前方净空 1.79 m ≤ 3.00 m，侧移脱离
```

### 结论修正（同日晚些的决定性实验）

此前怀疑"速度指令坐标系不符"的判断**是错的**，已用决定性实验推翻：

```text
起始 ENU=(-0.04,+0.10)，航向 +77°（朝北）
指令   ENU 东向 0.6 m/s × 5 s
实测   位移 Δe=+3.37 m、Δn=-0.12 m
判定   与世界轴假设相差 2°，与机体系假设相差 79°  → 速度设定点按世界系(ENU→NED)解释 ✅
取证   scripts/test_velocity_frame.py
```

避让测试中"北向位移 7.8 m"的真实原因：侧移脱离后净空恢复 → 规划器转入巡航朝目标飞，
10 s 巡航足以覆盖 7.8 m。也就是说该次观测是"避让 + 巡航"的合成结果，
而我当时只读了单点状态快照，误判成坐标系问题。

### 已知问题

- 避让过程的**模式序列**此前未被记录，导致只能靠单点快照推断。已在 `local_planner`
  增加模式切换日志（打切换事件而非节流打印），后续测试可直接回放 avoid→cruise 过程。
- `planner/local_planner.py` 中 `ned_to_enu(*position_ned)` 在偏航计算里被调用了两次，
  属冗余（不影响正确性，后续清理）。

---

## [2026-10-10] 阶段 4 起步：局部障碍地图（mapping/local_map）

### 新增

- `ros2_px4_ws/src/mapping`（新包 0.1.0）：
  - `voxel_grid.py`：纯逻辑体素栅格（不依赖 ROS，可单测）——
    量化、范围裁剪、时间衰减、占据中心输出、**前方锥形区域最近障碍距离**；
    以及机体 FLU → ENU 的航向旋转。
  - `local_map.py` 节点：订阅 `/airsim_camera/points`（机体 FLU）与 PX4 位姿，
    把点云转到 ENU 世界后累积成体素地图；发布
    `~/occupied`（PointCloud2，占据体素中心）与 `~/status`
    （体素数、处理帧数、前方最近障碍距离）。
  - 参数：分辨率 0.15 m、局部范围 ±8 m、高度窗口 -2~6 m、衰减 10 s、
    前视锥 30°/6 m，均可配。
- `mapping/test/test_voxel_grid.py`：8 个单元测试（旋转、量化、范围裁剪、衰减、
  锥内最近距离、最大距离限制等）。测试总数 35 → **43**。

### 验证（实时）

```text
节点           airsim_camera(2.5 Hz) + local_map（分辨率 0.15 m，±8 m，衰减 10 s）
体素累积       363 → 364（衰减生效后回落到 181 → 179，滑动窗口行为符合预期）
处理帧数       75 帧（约 30 s，与相机 2.5 Hz 一致）
前方最近障碍   0.25 m（飞机当时正贴近一个障碍物，数值合理）
占据点云       与体素计数一致（363 / 364 / 181 / 179）
取证           logs/2026-10-08/{local_map_console.log, local_map_camera.log}
```

### 已知问题

- v1 简化：假设**相机与机体同向、无安装偏移**（settings.json 中相机 pitch=0、偏移很小），
  尚未引入 TF 树；相机姿态与机体不一致时需要补外参变换。
- 点云解析假设发布方固定为 3×float32（12 字节/点），与本项目自产点云一致；
  接入第三方点云需改用通用解析。
- 地图只保留机体周围 ±8 m，超出即丢弃；全局地图与语义地图留待后续阶段。
- 实验记录的两次踩坑：`&` 后台任务未继承 ROS 环境导致 `ros2: command not found`；
  `pkill -f "local_ma[p]"` 与命令中的字面量自匹配（第 4 次），清理需单独执行。

---

## [2026-10-09] 阶段 3：深度提频 5.6 倍 + 点云发布

### 关键测量（修正此前的错误优化）

对 AirSim 取图开销做基准测试：

```text
仅彩色 1280×720            334 ms → 3.0 Hz
仅深度 640×480（float）    334 ms → 3.0 Hz
彩色+深度（同一次调用）     350 ms → 2.9 Hz   ← 几乎不增加开销
```

**开销由每次 RPC 调用的固定延迟（约 330 ms，LockStep 步进）决定，与数据量无关。**
因此此前"深度每 5 帧抽一次"的优化是基于误判（当时把测量窗口过短导致的
`ros2 topic hz` 抖动当成了真实帧率），实际既没必要也代价很大。

### 变更

- `airsim_camera`：`depth_interval` 默认 5 → **1**（每帧都取深度），
  采样率默认 2.0 → **2.5 Hz**（在 3 Hz 上限内留余量）。
- 新增 **点云发布**：`~/points`（`sensor_msgs/PointCloud2`），
  由深度图按针孔模型反投影生成，坐标转换为机体 FLU（x 前、y 左、z 上），
  参数 `pointcloud_stride`（默认 4）与 `max_depth_m`（默认 10 m）。

### 验证

```text
image   2.25 Hz（此前 2.04 Hz）
depth   2.25 Hz（此前 0.4 Hz，提升 5.6 倍）
points  2.20 Hz，单帧 9440 点（stride=4、10 m 内）
取证    logs/2026-10-08/airsim_camera_depth_pointcloud.log
```

### 修复

- 布尔参数属性 `_publish_pointcloud` 与同名方法冲突（`TypeError: 'bool' object is not callable`），
  属性改名为 `_publish_cloud`。这类命名冲突只有启动冒烟才会暴露，再次印证"新通道必须跑一次"。

### 已知问题

- 深度/点云上限约 3 Hz（受 LockStep 每次 RPC 330 ms 限制）。
  若避障需要更高频率，需要改 AirSim 侧配置（降低深度分辨率、或启用 LiDAR 并减少点数），
  且改 settings.json 需重启仿真。

---

## [2026-10-09] failsafe 动作感知的任务语义（保持类→PAUSED，恢复类→ABORT）已验证

### 变更

- `waypoint_mission` 不再对任何 failsafe 一律中止，改为按 PX4 的**动作类型**处理：
  - `RECOVERY_NAV_STATES = {AUTO_RTL(5), AUTO_LAND(18)}` → 飞控已自主返航/降落，
    任务中止（交还飞控）。
  - 其余离开 OFFBOARD 的情形（人工接管、failsafe 保持类如 POSCTL）→ **PAUSED**，
    保留当前航点，等待交还后续飞。
  - PAUSED 期间同样按动作类型判断，避免"暂停后立刻又中止"。

### 验证（飞行实测）

```text
CRUISE 中杀掉 offboard 桥接
  PX4            → 位置保持（failsafe 保持类）
  任务层         ✅ “检测到离开 OFFBOARD（nav_state=2，failsafe 保持类动作），暂停任务”
                    （修复前会立即 ABORT）
恢复桥接 + 请求 OFFBOARD
  任务层         ✅ “检测到已交还控制，恢复自主：回到 CRUISE” → “飞往航点 1”
  后续           ✅ “到达航点 1，位置误差 0.46 m” → “飞往航点 2”（任务继续推进）
取证            logs/2026-10-08/{fs7_mission,fs7_bridge,fs7_manual}_failsafe_pause.log
```

### 修复

- 首次实现时只在飞行阶段加了动作类型判断，**PAUSED 分支仍保留旧的“任何 failsafe 都中止”**，
  导致暂停 0.2 s 后立刻中止（日志：`任务中止：暂停期间 PX4 进入 failsafe`）。
  已在两处统一为同一判断。教训：改语义时要搜索全部同类判断点。
- 早前一轮 `RECOVERY_NAV_STATES` 使用了未导入的 `VehicleStatus`，导致任务节点启动即崩；
  已补 `from px4_msgs.msg import VehicleStatus`（说明新增模块级常量后需冒烟启动一次）。

---

## [2026-10-09] offboard 丢失行为验证通过（阶段 2「Offboard loss」验收项）

### 验证结果

保持手动源在线（`manual_control.py --script "hover:70"` 只发中位摇杆、不接管），
飞行中杀掉 offboard_bridge：

```text
PX4 判据    offboard_control_signal_lost = True，manual_control_signal_lost = False
PX4 动作    ✅ "Failsafe activated" → navigation mode: Position（位置保持，非返航）
            即 COM_OBL_RC_ACT=0 的预期行为
任务层      ✅ 检测到 PX4 failsafe 并中止任务（当前设计：failsafe → ABORT）
```

对照实验（手动源也丢失时）此前观测到的是 RTL——现在两条路径都解释清楚了：

- **offboard 丢失 + 手动源在线** → Position 保持（本次验证）
- **手动源丢失**（`NAV_RCL_ACT` 默认返航）→ Hold 5 s → RTL → 落地 → 上锁（本次同时观测到，
  属安全正面行为：手动源消失后自动返航）

### 修复

- `manual_control.py` 增加 `sys.stdout.reconfigure(line_buffering=True)`：
  此前脚本模式输出被块缓冲，进程被 SIGTERM 终止时日志文件为空，导致"看不出工具是否在运行"。

### 已知问题 / 待决策

- 任务层目前对**任何** failsafe 都 ABORT。而 offboard 丢失的动作是良性的"位置保持"，
  更合适的语义是 **PAUSED**（等操作手恢复链路或接管，然后续飞）。
  建议加参数区分：failsafe 动作是保持类 → PAUSED；返航/降落类 → ABORT。**待确认后实现**。
- 实验期间为排除干扰禁用了电池阈值（`BAT_LOW/CRIT/EMERGEN_THR=0`，仅内存生效）。
  **手动飞行前请恢复**：`BAT_LOW_THR 0.15`、`BAT_CRIT_THR 0.07`、`BAT_EMERGEN_THR 0.05`。

---

## [2026-10-09] offboard 丢失注入：定位到真正的 failsafe 来源

### 分析结论（本轮的关键发现）

用 `listener failsafe_flags` 直接读 PX4 的 failsafe 判据，修正了此前的误判：

```text
offboard_control_signal_lost : True    ← offboard 丢失确实被检测到
manual_control_signal_lost   : True    ← 但同时没有任何手动输入源
battery_warning              : 0       ← 电池不是原因（阈值已禁用）
remote_id_unhealthy          : True    ← 与本实验无关的噪声项
```

因此观察到的 RTL 来自**数传/手动控制丢失 failsafe**（`NAV_RCL_ACT` 默认返航），
它**掩盖**了 offboard 丢失动作（`COM_OBL_RC_ACT=0` 应为位置保持）。
根因：本环境没有真实遥控器，实验时键盘工具也未在发送摇杆流，
PX4 同时判定"offboard 丢失 + 手动源丢失"→ 升级为返航。

### 环境结论

- `SIM_BAT_*` 等仿真参数**不会被 `param save` 持久化**，每次 PX4 启动后需重新设置；
  本轮改为实时设置 `BAT_LOW_THR=0`、`BAT_CRIT_THR=0`、`BAT_EMERGEN_THR=0` 后，
  `battery_warning` 归零，电池 failsafe 不再干扰。
- 禁用电池阈值后，`offboard_control_signal_lost` 可被干净观测，实验可重复。

### 下一步（已明确）

飞行中先用 `manual_control.py` 持续发送**中位摇杆**（不接管）保持"手动源在线"，
再杀掉 offboard_bridge，即可单独观测 `COM_OBL_RC_ACT` 的动作（预期位置保持），
同时验证任务层"离开 OFFBOARD → PAUSED"与键盘随时接管。

---

## [2026-10-09] 交还前置检查 + offboard 丢失故障注入（部分完成）

### 新增

- `manual_control.py` 交还前置检查：切换 OFFBOARD 前扫描 `/proc` 确认 `offboard_bridge` 进程在线
  （**不依赖 ROS 2**）；若不在线则拒绝交还、保持人工模式并给出提示。
  局限：进程存在 ≠ 链路健康（agent 挂掉时进程仍在），因此仍以"模式是否真正切换成功"为准。

### 验证：offboard 丢失注入（飞行中）

```text
注入方式        任务处于 CRUISE 时杀掉 uXRCE-DDS Agent
ROS 2 侧        ✅ offboard_bridge 检测到 PX4 状态超时（>1 s）并**主动停止发送设定点**
                ✅ waypoint_mission 进入“等待 PX4 状态”，不再下发指令
PX4 侧          ✅ 触发 failsafe 并安全收尾：Failsafe activated → RTL → Landing detected
                   → Disarmed by landing（未失控、未飞远）
取证            logs/2026-10-08/{bridge_offboard_loss_injection.log,
                mission_offboard_loss_injection.log}
```

### 已知问题

- **本轮触发的其实是电池 failsafe，不是 offboard 丢失 failsafe**，因此 `COM_OBL_RC_ACT=0`
  （offboard 丢失 → 位置保持）的行为**尚未验证**。虽然安全收尾成立，但结论不能混用。
- 电池 failsafe 反复干扰实验：`COM_LOW_BAT_ACT=0` 只关掉低电量告警动作，危急阈值仍会 RTL。
  已设置 `SIM_BAT_MIN_PCT=100`、`SIM_BAT_DRAIN=0` 并保存，**需重启 PX4 后生效**；
  下一轮实验前务必确认电池状态，否则同样会污染结果。

---

## [2026-10-09] 隔离验证通过 + 手动接管链路启动自动化

### 新增

- `scripts/enable_manual_link.sh`：把键盘接管所需的专用 MAVLink 实例注入 PX4 的**构建产物**
  `etc/init.d-posix/rcS`（幂等）。`scripts/run_px4_sitl.sh` 启动前自动调用，
  因此**重建 PX4 后也会自动补上**，且不修改 PX4 源码（submodule 保持干净）。
  实测启动日志出现 `mode: Onboard, data rate: 1000000 B/s on udp port 14700 remote port 14701`。
- `manual_control.py` 脚本模式新增周期位置打印（每 2 s 输出位置与模式），使证据自包含。

### 验证：隔离测试（方案 A 的核心主张）

```text
前置          遥控/键盘链路已接管，任务处于 CRUISE（期间任务已记录“判定为人工接管，暂停任务”）
终止          杀掉 offboard_bridge、waypoint_mission、MicroXRCEAgent（pgrep 确认全部消失）
结果          手动控制继续生效约 13 s：位置样本 N(北) +0.36 → +8.26 m（约 8 m），
              模式全程 POSCTL(3)；总位移 Δn=+10.50 Δe=+9.79 m
结论          ✅ 键盘手动接管**不依赖 ROS 2**，符合 README §12 的安全架构要求
取证          logs/2026-10-08/{manual_control_isolation_verified.log,
              mission_takeover_isolation.log, bridge_isolation.log, *.ulg}
```

### 已知问题

- 交还（切回 OFFBOARD）需要 offboard 数据源在线：桥接被杀后 PX4 拒绝切换并打印
  `Switching to Offboard is currently not available`。属安全正面行为，但流程上要求
  **交还前先恢复 offboard_bridge**；下一步在 `manual_control.py` 的交还流程里加提示或前置检查。
- 电池 failsafe 会持续干扰模式切换实验：本次虽已设 `COM_LOW_BAT_ACT=0`、`SIM_BAT_DRAIN=0`，
  但本会话电池已耗尽，仍触发 Hold→RTL。**建议做接管类实验前重启 PX4 让电池状态归零**。
- 交互式键盘（F9/F10 与 WASD/方向键）仍待用户实机敲键验证；脚本化路径已验证。

---

## [2026-10-08] Phase B 起步：键盘人工接管 + 任务暂停/恢复（已验证）

### 新增

- `scripts/manual_control.py`：键盘手动接管工具（**方案 A：键盘 → MAVLink → PX4，不经过 ROS 2**）。
  - 按键：W/S 油门升降、A/D 偏航、方向键俯仰/横滚；F9 接管（POSCTL）、F10 交还（OFFBOARD）；
    死区/幅度系数 `--scale` 可配；**松键即回中**。
  - 交互式（终端 raw 模式，供手动飞行）与脚本化（`--script`，供自动化验证）共用同一套映射；
    脚本模式会打印本段位移（NED）便于量化验证。
- `waypoint_mission` 新增 **PAUSED 状态**：飞行中检测到离开 OFFBOARD 判定为人工接管，
  暂停任务并保留当前航点；交还后回到原阶段/原航点继续。参数 `pause_timeout_s`（默认 120 s）超时则中止。

### 验证（飞行实测）

```text
任务进入 CRUISE            ✅
人工接管                   ✅ custom_mode OFFBOARD(393216) → POSCTL(196608)，"接管成功"
摇杆前飞 0.6×4 s           ✅ 位移 Δn=+1.23 m（pitch 前进）
任务侧响应                 ✅ "检测到离开 OFFBOARD（nav_state=2），判定为人工接管，暂停任务"
交还控制                   ✅ 模式回到 OFFBOARD(393216)，"交还成功"
任务恢复                   ✅ "检测到已交还控制，恢复自主：回到 CRUISE" → "飞往航点 0"
取证                       logs/2026-10-08/mission_takeover_resume.log 等
```

### 已知问题

- **隔离验证（接管期间杀掉全部 ROS 2 节点）尚未成立**：第一次尝试的 `kill` 打到了 `ros2 run`
  包装进程而非真实节点，桥接仍在运行；第二次尝试因命令中的 `pkill` 模式匹配到自身而中断。
  该项是方案 A 的核心价值主张，**必须在下一轮重做并留下证据**。
- PX4 在接受进入 POSCTL 前**需要持续的手动输入流**：原先"接管后才开始发 MANUAL_CONTROL"会被拒绝，
  已改为始终流式发送（未接管时摇杆中位）。
- 若 offboard 数据源已不存在，PX4 会拒绝切回 OFFBOARD（日志 `Switching to Offboard is currently not available`），
  交还会失败——属安全正面行为，需在流程上保证交还前桥接在线。
- 长会话后电池进入 failsafe（Hold → RTL → Land），会干扰模式切换实验；
  做接管实验前应确认电池状态，或改用参数放宽电池 failsafe 阈值。

---

## [2026-10-08] 人工接管通道验证（方案 A：键盘 → MAVLink → PX4）

### 新增（本轮的通道验证与结论）

- 确认可行的 MAVLink 手动控制通道：**在 PX4 侧为键盘客户端专开一个实例**

```text
PX4 控制台：mavlink start -u 14700 -o 14701 -r 1000000 -m onboard
客户端：绑定 udpin 14701（收流），显式发往 127.0.0.1:14700
```

  实测可收到 100 Hz 的 ATTITUDE/HIGHRES_IMU/LOCAL_POSITION_NED 遥测流；
  请求 `DO_SET_MODE → POSCTL` 得到 `COMMAND_ACK result=0`（接受）。

### 已知问题（踩坑记录）

- PX4 自带的 GCS 实例（UDP 18570，remote 14550）**不适合作为客户端入口**：
  它只更新对端地址、固定把流发往 14550，实测绑定 14550 收不到任何包，
  同一端点的连通性在不同次运行间也不一致（`mavlink status` 显示我方心跳被计入
  “sysid:245 compid:190, lost:508”，但流不回传）。专开实例后问题消失。
- 该实例**必须每次 PX4 重启后重新建立**（写在 pxh 里），尚未自动化，属待办。
- `MAV_TYPE_JOYSTICK` 在当前 pymavlink 方言中不存在，心跳类型用 `MAV_TYPE_GCS`。
- PX4 的 `custom_mode` 编码为 `(主模式 << 16) | (子模式 << 24)`：3=POSCTL(196608)、
  6=OFFBOARD(393216)，判读模式时需按此换算。

### 待确认（开发前需拍板）

- 人工接管期间任务层的行为：现有 `waypoint_mission` 遇到“离开 OFFBOARD”直接 ABORT，
  而路线图要求“手动 → 自主恢复”。需要在“中止”与“暂停并在交还后恢复”之间选定。

---

## [2026-10-08] 降落根因定位：EKF 触地后发散（附可用收尾方案）

### 新增

- `scripts/analyze_landing.py`：解析 `.ulg`，切出 AUTO_LAND 段并统计高度、实际下沉速率、
  垂速分布、姿态倾角、推力与落地检测各标志的翻转情况。
- `scripts/flight_check.py` 新增 `landing` 子命令：AUTO_LAND 期间同时记录
  PX4 估计（z/vz/标志/armed）与 AirSim 真值 z，用于对照。
- `offboard_bridge` 新增 `land_autodisarm_s`（默认 10 s）：AUTO_LAND 后若落地检测未置位，
  静置到时间即强制上锁，使任务能正常收尾。

### 分析结论（.ulg + 真值对照）

对 `logs/2026-09-19/08_27_49.ulg`（63.2 s AUTO_LAND 段）与 2026-10-08 的两次实时对照：

```text
真值 z              AUTO_LAND 后约 4 s 即稳定在 0.68 m（飞机已实际触地并静止）
PX4 估计 z          继续漂移到 +1.7 ~ +1.8 m，跨度 3.3 ~ 3.6 m
PX4 估计 vz         全程约 +1.3 ~ +1.4 m/s（谎报“高速下降”），与静止真值矛盾
落地检测            close_to_ground 全程为真，但 ground_contact/at_rest 仅偶发，
                    landed 从未置位
推力                悬停估计 0.454，AUTO_LAND 期间推力指令均值 -0.339（确实在命令下降）
```

**根因**：飞机实际已落地，但 EKF 的高度/垂速估计在触地后发散，
落地检测基于该发散的速度估计判断“是否静止”，因此永远无法锁定。
这不是落地检测阈值问题，也不是降落控制器问题。

排除项：将 `EKF2_BARO_CTRL` 从 1 改为 0（关闭气压融合，高度源保持 GPS）后发散现象完全一致，
说明**不是气压计数据导致**（实验后已恢复 `EKF2_BARO_CTRL=1`）。
推测与 AirSim 在触地状态下给出的加速度/速度与静止状态不一致有关（待进一步验证）。

### 验证

```text
单元测试            35 passed
收尾方案            AUTO_LAND 后 10 s 强制上锁：实测 t=10.3 s 时 armed=False，
                    任务可正常进入 WAIT_DISARM → DONE
                    （真值显示 4 s 已触地，10 s 静置时间充裕）
取证                logs/2026-10-08/{bridge_land_autodisarm.log, *.ulg}
```

### 已知问题

- 该收尾方案是**面向本仿真环境的工程兜底**：按“下发降落 + 静置时间”判定，
  不是真实触地判据，实机不可直接沿用；实机应依赖可靠的落地检测或新增测距传感器。
- EKF 触地发散的根本原因未彻底定位（已排除气压融合），需要对比 AirSim 送入的
  IMU/速度数据与静止状态的关系才能进一步确认。

---

## [2026-10-08] 接近终止策略移到任务层 + 诊断脚本合并

### 新增

- `mission/approach_policy.py`：接近终止策略（纯函数 `evaluate_approach`），
  输入目标深度与已用时间，输出 `continue / hold / timeout`。
- `mission/approach_supervisor.py` 节点：订阅 `/vlm_navigator/decision`，
  按策略在到达站定距离时向 offboard_bridge 下发 `hold`，并发布 `~/status`。
  安全边界：仅在 PX4 已解锁且处于 OFFBOARD 时干预，不接触 `/fmu/*`。
- `vlm_navigator` 新增 `decision_topic`（默认 `~/decision`），把指向结果与生成的机体速度
  发布出去供任务层决策；`stop_distance_m` 默认从 0.35 m 放宽到 0.8 m，退化为**兜底**。
- `scripts/flight_check.py`：把 `compare_altitude.py` 与 `test_vertical_command.py`
  合并为带子命令的诊断入口（`altitude` / `vertical`），原两个脚本删除。

### 变更

- 分层调整：**是否继续接近、何时停下**由任务层决定，视觉层不再承担任务策略。
- `scripts/test_vlm_pipeline.sh` 支持 `WITH_SUPERVISOR=1`、`STANDOFF_M=<米>`。

### 验证

```text
单元测试            35 passed（新增接近策略 5 例）；按约定保留 px4_offboard 模板测试
接近终止飞行        vision_stub 实时图像，站定距离 1.2 m
  监督节点日志      停止接近并保持位置：已接近到 0.46 m（阈值 1.20 m）
  深度范围          0.46 ~ 0.72 m（9 次决策）
  位移              20 s 内仅约 0.25 m（此前无终止策略时约 12 m）
取证                logs/2026-10-08/{vlm_navigator_approach.log,
                    approach_supervisor.log, bridge_approach.log, *.ulg}
```

### 已知问题

- 首次运行时 `vlm_navigator` 启动即崩：决策发布用到的 `std_msgs/String` 未导入
  （`NameError: name 'String' is not defined`），已修复；说明新增发布通道后需要冒烟启动一次。
- 视觉层兜底阈值（0.8 m）与任务层站定距离（默认 0.8 m）目前取值相同，
  若任务层配置更大的站定距离，需注意兜底阈值应小于等于任务层阈值，否则视觉层会先停。

---

## [2026-09-23] 竖直指令符号验证 + 大偏角收敛验证通过

### 新增

- `scripts/test_vertical_command.py`：只发上向/下向速度指令，同时记录 PX4 与 AirSim 真值高度，
  用于检查速度指令链路的符号方向。

### 验证

```text
竖直指令符号     上向 +0.50 m/s × 6 s → PX4 Δz=-2.761 m（爬升），真值 Δz=-2.691 m ✅
                 下向 -0.50 m/s × 6 s → PX4 Δz=+2.447 m（下降），真值 Δz=+1.628 m ✅
                 结论：ENU 速度 → offboard_bridge → PX4 NED 的符号链路正确

大偏角收敛       出发点目标像素 x=1207（画面最右），完整过程：
                   1207 → 偏航 -0.36 rad/s（转向修正）
                    806 → -0.13（偏差缩小，指令衰减）
                    473 → +0.13（小幅过冲，反向修正）
                    533 →  0.00（进入 10° 死区，停止修正）
                    634/631/625/623/623 → 全部 0.00（锁定，同时前飞接近，深度 0.59→0.46）
                 后 6/9 个采样点零偏航，目标稳定在画面中心 ±10 px
```

结论：竖直指令方向正确；此前的“方向不一致”是**误读单点日志**——
那一轮 20 s 内竖直指令正负交替，净效果才是下降。
大偏角收敛（先转过去、再稳住）已完整验证。

### 已知问题

- **停止距离阈值偏小**：阈值 0.35 m 大于本轮观测到的最近深度 0.46 m，因此未触发，
  飞机在 20 s 内又飞出约 12 m。实际使用应把阈值提到 0.5–1.0 m，或由任务层限定“接近”行为。
- 长会话中电池降到阈值触发 failsafe，PX4 自动执行 RTL → 落地 → 上锁（日志可见
  `Failsafe activated` → `RTL` → `Landing detected` → `Disarmed by landing`）。
  说明 AUTO_LAND 链路本身可用，只是下降极慢（与此前观测一致）。

---

## [2026-09-21] 高度真值对比结果：PX4 估计与仿真真值一致

### 验证

`scripts/compare_altitude.py`（首次跑通）——起飞到 3 m 并悬停 20 s，
同时记录 PX4 `vehicle_local_position_v1` 的 z 与 AirSim `simGetVehiclePose` 的真值 z：

```text
爬升段变化量        PX4 Δz = -2.990 m，AirSim Δz = -2.995 m，差值 +0.005 m（5 mm）
悬停段两者偏差      全程 ≤ 0.02 m（例如 t=18.0 s：PX4 -2.995，真值 -3.008）
PX4 vz              悬停稳定在 ±0.01 m/s
```

**结论（修正此前判断）**：在正常飞行段，PX4 的高度估计与仿真真值一致到毫米级，
不是“高度估计不可用”。此前观测到的 z 从 -3 漂到 +2 只出现在 **AUTO_LAND / 近地阶段**，
因此“降落异常”应定位到近地与落地检测环节，而不是全程的估计器问题。

### 已知问题

- **不要探测 4560 端口**：AirSim 的 PX4 仿真链路只接受一个连接，
  从 WSL 用 `/dev/tcp` 探测会占掉该连接槽，导致随后启动的 PX4 一直停在
  “Waiting for simulator to accept connection on TCP port 4560”。
  本次因此白重启了一次 UE4。要判断 PX4 是否连上，看 PX4 自己的日志即可。
- 仍需验证：VLM 闭环运行中出现“机体上向指令为 +0.11 m/s，PX4 高度却下降 0.63 m”。
  既然高度估计可信，就需要单独做一次纯竖直速度指令测试，检查指令链路的符号方向。

---

## [2026-09-21] 高度真值对比脚本 + AirSim reset 的副作用（需要重启仿真）

### 新增

- `scripts/compare_altitude.py`：同时采样 PX4 高度估计（`vehicle_local_position_v1`）
  与 AirSim 真值位姿（RPC `simGetVehiclePose`），输出逐点对比表与期间变化量差值，
  用于核查“指令方向与高度变化不一致”以及 EKF 高度漂移。**尚未跑通验证**（见下）。

### 已知问题

- **AirSim `reset()` 会破坏 PX4 的仿真链路**：本机为 `PX4Multirotor` + `LockStep`，
  reset 后飞机位姿确实回到原点（实测 (31.56, 30.49) → (0, 0)），
  但 PX4 连接的 TCP 4560 不再监听，`reset()` 与 `simPause(False)` 均无法恢复，
  PX4 一直停在 “Waiting for simulator to accept connection on TCP port 4560”。
  必须重启 UE4 仿真才能恢复。
  结论：**不要在与 PX4 会话共存时调用 `reset()`**；需要复位位姿就重启仿真。

---

## [2026-09-21] 测量基础修复 + 接近停止保护

### 新增

- `scripts/px4_state_snapshot.py`：等待 `xy_valid/z_valid` 与状态消息齐备后输出一行快照。
  替换测试脚本里的 `ros2 topic echo --once`（此前连续读到 x=y=z=0.00，位移测量不可信）。
- `vlm/pointing.py` 新增 `stop_distance_m`：目标深度小于阈值时**停止平移**（仍允许偏航修正），
  `vlm_navigator` 新增同名参数（默认 0.35 m）。对应 SeePointFly adaptive 模式“太近就不再前进”的做法。
- `scripts/test_vlm_pipeline.sh` 支持 `PRE_ROTATE_S`：起飞后先用速度指令预旋转，制造初始偏角。

### 验证

```text
单元测试           30 passed（新增停止距离 3 例）
位置测量修复       ✅ 起飞后 x(N)=-0.02 y(E)=-0.02 z(D)=-2.85 armed=True nav=14
                    （同一脚本此前一直读到 0.00）
接近停止保护       ✅ 单元测试覆盖：深度 ≤ 阈值 → 平移归零；深度 > 阈值 → 正常前飞
飞行表现对比       整定前单次运行位移约 12 m 且高度冲到 4.38 m；
                    本轮 20 s 位移 2.44 m（深度 0.9–2.0 m，未触及 0.35 m 阈值）
取证               logs/2026-09-21/{vlm_navigator_console_stopguard.log,
                   bridge_console_stopguard.log, camera_close_to_target.jpg}
```

### 已知问题

- **停止距离保护尚未在飞行中触发**：本轮起飞点在目标上方 3 m，深度估计 0.9–2.0 m，
  始终高于 0.35 m 阈值。需要在“贴近目标”的初始状态下再跑一轮才能验证空中行为。
- **大偏角起步验证未成立**：`PRE_ROTATE_S=0.6`（约 34°）后目标仍居中（像素 628–709），
  未落到死区外，因此没有验证“先转过去再稳定”的过程；且目标一旦出画就没有指向来源，
  会停在原地（与 SeePointFly 行为一致），这是下一步需要专门设计的测试。
- 该轮飞行中“机体上向速度指令为正，但高度 z 从 -2.85 变到 -2.22（下降 0.63 m）”，
  方向一致性存疑，需要单独核查（可能是深度估计变化导致指令方向随之翻转）。

---

## [2026-09-21] 偏航死区整定（闭环收敛）

### 变更

- `vlm/pointing.py` 的 `pointing_to_body_velocity` 新增 `yaw_deadband_rad`：
  横向偏角小于死区时不转向（借鉴 SeePointFly 约 10° 的阈值做法，做成可配参数）。
- `vlm_navigator` 新增 `yaw_deadband_deg` 参数（默认 10.0），可叠加上限幅与增益一起整定。
- `scripts/test_vlm_pipeline.sh` 支持把额外参数透传给 navigator（`"${@:4}"`），便于调参复跑。

### 验证

```text
整定参数          yaw_gain=0.5、max_yaw_rate=0.4、yaw_deadband_deg=10、base_velocity=0.8
单元测试          27 passed（新增死区/增益用例 3 个）
目标像素轨迹      629 → 629 → 629 → 631 → 632 → 631 → 629 → 631 → 631
相对画面中心      |x-640| 平均 10 px，中心带（±150 px）占比 100%
偏航指令          9/9 次为零（死区生效，无来回修正）
接近过程          深度 0.46 → 0.26 m 单调下降（保持目标居中并前飞接近）
对照（整定前）    像素 x 在 162 ~ 1212 间大幅摆动，偏航在 ±0.50 rad/s 反复饱和
取证              logs/2026-09-21/{vlm_navigator_console_tuned.log, bridge_console_tuned.log}
```

### 已知问题

- 本次整定运行的起始朝向已基本对准目标，验证的是“目标居中时不来回修正”；
  从大偏角起步的收敛过程（先转过去、再稳定）仍需单独跑一轮确认。
- 位置回读用 `ros2 topic echo --once` 仍返回 0.00，需改为连续采样才能量化高度与位移。

---

## [2026-09-21] 实时图像闭环验证（vision_stub）

### 新增

- `vlm/vision_stub.py`：确定性视觉桩（provider `vision_stub`），用颜色检测（默认橙色目标）替代 VLM，
  返回与真实 VLM 完全相同的 JSON 指向格式，用于在没有 API key 时验证**闭环动力学**（测试替身，非感知模块）。
- `scripts/test_vlm_pipeline.sh` 支持 provider 与实时图像话题：
  `bash scripts/test_vlm_pipeline.sh "" vision_stub /airsim_camera/image`。

### 修复

- 脚本传 `-p image_file:=""` 会被 rcl 拒绝（`Couldn't parse parameter override rule`），
  导致 `vlm_navigator` 未启动、日志为空；实时图像模式下改为不传该参数。
- `vision_stub.locate_color` 原为纯 Python 逐像素扫描，1280×720 耗时约 80 s；改 numpy 向量化后约 0.2 s。

### 验证

```text
链路             vlm_navigator(vision_stub, /airsim_camera/image) → offboard_bridge → PX4
指向随图像变化    ✅ 像素 x: 1212 → 802 → 162 → 350 → 1176 → 1187 → 304 → 170 → 1052
偏航随之变化      ✅ -0.50 → -0.25 → +0.50 → +0.42 → -0.50 …（上限 0.5 rad/s）
桥接模式切换      ✅ position → velocity，ENU 速度 (0.73, 0.69, 0.06) m/s
PX4 侧            ✅ Armed by external command、Takeoff detected
取证              logs/2026-09-21/{vlm_navigator_console.log, bridge_console.log,
                  vlm_decisions/decision_20260921_*.json}
```

结论：指向→几何→机体速度→PX4 的**闭环**已跑通，偏航指令随目标在画面中的位置变化而变化，
不再像静态图像那样单向转圈。

### 已知问题

- **偏航震荡**（1212→802→162→350 反复）：纯比例控制且无死区，叠加机体前飞导致过冲。
  下一步参照 SeePointFly 加横向偏差死区（约 10°）并降低增益。
- 测试脚本用 `ros2 topic echo --once` 回读位置，本次两次都取到 0.00（疑似无效样本）；
  PX4 日志已确认解锁与起飞，精确高度需改用连续采样确认。
- `vlm_navigator` 被 SIGTERM 结束时会抛 `ExternalShutdownException`，属退出路径噪声。

---

## [2026-09-21] 相机接入验证通过（sensor_bridge）

### 变更

- `airsim_camera` 新增 `depth_interval` 参数（默认 5）：彩色图每帧抓取，深度图按间隔抽取。
  原因：LockStep 下每帧同时抓 1280×720 彩色 + 640×480 深度会把帧率拖到约 0.3 Hz。
- `msgpackrpc` 兼容补丁（本机 site-packages，非仓库内容）：
  `~/.local/lib/python3.10/site-packages/msgpackrpc/transport/tcp.py` 中
  `msgpack.Packer(encoding=...)` / `msgpack.Unpacker(encoding=...)` 改为
  `msgpack.Packer(...)` / `msgpack.Unpacker(raw=False)`，以适配 msgpack ≥ 1.0。
  不改则报 `UnicodeDecodeError: 'utf-8' codec can't decode byte 0x94`（图像二进制被当字符串解码）。

### 验证

```text
AirSim RPC 连接                   ✅ 172.21.192.1:45000（ApiServerPort 非默认 41451）
彩色图分辨率                       ✅ 1280×720（新相机配置已生效；原为默认 256×144）
捕获帧内容                         ✅ logs/2026-09-21/camera_frame.jpg（大块墙体/天空/橙色球，画面正常）
彩色图频率                         ✅ 39 帧 / 20 s = 2.04 Hz（配置 2 Hz，平均间隔 0.49 s）
深度图频率                         ✅ 8 帧 / 20 s = 0.4 Hz（depth_interval=5）
发布话题                           ✅ /airsim_camera/{image(bgr8), depth(32FC1), camera_info}
```

### 已知问题

- 深度图在 LockStep 下仍较慢（0.4 Hz），做实时避障时需要更快的深度来源
  （AirSim DepthVis/LiDAR 或降低分辨率），或改为独立节点并放宽间隔。
- `msgpackrpc` 补丁位于 site-packages，重装该包会被覆盖；若后续报同样的 UnicodeDecodeError，
  按本节说明重新打补丁即可。

---

## [2026-09-19] 传感器接入（sensor_bridge，未完成验证）

### 新增

- `ros2_px4_ws/src/sensor_bridge`（新包 0.1.0）：`airsim_camera` 节点，
  把 AirSim 相机图像发布为 ROS 2 话题（`~/image` bgr8、`~/depth` 32FC1、`~/camera_info`）。
  默认 `host=127.0.0.1`、`port=45000`（本机 settings.json 的 ApiServerPort）、`vehicle_name=Drone1`。
- Windows 侧 `C:\Users\steve\Documents\AirSim\settings.json` 新增 `Cameras` 段：
  Scene 1280×720、DepthPlanar 640×480、DepthVis 640×480（原配置没有相机段，用的是 256×144 默认相机）。
  备份：`settings.json.bak-20260919`。**需重启 AirSim 生效。**

### 环境（本机一次性配置）

```text
pip3 install --user msgpack==0.6.2 msgpack-rpc-python==0.4.1 tornado==4.5.3
pip3 install --user --no-deps airsim        # 得到 airsim 1.8.1 客户端
```

坑：`msgpack-rpc-python 0.4.1` 仍在使用 `msgpack.Packer(encoding=...)`，而 msgpack ≥ 1.0
已移除该参数，报错为 `__init__() got an unexpected keyword argument 'encoding'`；
必须把 msgpack 钉在 0.6.x。另：`pip install airsim` 会先在元数据阶段导入自身而失败，
需先装好 msgpack/msgpackrpc 再 `--no-deps` 安装。

### 验证（部分）

```text
colcon build --symlink-install --packages-select sensor_bridge     ✅ 1 分 0 秒
ros2 topic list                                                     ✅ /airsim_camera/{image,depth,camera_info} 均出现
端到端抓帧                                                          ⏳ 未完成：探测 172.21.192.1:45000 与 :4560 均不可达，
                                                                   说明 AirSim 仿真当前未运行，需启动后重测
```

---

## [2026-09-19] 降落问题定位与快速降落通道

### 新增

- `scripts/landing_diagnose.py`：降落诊断脚本，自动完成“起飞→命令降落→逐周期记录
  高度/垂速/落地检测标志”，并计算**仿真时间倍率**（PX4 时间戳推进 ÷ 墙钟时间）。
- `offboard_bridge` 新增 `land_fast` 指令：按“已知高度 + 恒定下降速度 + 时间”执行速度模式下降，
  时间到即强制上锁（参数 `land_descend_speed_mps` 默认 0.5、`land_duration_margin_s` 默认 2.0）。

### 变更

- `scripts/landing_diagnose.py` 默认改用 `land_fast`，降落超时从 60 s 收紧到 30 s。

### 验证

```text
仿真时间倍率(RTF)            0.999 / 1.027（两次测量）→ 仿真基本实时，排除“仿真慢于墙钟”的猜测
AUTO_LAND 实测               3 m 下降到落地约 80 s（≈0.04 m/s），期间 z 估计在 -2.8 ~ +2.0 之间漂移
AUTO_LAND 最终结果           PX4 日志出现 “Landing detected” → “Disarmed by landing”（确实能落地，只是极慢）
land_fast 实测               起始 2.63 m，0.5 m/s 下降，7.3 s 后强制上锁，armed=False
取证                         logs/2026-09-19/{landing_trace_autoland.csv, landing_trace_fast.csv, *.ulg}
```

### 已知问题

- **根因是高度估计不稳定，而非落地检测本身**：`EKF2_HGT_REF=1`（GPS）下 z 估计仍大幅漂移，
  AUTO_LAND 的下降速率被拖到约 0.04 m/s。真正的修法是解决 AirSim 传感器（GPS/气压）与 EKF 的一致性，
  本轮先用 `land_fast` 绕开，尚未定位到传感器层面的根因。
- **`land_fast` 是时间驱动的兜底**：按时间而非真实触地判定，可能略高于地面就上锁，
  因此 `vehicle_land_detected.landed` 仍为 false（对 SITL 可接受，实机不可用）。
- 航段速度 0.3 m/s 偏慢的现象与 RTF 无关，怀疑是位置控制器参数/轨迹生成限制，待后续单独排查。

---

## [2026-09-19] VLM 指向式导航（借鉴 See, Point, Fly）

### 新增

- `ros2_px4_ws/src/vlm`（新包 0.1.0）——图像指向式导航层：
  - `pointing.py`：VLM 指向结果解析（兼容 `[{"point": [y, x], "depth": d, "label": ...}]`，
    容忍 markdown 包裹与前后解释文字）、深度等级→米映射、通用针孔反投影、
    “指向→机体 FLU 速度+偏航角速度”换算（含速度/偏航限幅）。纯标准库实现，可离线单元测试。
  - `vlm_client.py`：provider 抽象（`mock` / `gemini` / `openai`），仅用标准库 urllib 发 HTTPS，
    图像编码走 PIL（规避本机 cv2 与 numpy 2.x 的 ABI 冲突），密钥只从环境变量读取。
  - `vlm_navigator.py`：节点，负责图像→VLM→指向→机体速度→ENU 速度指令；
    安全边界：仅在 PX4 已解锁、处于 OFFBOARD、未 failsafe 时发布，指令超时即停止发布。
  - `test/test_pointing.py`：10 个单元测试（解析容错、深度映射、反投影、限幅、偏航符号）。
  - `test/data/test_scene.jpg`：离线验证用静态图像。
- `scripts/test_vlm_pipeline.sh`：VLM 管道端到端验证脚本（mock provider + 静态图像）。
- `docs/reference-see-point-fly.md`：参考实现记录（含许可说明、借鉴清单、有意差异）。
- `px4_interface` 新增**速度控制模式**：`~/velocity_setpoint`（`geometry_msgs/Twist`，
  ENU 世界速度 + 偏航角速度）；`frames.py` 新增 `body_flu_to_enu` 与 `flip_yaw_rate_sign`。

### 变更

- `scripts/env.sh`：修复在 `set -u` / `set -e -o pipefail` 调用下必然失败的问题
  （ROS 2 `setup.bash` 引用未定义变量；沙箱内 `ip route` 返回非零）。此前所有
  `run_*.sh` / `build_*.sh` 在该场景下都会中断。

### 验证

- 单元测试：`px4_interface` 14 项 + `vlm` 10 项，共 24 passed。
- 编译：`colcon build --symlink-install --packages-select px4_interface vlm`（1 分 3 秒）。
- 端到端飞行（AirSim + PX4 SITL + uXRCE-DDS + ROS 2，mock provider + 静态图像，真实发布速度指令）：

```text
起飞后位置                    x(N)=0.01  y(E)=0.04  z(D)=-2.99
VLM 指向                      像素 (396,240)，深度 1.00 m
机体速度                      前 0.97、左 -0.23 m/s，偏航 -0.24 rad/s
桥接节点                      ENU (0.33, 0.94, 0.00) m/s，模式切换 position → velocity
VLM 阶段结束后位置             x(N)=-5.34 y(E)=3.83 z(D)=-2.92（实际位移约 6.5 m）
取证                          logs/2026-09-19/{06_01_50.ulg, bridge_console.log,
                              vlm_navigator_console.log, vlm_decisions/*.json}
```

结论：图像 → 指向 → 几何反投影 → 机体速度 → PX4 的整条链路已打通并驱动机体运动。

### 已知问题

- **静态图像下不会收敛**：目标点固定在图像中不动，偏航指令持续存在，机体因此转圈
  （本次实测位移方向与持续转向一致）。闭环收敛需要真实相机图像，属下一步 `sensor_bridge`。
- **真实 VLM 未验证**：本机无 API key，`gemini` / `openai` 两个 provider 只做了接口实现与
  mock 验证，尚未发起过真实请求。
- **许可风险已规避但需留意**：参考仓库为 Proprietary（保留所有权利），本项目只借鉴思路、
  自写代码与提示词；若后续要复制其代码或提示词，需先取得作者授权
  （详见 `docs/reference-see-point-fly.md`）。

---

## [2026-09-16] 阶段 A：Offboard 桥接与航点任务（dev 分支）

### 新增

- `ros2_px4_ws/src/px4_interface`（新包 0.1.0）——项目中唯一直接收发 PX4 `/fmu/*` 的接口层：
  - `offboard_bridge` 节点：20 Hz 位置设定点流；接收 ENU 目标点（`geometry_msgs/PoseStamped`）
    并转换为 PX4 的 NED；接收文本指令 `arm` / `disarm` / `disarm_force` / `offboard` / `land` / `hold`；
    看门狗：PX4 状态超时即停止发送设定点（交还 PX4 failsafe），目标点 2 s 未刷新即保持当前位置。
  - `frames.py`：NED↔ENU 位置、偏航换算与距离函数（全项目坐标系约定的唯一实现处）。
  - `qos.py`：PX4 话题 QoS（BEST_EFFORT + TRANSIENT_LOCAL），避免默认 RELIABLE 导致收不到数据。
  - `state.py`：`Px4State` 快照 + `Px4StateMonitor`（状态订阅、时间戳与超时判定；实现为库而非节点）。
  - `test/test_frames.py`：8 个坐标系换算单元测试。
- `ros2_px4_ws/src/mission`（新包 0.1.0）——任务层，不直接接触 PX4 消息：
  - `waypoint_mission` 节点：WAIT_PX4 → ARM → OFFBOARD → TAKEOFF → CRUISE → LAND → WAIT_DISARM → DONE；
    逐阶段超时、到达判定（半径 0.5 m + 保持 1 s）、failsafe 或离开 OFFBOARD 立即中止、
    超时中止时自动触发降落。
  - `launch/square_5m.launch.py`：一键启动桥接与 5 m 方形航迹任务。

### 变更

- README 4.3 / 4.4 / 4.5：补充航点任务与桥接能力、更新未实现清单与已知问题。
- `docs/roadmap.md` 阶段 A：写入本次进度与遗留项。
- README 全量同步当前实现进度：第 5 节目录树补入 `px4_interface` 与 `mission`；
  第 6 节增加模块实现状态并说明状态跟踪以共享库实现；
  第 14 节 Phase 1 勾选航点状态机与 5 m 方形航迹、自动降落标记为进行中；
  第 20 节验收指标更新飞控层与规划层勾选状态；第 22 节「当前下一步」同步已完成项与后续步骤。

### 验证

- 单元测试：`pytest src/px4_interface/test/test_frames.py` → 8 passed。
- 编译：`colcon build --symlink-install --packages-select px4_interface mission` → 2 个包成功（1 分 31 秒）。
- 端到端飞行（AirSim + PX4 SITL + uXRCE-DDS + ROS 2）：

```text
起飞高度误差                       0.03 m
航点 0 / 1 / 2 / 3 / 4 位置误差     0.15 / 0.26 / 0.44 / 0.45 / 0.46 m（判定半径 0.5 m）
指令 ACK                           arm(400) OK、offboard(176) OK、land(21) OK
设定点流                           20 Hz（保持既有验证结论）
任务结果                           LEG 全部通过后，LAND 阶段 40 s 超时 → ABORT
取证                               logs/2026-09-16/05_45_12.ulg、logs/2026-09-16/mission_run2_console.log
                                   （日志目录按约定不入库，保留在本地）
```

### 已知问题

- **自动降落仍未通过**（沿用 2026-09-14 的遗留项）：`AUTO_LAND` 后 40 s 未落地。
  现场读取 `vehicle_land_detected`：`in_ground_effect=true`、`in_descend=true`、
  `close_to_ground_or_skipped_check=true`，但 `has_low_throttle=false`、
  `ground_contact=false`、`at_rest=false`、`landed=false`，
  即飞机在近地面悬停而非触地，落地检测链路无法置位。
- **航段速度偏慢**：5 m 直线航段耗时约 15–19 s（约 0.3 m/s），与 PX4 默认参数不符，
  待确认限制来自轨迹生成参数还是 AirSim 锁步时间。
- **首次运行失败（已修复）**：任务只在进入阶段时发布一次目标，被桥接节点的 2 s 目标超时判定为过期，
  导致未起飞即超时；已改为任务侧按 5 Hz 周期刷新目标。
- 状态桥接以共享库形式实现（与路线图原描述的“独立节点”不同），
  原因是避免重复订阅代码与额外进程，属于有意偏离。

---

## [2026-09-16]

### 新增

- 版本库初始化：项目根目录建立 Git 仓库，主分支 `main`，首个提交 `52daa86`
  （25 个文件，仓库对象 144 KiB，提交树 0.07 MB）。
  - `.gitignore`：排除构建产物（`build/`、`install/`、`log/`）、飞行与仿真数据
    （`logs/`、`*.ulg`、`*.bag`、`*.db3`、`*.mcap`）、Python 缓存与编辑器目录。
  - `.gitmodules`：三个第三方依赖以 submodule 固定版本，源码不入库：
    - `PX4-Autopilot` → `PX4/PX4-Autopilot` @ `d0c6c0df7f`（main 分支）
    - `ros2_px4_ws/src/px4_msgs` → `PX4/px4_msgs` @ `598c7aa`（release/1.18 分支）
    - `px4_ros_uxrce_dds_ws/src/Micro-XRCE-DDS-Agent` → `eProsima/Micro-XRCE-DDS-Agent` @ `57d0862`（v2.4.2）
- `scripts/build_dds_ws.sh`：重建 Micro XRCE-DDS Agent 工作区。
  Agent 的 `install/`（含 `MicroXRCEAgent` 可执行文件）不入库，
  新克隆的仓库必须执行本脚本才能得到 Agent；首次构建需要联网。
- README 新增「5.1 从远程仓库克隆与重建」：从零克隆 → 初始化 submodule →
  重建三套构建 → 启动链路，并说明 `--recursive` 的必要性。

### 变更

- 推送 `main` 到远程仓库 `https://github.com/hassetorvalds/px4_sar_uav.git`，
  本地 `main` 已跟踪 `origin/main`。
- 建立开发分支 `dev`（自 `main` 牵出）：后续功能开发在 `dev` 进行，稳定后合回 `main`。
- README 第 5 节目录树补充 `CHANGELOG.md`、`logs/` 与新增脚本条目。
- README 新增「5.2 分支约定」：`main`（稳定基线，只接受合并）/ `dev`（集成）/
  `feature/*` / `hotfix/*` 的分工，分支命名、提交信息前缀、CHANGELOG 同步要求，
  以及 submodule 指针变更与禁入库内容的约束。

### 验证

- `git ls-remote origin`：`refs/heads/main` 与本地 `HEAD` 同为 `52daa86`，工作区干净。
- 入库体积核对：提交树 0.07 MB、仓库对象 144 KiB；3.5 GB 构建产物与 71 MB 飞行日志均未入库。

### 已知问题

- README 5.1 的「全新克隆 → 重建」路径尚未在干净环境完整跑通。
  `build_dds_ws.sh` 的构建逻辑与已实际执行过的命令一致（`colcon build --packages-select
  microxrcedds_agent`），但本机 Agent 工作区是既有构建，未做从零重跑验证；
  首次在新机器使用若报错，请在本文件补充记录。

---

## [2026-09-14]

### 新增

- `scripts/`：项目统一脚本集
  - `env.sh`：统一环境入口，导出 `PX4_SAR_ROOT` / `PX4_DIR` / `ROS_WS` / `DDS_WS`，
    按顺序 source ROS 2、Agent、本工作区，并自动探测 `PX4_SIM_HOST_ADDR`（默认网关，失败时回退 `/etc/resolv.conf`）。
  - `run_agent.sh`、`run_px4_sitl.sh`、`run_offboard.sh`：启动 Agent / PX4 SITL / Offboard 验证节点。
  - `check_stack.sh`：链路冒烟检查（`/fmu/out` 话题存在性 + 频率）。
  - `build_ros_ws.sh`、`build_px4_sitl.sh`：换目录或改依赖后的重建脚本。
- `docs/roadmap.md`：后续开发路线图（工程前置 + 阶段 A–H，含依赖、工期、验收标准、风险）。
- `docs/migration-2026-09-14.md`：目录整合迁移记录（原因、步骤、验证、回退）。
- `logs/2026-09-14/10_54_06.ulg`：本次端到端验证的 PX4 飞行日志（约 71 MB），
  与 `build/` 解耦保存，避免重建时丢失。
- `CHANGELOG.md`：本文件。

### 变更

- **项目整合为单一目录**：`PX4-Autopilot/`、`px4_ros_uxrce_dds_ws/`、`ros2_px4_ws/`、`README.md`
  由 `/home/sol/` 根下搬迁至 `/home/sol/px4_sar_uav/`。
  - 搬迁方式：同一文件系统内的 `mv`（瞬时 rename，无数据拷贝），可秒级回退。
  - PX4 分支与提交**未改动**：`main`，`v1.18.0-beta1-442-gd0c6c0df7f`。
  - 版本基线：PX4 `main@d0c6c0df7f`、px4_msgs `598c7aa`（对应固件 `993f542f`，release/1.18 线）、
    Micro XRCE-DDS Agent `v2.4.2`、ROS 2 Humble。
- `README.md`：第 4 节改为准确的「已实现 / 未实现 / 已知问题」清单；
  第 5 节目录结构更新为整合后的真实结构，并写明“移动即重编译”的约定。
- `px4_ros_uxrce_dds_ws/install/setup.sh`、`install/local_setup.sh`、
  `install/microxrcedds_agent/share/microxrcedds_agent/package.sh`：
  修正写死的旧工作区绝对前缀（仅 POSIX `sh` 变体受影响，`setup.bash` 本身为相对解析）。

### 移除

- `/tmp/px4_old_build/`（约 1.7 GB）：搬迁前的 PX4 构建产物，含旧路径的绝对引用，迁移后已失效。

### 验证

重建（换目录后必须执行）：

| 项目 | 结果 |
|---|---|
| `make px4_sitl`（PX4-Autopilot） | 成功；二进制内路径更新为新目录；`rootfs/etc` 软链接重建 |
| `colcon build --symlink-install`（ros2_px4_ws） | 成功，2 个包，5 分 31 秒 |
| 旧路径残留引用 | ROS 工作区 0 处、PX4 构建产物 0 处、Agent 工作区 0 处 |

端到端飞行（Windows 侧 AirSim 在线，`PX4_SIM_HOST_ADDR=172.21.192.1`，`PX4_SYS_AUTOSTART=10016`）：

```text
Simulator connected on TCP port 4560      ✅
uxrce_dds_client synchronized             ✅ 全部 /fmu/out 数据写入器创建成功
Ready for takeoff!                        ✅
OFFBOARD                                  ✅ nav_state=14
ARM                                       ✅ arming_state=2
3 m 悬停                                  ✅ 实测 (0.00, 0.04, -2.99)
trajectory_setpoint 频率                   ✅ 20.000 Hz（min=max=0.050 s）
accepts_offboard_setpoints / preflight     ✅ true / true，failsafe=false
```

结论：整合未破坏既有链路，验证状态与搬迁前一致。

### 已知问题

- **自动降落未通过验证**：下发 `commander land` 后进入 `AUTO_LAND`（`nav_state=18`），
  但落地后出现 `Preflight Fail: High Accelerometer Bias`、`Preflight Fail: vertical velocity unstable`，
  `vehicle_land_detected.landed` 始终未置位，local position 的 z 由 -3 漂移至约 +2.2，
  最终以 `commander disarm -f` 强制收尾。取证日志见 `logs/2026-09-14/10_54_06.ulg`，
  安排为下一阶段（路线图阶段 A）的首要调试项。
- **坐标系约定待固化**：`/fmu/out/vehicle_local_position_v1` 与 `/fmu/out/vehicle_odometry`
  的数值与 PX4 uORB 一致（DDS 侧未做 NED→ENU 转换），进入感知/规划前需统一定义并配单元测试。

### 回退方式

```bash
cd /home/sol/px4_sar_uav
mv PX4-Autopilot ros2_px4_ws px4_ros_uxrce_dds_ws README.md /home/sol/
```

搬回后同样需要重新构建（`scripts/build_px4_sitl.sh` 与 `scripts/build_ros_ws.sh`），
因为构建产物中烧入了绝对路径。
