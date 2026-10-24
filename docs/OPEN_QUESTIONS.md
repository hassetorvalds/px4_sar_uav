# 未决事项与重要决策（Open Questions / Decisions）

> **用途**：项目中真正未定的问题、需要拍板的决策、以及“已知但暂不处理”的风险，统一记在这里。
> 不再散落在聊天/comm 信件里。每次出现新的不确定、契约变化、验证结论变化，**更新本文件**。
>
> 最近更新：2026-10-22
> 状态：`OPEN` 待处理 · `BLOCKED` 被外部条件卡住 · `DEFERRED` 明确暂缓 · `DECIDED` 已拍板 · `CLOSED` 已验证关闭

## 0. 证据等级（C/避障类结论必须标注）

| 等级 | 含义 | 例子 |
|---|---|---|
| `model` | 纯模型/离线注入，不经过 ROS/PX4 | `scripts/height_gate_validate.py` |
| `injection` | 通过 ROS 话题/SITL 注入可控输入 | `/inject/height_status` A/B；假 PX4 block |
| `real` | 真实 AirSim/PX4 飞行，真实结构/真实传感器 | 真机 coverage、n=3 |

**规则**：不得把 `model`/`injection` 写成 `real`；同一结论跨等级要分开列。

---

## 1. 当前真正未决（按优先级）

| # | 事项 | 状态 | 当前事实/证据 | 影响 | 下一步 / 决策点 |
|---|---|---|---|---|---|
| 1 | **C 的真实偏移/真实结构验证** | `OPEN` | 证据等级=`model + injection`。注入 A/B 通过：top1→true、top5→false；三路高度/占位标签已落盘。±0.5 m 偏移仅 `model`（`logs/height_gate/c_validation_20261021-195034.json`）；真实 PX4 偏移未注入；阶段 2 真实低/高结构未做。**已产出 `docs/REPLAY_PLAN.md`（计划，未实施）；real 级验证待场景 2 或对外声明前触发** | 影响能否对外声明 C、能否开 Batch2b；不影响新执行器主链路 | ①阶段 1 只声明 `model+injection`；②若推进 `real`，按 `docs/REPLAY_PLAN.md` 的 R1/R2 计划（PX4 Replay + C-gate 离线重放）；③阶段 2 真结构等可达场景 |
| 2 | **Batch2a 真实高结构进场** | `scene_blocked` | 真实高结构 E23.5/N-21.9；15 m/45 s 超时；8 m/90 s 在距目标 6.3 m 处被真障碍挡住（`logs/2026-10-22/batch2a_high2_s1022-201223_driver.log`）；0 episode，不计分。**真实高结构进场被真实障碍阻挡，按场景侧阻塞处理，不计入失败统计。** | 拿不到有意义的闭合/避让统计；不阻塞主链路 | 采用 `scene_blocked` 口径或距离阈值到达；换可达进场路径/点位；不强行闭合、不并表 |
| 3 | **有界侧向绕行的真实触发** | `OPEN` | 语义已收紧：**只在 `goto` 段启用**；takeoff/land/search 不触发。假 PX4 block 8–13 s PASS（`detour_start/done`）；真实注入测试 goto 正常到点、未触发（注入链路未确认） | 真实作业遇到局部障碍时仍可能 abort；目前 `injection` 级证据 | 用 SITL/注入脚本断言 `detour_start`，不依赖真机偶然触发；真实注入链路另查 |
| 4 | **`area.polygon` 编译期 map/keepout 检查** | `PARTIAL` | 编译器 `SemanticIntent.validate` 在 polygon 形状/面积/自交之后、产出 TaskSpec 前检查：`map_boundary` inclusion + `keepout_polygons` exclusion；错误码独立为 `ERROR_TARGET_OUTSIDE_MAP` / `ERROR_TARGET_IN_KEEPOUT`。**地图边界权威来源当前为 config 占位**；`map_boundary` 缺失时跳过（保持旧行为），`keepout_polygons` 缺失时回退 `keepout`。**空列表语义：显式 `[]` = 无 exclusion，不回退。** **顶点级检查，边相交不拒绝（已知边界）。** | 已消除“非法区域到执行期才失败”；但边界来源仍未接权威地图 | 接真实地图边界/keepout 来源；当前先由调用方显式配置 |
| 5 | **覆盖默认 overlap 的统一**（本轮需求称 #7；当前表内 #5） | `DEFERRED` | 1.1 area 无 spacing/overlap 时默认 overlap=0.25（spacing=1.5×alt）；旧文本解析默认 spacing=2（2 m 高时约 50% 重叠）。**已预留 `area.overlap_override` 配置入口**：缺失/null=默认，显式数值仅记录、不参与计算，非法值 `SCHEMA_INVALID`；本轮不实现覆盖逻辑 | 影响航迹密度和覆盖率可比性 | 切换 global/密集障碍场景/planner 时启用 `overlap_override` 并实现覆盖算法 |
| 6 | **起飞前机体周围净空检查** | `OPEN` | 曾从贴 TemplateCube 的落点起飞即碰撞；当前 arm 前只查状态/位置/preflight/failsafe/电池，不查机体周围净空 | 真实作业起飞安全 | 需要可用的近场障碍判据（地面点会干扰），否则不做假检查 |
| 7 | **VLM `source=vlm` 的 JSON 路径** | `BLOCKED` | 模型仍是点预测器，不产可靠 JSON；`source=vlm` 的 `area_cover` 已按服务器要求 `OPERATOR_REQUIRED` 拒绝；当前 `--remote-intent` 用手写 operator SemanticIntent | 模型自动生成任务的能力；不影响任务层/Action | 等模型侧 JSON 约束解码；不要把 mock/手写当 VLM |
| 8 | **SAR Demo v0 代理目标检测可靠性** | `OPEN` | 3 轮同一指令：搜索 8/8、无 abort、均落地；代理目标 2/3 命中、finding 2/3（轮3 miss）。目标为橙色物，**非人员检测** | 影响 Demo 可重复性；不影响任务/飞行链路 | 调 scan 时长/检测阈值/点位；先离线评估再跑少量真机；不要用单轮成功掩盖 2/3 |
| 9 | **VLM grounding provider 选择** | `PARTIAL` | Qwen3-VL-8B 同帧 offline：EN/ZH 15/15、平均误差 0.026/0.030、稳态 ~470 ms；LightNav 0.456、122–164 ms。one-shot `go_to` 0.27 m；3 样本 0.41 m；动态 `goto` 0.70 m（实测 provider 0.747 Hz，不是 2 Hz）；**未做视觉伺服/4Hz** | Qwen 可做 one-shot/异步目标 provider，LightNav 保留快速 fallback；目标仍有 1–2 m 不确定性 | 若要真 2Hz，需并发采集/推理；4Hz 仍用 LightNav 或量化 |

---

## 2. 已拍板、但需要保留上下文

| 事项 | 决定 | 理由/边界 |
|---|---|---|
| C 证据口径 | **阶段 1 = `model+injection`**；`real` 需真实结构/真实偏移 | 借鉴 PX4 测试分层；不自建真机偏移注入，优先 Replay |
| 旧驱动高度门默认值 | **默认 `height_gate_mode=legacy`，C 用 `c_map_rel` 显式开启** | 保留旧行为作保守基线；C 已实现但不默认，避免未验证路径影响旧回归 |
| legacy 能否当 C | **不能** | 服务器明确：legacy 轮 0 障碍=0 次决策，不能当 C 证据；只能标 `legacy_no_overfly` |
| Batch2a/Batch2b | **拆开**：2a 用 legacy 保守基线，标 `overflyable=not_exercised`；2b 等 C 通过 | 高结构点位上 legacy 与 C 行为等价；低结构/飞越分支只有 C 能回答；场景阻塞标 `scene_blocked` |
| 终态 cancel | **不新增 result**；cancel 一次、result 一次；三态 `ERROR_NONE/GOAL_TERMINATED/UNKNOWN_GOAL_ID` 已验证 | ROS 2 Action 语义；服务器已接受 |
| Finding 入口状态 | **入口即 `confirmed`**（detector hits≥3），`candidate` 留在 schema，不另造候选话题 | 服务器已接受；报告带 `entry_status/detector_hits` |
| `area` 字段名 | **`frame_id/polygon/line_spacing_m/overlap`**，兼容旧 `shape/center/width/height/spacing_m` | 与服务器 1.1 文本一致；已回信 |
| 旧驱动/新执行器相机 | 旧驱动=`bringup_legacy.sh`（前视单相机）；新执行器/覆盖=`bringup_coverage.sh`（下视双相机） | 避免同一开关在两套链路漂移 |
| 绕行策略 | **有界侧移，不是全局规划；只在 `goto` 段**；2 次×2 s×0.25 m/s | 先拿可用的局部脱困；Takeoff/Land/Search 不触发 |
| `epoch_id` 语义 | **会话级 id**（进程启动时间/序号），与 `task_id` 解耦 | 对齐 PX4 uLog “会话起点 + 相对时间”；分析用 `(epoch_id, task_id, timestamp)` |
| C 的 anchor/transform | 当前为 **`local_map@stage1_placeholder` / `static_arm@stage1_placeholder`** | 明确是占位符，不用于跨会话 C 追溯；阶段 2 再落真实 arm 时静态变换 |

---

## 3. 已关闭（有证据，不再重复讨论）

| 事项 | 证据 |
|---|---|
| SemanticIntent → TaskSpec → `--remote-intent` 真机 | `logs/tasks/report_20261021-191101-802.jsonl`：8/8、16 m²、64/64 |
| 同点位 n=3 | `report_20261021-192645-123` / `192735-681` / `192831-246`：均 64/64、无 abort |
| 真机转场高度剖面 | `report_20261021-191221-106`：36/36 → climb 3 m → 36/36 |
| cancel 三态 / result 一次 | active 0/1、terminal 3/0、unknown 2/0 |
| `LANDED_ELSEWHERE` / 落地单所有者 | 假 PX4 external disarm：`LANDED_ELSEWHERE → DONE`，不再发落地 |
| `land`/`land_fast` bridge 去重 | 2 s 窗口单测 |
| simcheck 域不可达 fail-closed | 探域失败 exit 2；连续 2 次 frames>0 才 ready |
| 覆盖热力图/位姿误差 | 真机 144/144；PNG + `pose_age/eph/epv` |
| 坐标偏移（相对起飞点） | Task1 落点 + Task2 相对 search 64/64 |
| A/B/area 契约 | 服务器 2026-10-21/22 回信确认 |

---

## 4. 更新规则

出现以下情况时，**先更新本文件，再继续开发/飞行**：

1. 服务器/合作方新增或改变契约；
2. 一次验证得到“不支持/部分支持”的结论；
3. 需要选择实现路线（legacy vs C、绕行 vs 全局规划、默认 overlap 等）；
4. 外部条件阻塞（场景/隧道/服务/点位不可达）；
5. 决定“暂不做”的风险（必须写影响和复查条件）。

更新内容至少包含：**状态、事实/证据、证据等级、影响、下一步/决策点、日期**。
已关闭项移入第 3 节，不删除历史。
