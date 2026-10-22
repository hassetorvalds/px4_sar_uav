# 未决事项与重要决策（Open Questions / Decisions）

> **用途**：项目中真正未定的问题、需要拍板的决策、以及“已知但暂不处理”的风险，统一记在这里。
> 不再散落在聊天/comm 信件里。每次出现新的不确定、契约变化、验证结论变化，**更新本文件**。
>
> 最近更新：2026-10-22
> 状态：`OPEN` 待处理 · `BLOCKED` 被外部条件卡住 · `DEFERRED` 明确暂缓 · `DECIDED` 已拍板 · `CLOSED` 已验证关闭

---

## 1. 当前真正未决（按优先级）

| # | 事项 | 状态 | 当前事实/证据 | 影响 | 下一步 / 决策点 |
|---|---|---|---|---|---|
| 1 | **C 的真实偏移/真实结构验证** | `OPEN` | 注入 A/B 通过：top1→true、top5→false；三路高度/anchor/transform 已落盘。但 ±0.5 m 偏移只是模型级（`logs/height_gate/c_validation_20261021-195034.json`），真实 PX4 偏移未注入；阶段 2 真实低/高结构未做 | 影响能否对外声明 C、能否开 Batch2b；不影响新执行器主链路 | 二选一：① 做真实 PX4 偏移/漂移时段回放；② 接受“阶段 1 模型级 + 注入 A/B”作为当前上限，阶段 2 等场景 |
| 2 | **Batch2a 真实高结构进场** | `BLOCKED` | 真实高结构 E23.5/N-21.9；15 m/45 s 超时；8 m/90 s 在距目标 6.3 m 处被真障碍挡住（`logs/2026-10-22/batch2a_high2_s1022-201223_driver.log`）；0 episode，不计分 | 拿不到有意义的闭合/避让统计；不阻塞主链路 | 需要可达进场路径（绕行/换点），否则 Batch2a 继续挂起 |
| 3 | **有界侧向绕行的真实验证** | `OPEN` | 新执行器 `detour_start/detour_done`，假 PX4 block 8–13 s PASS；真实注入测试 goto 正常到点，未触发绕行（注入链路未确认） | 真实作业遇到局部障碍时仍可能 abort；目前只有假 PX4 证据 | 先确认注入链路，或直接把同一策略接进旧驱动；不要重复烧真机 |
| 4 | **`area.polygon` 在已知地图外 → `TARGET_UNREACHABLE`** | `OPEN` | 编译器已做 polygon 形状/面积/自交校验；未接“已知地图边界”，只做 40 m 围栏（map 坐标时还跳过围栏距离校验） | 服务器 1.1 要求；非法区域可能到执行层才失败 | 需要地图边界/keepout 的权威来源；在编译器或执行器前置检查 |
| 5 | **`epoch_id` 语义** | `OPEN` | 目前 `epoch_id = task_id`（报告名去 `report_`）；服务器定义为 sim 起点/轮次 | 跨轮离线分析可比性；不影响飞行 | 定一个运行级 epoch（sim 启动/会话 id），与 task_id 区分 |
| 6 | **C 的 anchor/transform 真实性** | `OPEN` | 日志里 `anchor_id=local_map`、`transform_id=static_arm` 是固定标签；没有真正的 arm 时 REL/LCL 静态变换和地图重锚定记录 | 服务器要求 anchor_id/transform_id；影响 C 的可追溯性 | 若继续 C：补 arm 时静态变换落盘/重锚定；否则明确“阶段 1 标签是占位” |
| 7 | **覆盖默认 overlap 的统一** | `DEFERRED` | 1.1 area 无 spacing/overlap 时默认 overlap=0.25（spacing=1.5×alt）；旧文本解析默认 spacing=2（2 m 高时约 50% 重叠） | 影响航迹密度和覆盖率可比性 | 统一默认值前先确认旧 demo 是否需要保持 2 m 间距；不要无理由改历史基线 |
| 8 | **起飞前机体周围净空检查** | `OPEN` | 曾从贴 TemplateCube 的落点起飞即碰撞；当前 arm 前只查状态/位置/preflight/failsafe/电池，不查机体周围净空 | 真实作业起飞安全 | 需要可用的近场障碍判据（地面点会干扰），否则不做假检查 |
| 9 | **VLM `source=vlm` 的 JSON 路径** | `BLOCKED` | 模型仍是点预测器，不产可靠 JSON；`source=vlm` 的 `area_cover` 已按服务器要求 `OPERATOR_REQUIRED` 拒绝；当前 `--remote-intent` 用手写 operator SemanticIntent | 模型自动生成任务的能力；不影响任务层/Action | 等模型侧 JSON 约束解码；不要把 mock/手写当 VLM |

---

## 2. 已拍板、但需要保留上下文

| 事项 | 决定 | 理由/边界 |
|---|---|---|
| 旧驱动高度门默认值 | **默认 `height_gate_mode=legacy`，C 用 `c_map_rel` 显式开启** | 保留旧行为作保守基线；C 已实现但不默认，避免未验证路径影响旧回归 |
| legacy 能否当 C | **不能** | 服务器明确：legacy 轮 0 障碍=0 次决策，不能当 C 证据；只能标 `legacy_no_overfly` |
| Batch2a/Batch2b | **拆开**：2a 用 legacy 保守基线，标 `overflyable=not_exercised`；2b 等 C 通过 | 高结构点位上 legacy 与 C 行为等价；低结构/飞越分支只有 C 能回答 |
| 终态 cancel | **不新增 result**；cancel 一次、result 一次；三态 `ERROR_NONE/GOAL_TERMINATED/UNKNOWN_GOAL_ID` 已验证 | ROS 2 Action 语义；服务器已接受 |
| Finding 入口状态 | **入口即 `confirmed`**（detector hits≥3），`candidate` 留在 schema，不另造候选话题 | 服务器已接受；报告带 `entry_status/detector_hits` |
| `area` 字段名 | **`frame_id/polygon/line_spacing_m/overlap`**，兼容旧 `shape/center/width/height/spacing_m` | 与服务器 1.1 文本一致；已回信 |
| 旧驱动/新执行器相机 | 旧驱动=`bringup_legacy.sh`（前视单相机）；新执行器/覆盖=`bringup_coverage.sh`（下视双相机） | 避免同一开关在两套链路漂移 |
| 绕行策略 | **有界侧移，不是全局规划**；2 次×2 s×0.25 m/s | 先拿到可用的局部脱困，不把局部规划混进本轮 |
| C 阶段 1 结论 | **注入 A/B 有效；偏移仅模型级；阶段 2 未做** | 不能对外说“C 已验证” |

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

更新内容至少包含：**状态、事实/证据、影响、下一步/决策点、日期**。
已关闭项移入第 3 节，不删除历史。
