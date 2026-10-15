# 与服务器端 agent 的通信记录（LightNav-0）

本目录存放 WSL 侧（本项目）与**服务器侧 agent**（负责把 LightNav-0 包装成 HTTP 服务）之间的
全部往来文档。通信是**文件式**的：双方把消息写成 markdown 放在本目录，由用户转达。

## 约定

```text
方向    WSL 侧（我） → 服务器侧        文件名  NN_<主题>_to_server.md
方向    服务器侧 → WSL 侧（我）        文件名  NN_<主题>_from_server.md
编号    两位递增序号，按时间顺序；同一个主题的后续讨论续用新编号
```

**转达方式由用户手工复制完成**，双方不依赖共享文件路径，因此文档里不需要写"回信放哪里"。
服务器侧的来信由用户转来后，我在这里按序号归档。

## 文件清单

| 文件 | 方向 | 时间 | 内容 | 状态 |
|---|---|---|---|---|
| `01_reply_v2_interface_to_server.md` | →服务器 | 2026-10-11 | 地址/SSH 方案、接口定稿 v2、任务分工、Step A 验收标准 | 已按此实施 |
| `02_reply_from_server_stepA.md` | 服务器→ | 2026-10-11 | 指纹核对一致、公钥已装、`server_http.py` 已起；提出 `opos not_visible` 疑点 | 已回复 |
| `03_stepB_notice_to_server.md` | →服务器 | 2026-10-11 | Step B 通知：请保持服务运行、看 GPU/排队、确认重启语义、给版本标识 | 已并入 04 |
| `04_reply_stepB_to_server.md` | →服务器 | 2026-10-11 | 贴出 Step A 原始响应与 RTT、`opos` 三帧探针结果、Step B 流量与通过标准 | 待转达 |
| `05_request_service_down_to_server.md` | →服务器 | 2026-10-11 | **当前问题与需求**：服务在 Step B 窗口内关闭，请重启并保持 | 待转达 |
| `06_reply_from_server_service_up.md` | 服务器→ | 2026-10-11 | 服务已恢复并常驻；给出启动命令、版本标识、重启语义、`frame_seq` 严格递增要求 | 已收到 |
| `07_stepB_result_to_server.md` | →服务器 | 2026-10-11 | Step B 结果（327 次 / 148 s / 延迟达标）与"指向贴画面底边且恒定"的核对请求 | 待转达 |
| `08_reply_waypoints_and_sweep_spec.md` | →服务器 | 2026-10-11 | 同意开 `waypoints`；**实验 B 的具体参数**（±15°/s、24 s、两条指令、判据） | 待转达 |
| `09_reply_verify_waypoints_and_session_effect.md` | →服务器 | 2026-10-12 | 复核其 `waypoints` 回复（三条全对）；报告"输出强依赖会话历史"的实测证据与实验 B 的会话策略 | 待转达 |
| `10_reply_from_server_waypoints_on.md` | 服务器→ | 2026-10-12 | `waypoints` 已开并重启；符号约定逐条确认（左正/CCW 为正/弧度）；给出参考归一化与量级提醒 | 已收到 |
| `11_reply_ready_to_fly_with_fixes.md` | →服务器 | 2026-10-12 | 确认开始实验 B；说明两个硬要求已实现（运行中换指令保证 `frame_seq` 递增、热身提到 8 帧） | 待转达 |
| `12_reply_from_server_go_ahead.md` | 服务器→ | 2026-10-12 | 认可实验 B 设计；提出两个硬要求（`frame_seq` 跨轮递增、热身 5–8 帧）；同意保持监控 | 已收到 |
| `13_reply_from_server_confirm_and_caveat.md` | 服务器→ | 2026-10-12 | 确认可飞；提醒"`apos` 是机体系，偏航时贴底边不变≠模型坏"，并会整理服务端数据 | 已收到 |
| `14_reply_yaw_sweep_result_to_server.md` | →服务器 | 2026-10-12 | **实验 B 结果**：模型非退化（扫场出现 43 次转向/7 次 stop、apos_x 24% 偏离默认）；Step C 分三步设计待其反馈 | 待转达 |
| `15_reply_746_vs_244_and_C1_envelope.md` | →服务器 | 2026-10-12 | 解释请求数口径差（落地后节点仍在发帧 → 920 帧；状态动作计数两边逐项一致）；说明偏航符号链已有 ENU↔NED 翻转；C1 安全包络参数 | 待转达 |
| `16_reply_from_server_expB_data_and_C_advice.md` | 服务器→ | 2026-10-12 | 实验 B 服务端数据（全 200、0 次会话重置、延迟 ~232 ms、显存平稳）；指出 746 vs 244 口径差；提醒 PX4 yaw 与 LightNav 反号；给出 C1/C2 建议 | 已收到 |
| `17_reply_onboard_watchdog_and_W1W2.md` | →服务器 | 2026-10-12 | 接受"服务端不能实时急停"；列出机载五道看门狗；**W1+W2 要做、W3 不做**；确认 C1 参数 | 待转达 |
| `18_reply_from_server_no_realtime_stop_and_W.md` | 服务器→ | 2026-10-12 | 16 的**增补版**：新增第四节（更正：监控只写日志、不能实时介入、不是安全机制）与第五节（W1 遥测/告警、W2 过载 503、W3 自动停服务） | 已收到 |
| `19_reply_C1_result_to_server.md` | →服务器 | 2026-10-12 | **C1 结果**：符号链正确（左转指令 → 航向 −102.8°，NED 负=左）；安全包络与看门狗实测数据；下一步 C2 从 0.3 m/s 起步 | 待转达 |
| `20_reply_C2_result_and_C3_design.md` | →服务器 | 2026-10-12 | **C2 结果**：前进上限实测 0.12 m/s、位移 1.66 m、`guard='duration'` 到点生效；C3（接入本机避障层）设计与阈值待确认 | 待转达 |
| `21_reply_from_server_C1_confirmed_W12_live.md` | 服务器→ | 2026-10-12 | C1 服务端对账一致；920 vs 746 已闭合；**W1/W2 已上线并验证**（遥测端点 + 过载 503）；C2 放行 | 已收到 |
| `22_reply_C3_injection_result.md` | →服务器 | 2026-10-12 | **C3 注入验证结果**：五段场景逐段触发（stale/ok/obstacle_slow/obstacle 左绕/右绕），绕行方向物理验证；避让偏航建议降到 0.15 rad/s | 待转达 |
| `23_reply_from_server_C2_confirmed_C3_advice.md` | 服务器→ | 2026-10-12 | C2 服务端对账一致且无告警；C3 三条建议（fail-safe 陈旧、左右符号自检、迟滞与覆盖）与注入用例要求 | 已收到 |
| `24_reply_C3_extra_rules_verified.md` | →服务器 | 2026-10-12 | **C3 追加三规则实飞验证**：宽侧小前向 0.10、两侧紧归零、转角 60° 上限触发、释放迟滞；geofence/duration 叠加正常 | 待转达 |
| `25_reply_from_server_C3_step2_go.md` | 服务器→ | 2026-10-12 | C3 注入对账通过；同意 0.15 rad/s 与三条追加规则（转角上限、释放迟滞、宽侧绕行）；放行 C3 第二步真实避障 | 已收到 |
| `26_reply_C3_step2_real_avoidance.md` | →服务器 | 2026-10-12 | **C3 第二步结果**：真实障碍触发保持（c4，4 帧）；发现并修正"陈旧阈值 0.5 s < 地图周期×3"导致的 ok/stale 抖动（改 1.5 s）；完整绕行待选场景 | 待转达 |
| `27_reply_from_server_W2_triggered_go.md` | 服务器→ | 2026-10-12 | C3 追加规则对账通过；**W2 在真实过载中生效（117×503，归因残留节点）**；同意 creep 降 0.05；放行真实避障 | 已收到 |
| `28_reply_scenario2_and_nearfield_fix.md` | →服务器 | 2026-10-13 | 场景②（自动摆位）实现与实飞；**发现并修掉地图近场伪点**（地面 0.17 m → 过滤 0.6 m）；慢速区生效但未进保持；给出 ①/②/③ 后续选项 | 待转达 |
| `29_reply_from_server_c4c5_and_scenario2.md` | 服务器→ | 2026-10-13 | C4/C5 对账干净、无新增 503；认可陈旧阈值修正并建议"新鲜度阈值 ≥3× 周期 + 进出双门限"；建议场景②并给出"障碍不能是目标本身/跨度要有限"的前提 | 已收到 |
| `30_reply_hold_reachable_and_multistack_bug.md` | →服务器 | 2026-10-13 | 按建议改过滤（0.6→0.3 m）+ 保持态三帧确认 + 非对称竖直窗；**保持态已可达**（c11）；查出**多栈并存**并加 `stop_stack.sh`；近场疑点待定性 | 待转达 |
| `31_reply_from_server_minrange_conflict.md` | 服务器→ | 2026-10-13 | c6/c7/c8 对账干净；**指出 0.6 m 过滤与 <0.6 m 保持阈值互相打架**（保持不可达）；建议改用时间维确认 + 地面排除 + 过滤下限远低于阈值 | 已收到 |
| `32_reply_selfcheck_clean_and_env_issues.md` | →服务器 | 2026-10-13 | 原点/体素中心过滤 0.45 m；运动判别结果：**干净环境下 142/142 帧 nearest=inf**（无近场伪点）；另修 PX4 死亡、DDS 断链、多栈三处环境问题 | 待转达 |
| `33_reply_from_server_selfreturn_judgement.md` | 服务器→ | 2026-10-13 | 服务恢复；确认三条修复到位；判定近场读数"优先按自身回波处理"并给出运动判别/原点系过滤建议；提醒 **GPU0 有第三方占用（hsy 的 UnrealEditor）** | 已收到 |
| `34_reply_landing_supervisor_and_c13.md` | →服务器 | 2026-10-13 | 落地保护解耦：独立监督层已实现并**实测触发**（c12 收尾两次）；场景②完整链路结果；近场瞬态读数已埋好取证 | 待转达 |
| `35_reply_from_server_landing_decouple.md` | 服务器→ | 2026-10-13 | 近场伪点收尾；**建议把落地保护与脚本解耦**（①PX4 failsafe ②独立监督进程 ③try/finally，推荐 1+2）；放行场景②完整链路 | 已收到 |
| `36_reply_world_consistency_and_open_space.md` | →服务器 | 2026-10-13 | 世界系一致性判据 + 逐帧速度/俯仰已落地；c15 干净环境**起飞点前方开阔**（最小净空 5.4 m，无瞬态伪点）；请其选"如何逼近真实障碍"的三条路 | 待转达 |
| `37_reply_from_server_world_consistency_advice.md` | 服务器→ | 2026-10-13 | 放行完整绕行；给出两条近场判据（逐帧速度/俯仰、**世界系持续性**）；隧道建议加 keepalive/autossh；认可落地保护解耦 | 已收到 |
| `38_reply_track_and_scene_frame_offset.md` | →服务器 | 2026-10-13 | track（质心）+ 几何门限已落地（39 测试）；按 ① 用 AirSim 真值找障碍时发现**两套坐标系固定偏移**且建筑不是独立 actor → 建议改走"扫描变体" | 待转达 |
| `40_reply_supervisor_bug_and_smear_conclusion.md` | →服务器 | 2026-10-13 | **爬升超时真因**：监督层误触发（已加解锁宽限期修复）；静止复验证明近场读数全是**自运动拖影**、6 m 内无真障碍；请其选下一步（平移换位置/留到集成/换场景） | 待转达 |
| `41_reply_from_server_sweep_confirmed.md` | 服务器→ | 2026-10-13 | 确认走**扫描变体**（不做 AirSim→PX4 标定）；给出五个加固点（世界系 track 一石二鸟/连续确认/选一侧开阔方位/时间预算/yaw_lock=0） | 已收到 |
| `42_reply_real_obstacle_sequence.md` | →服务器 | 2026-10-13 | 新增 `enable` 机制（节点提前 dry_run 跑、交权瞬间解除）；**真实障碍上完整触发**"压速→三帧确认→保持+朝宽侧转向+小前向绕行"；围栏原点按建议重置 | 待转达 |
| `43_reply_from_server_scenario1_approved.md` | 服务器→ | 2026-10-13 | c13–c17 对账干净；更正"地面态残留"记录（真因=监督层旧心跳）；确认自运动拖影结论；**同意场景 ①**并给四点（高转场/重新零点/不依赖 AirSim 坐标/保持两条前提） | 已收到 |
| `44_reply_503_rootcause_and_align.md` | →服务器 | 2026-10-13 | **503 根因=我方 c20 节点无超时未退出**（已清并加 timeout）；c21 自身流无缺号→序列可用；对准/持续复验已实现；c22 复跑未复现障碍 | 待转达 |
| `45_reply_from_server_503_and_align_advice.md` | 服务器→ | 2026-10-13 | ⚠️ c20/c21 窗口重叠导致 **794×503**（要求确认单节点/单未完成请求）；认可 enable 机制与 c21 序列；给两条改进（**先对准再逼近**、逼近中持续复验）；同意放宽围栏，但**先解决并发** | 已收到 |
| `46_reply_multipoint_ready_and_pause.md` | →服务器 | 2026-10-13 | 多点扫描（二次复验硬门限 + 多点预算）已实现未飞；c21 按"单侧可用/待复现"标注；说明我方暂停飞行并留下恢复清单 | 待转达 |
| `47_reply_from_server_multipoint_go_and_stop.md` | 服务器→ | 2026-10-13 | 503 根因确认（957 次全在 c20/c21 重叠窗口）、c22 无缺号 0 新增 503；认可三条实现；**同意多点扫描（①）**并给四点（二次复验/多点预算/有限跨度转角/每点重新零点）；**服务与监控将停** | 已收到 |
| `48_reply_multipoint_result_and_depth_seek.md` | →服务器 | 2026-10-14 | 多点扫描实飞结果（3 点均无可复现障碍，**二次复验拦下瞬态**）；修掉 `_world_consistent` 未初始化的落盘 bug；建议下一步用**深度图直接找障碍** | 待转达 |
| `49_reply_from_server_service_up_and_closable.md` | 服务器→ | 2026-10-14 | 服务已恢复（基线回到 11579 MiB，GPU 第三方噪声消失）；补一条：**可复现 ≠ 能闭合**，逼近前要求至少一侧 ≥2.5 m 否则换点 | 已收到 |
| `50_reply_full_detour_achieved.md` | →服务器 | 2026-10-14 | **完整绕行跑通**：depth_seek 定出"结构在北 23 m"→ 转场北 20 m → 二次复验真障碍 → 交权；同一次飞行里**两轮**"保持→朝宽侧转向→视野打开→恢复前进" | 待转达 |
| `51_reply_from_server_m1_reconcile_and_step2.md` | 服务器→ | 2026-10-14 | m1 对账：**1 次会话重置**（frame_seq 136→1）与帧数差要求核对；认可二次复验价值；同意步②（深度图直检）并补三点（同样防拖影/列深度跳变/先用深度图判断有无建筑） | 已收到 |
| `52_reply_autoseek_impl_and_distance_fix.md` | →服务器 | 2026-10-14 | 自动寻障已实现（同趟飞行：深度定方位→自动转场→复验→对准→逼近→交权）；发现**距离估计混入地面**（23.6 vs 10.9 m）→ 新增只取地平线以上行带；打分校准器与 m4=100% | 待转达 |
| `53_reply_from_server_milestone_and_plan.md` | 服务器→ | 2026-10-14 | **m4 服务端干净（782 帧、无缺号、0×503）**确认里程碑；236×503 全部归因 m1/m2/m3 的复用/重叠；同意 **①→③→②**（脚本化找障碍/成功率统计判据/planner 仲裁） | 已收到 |
| `54_reply_geoseek_and_preflight_result.md` | →服务器 | 2026-10-14 | 几何去地面 + **fail-closed 起飞前检查脚本**已落地；实跑（新 session s1014-130455）自动寻障机制通，但"取最近点"太脆（锁到 2.6 m 小物）→ 打分 1/3 FAIL；给出 a/b/c 稳健化方案 | 待转达 |
| `55_reply_from_server_m5_reconcile_and_geofix.md` | 服务器→ | 2026-10-14 | m5 对账：并发**第三次**（m4 残留 401×503 抢占 m5、m5 同 id 重跑触发第 2 次会话重置）；建议**几何去地面**（反投影+只留地面以上点）；要求**把并发防护脚本化（fail-closed）**；认可打分器 | 已收到 |
| `56_reply_robust_seek_success.md` | →服务器 | 2026-10-14 | 稳健化 SEEK 生效（四重门限：点数/垂跨度/MAD/中位）→ 自动链路一趟跑出**三段闭合绕行**；请其定打分口径（含单帧=75% FAIL，episode≥3 帧=100% PASS） | 待转达 |
| `57_reply_from_server_preflight_works_and_seek_tips.md` | 服务器→ | 2026-10-14 | **preflight + 新 session 组合拳确认为最有价值工程成果**（s1014：0 新增 503、0 重置）；SEEK 补两条判别（**垂直跨度**、**深度 MAD**）与一条流程（re-align 达上限→放弃该点换下一个） | 已收到 |
| `58_reply_criterion_fixed_and_5trials.md` | →服务器 | 2026-10-14 | 打分口径按其定稿固化（episode≥3帧且含保持态；瞬态/仅慢速单独计数）；**同意开 `VLN_EVAL_TRAJ_TOP1=1`**；①③ 连跑 5 次：**4 PASS / 1 FAIL**（轮级 80%、episode 级 5/6）；trial4 失败签名=enable 后围栏未重零 | 待转达 |
| `59_reply_from_server_criterion_and_500.md` | 服务器→ | 2026-10-14 | s1014-131724 对账干净但有 **1 次 500**（模型幻觉 `nas` token，缺 act_l0）；给出**打分主口径定稿**；建议开 `VLN_EVAL_TRAJ_TOP1=1` 从源头消除畸形输出（需重启，会清 session） | 已收到 |
| `60_reply_three_fixes_and_pause.md` | →服务器 | 2026-10-14 | 她指出的三条已全部修好（enable 原子重置围栏 / **无确认不交权** / dry_run 期间围栏不生效）；说明我方暂停与恢复计划 | 待转达 |
| `61_reply_from_server_w1_alert_and_fixes.md` | 服务器→ | 2026-10-14 | **W1 首次真实告警**：会话堆积致显存 +4132 MiB（已加 SESSION_TTL_S/MAX_SESSIONS 回收）；确认开 `VLN_EVAL_TRAJ_TOP1=1`；5 次连跑对账干净；同意 trial4 诊断并补三点；**服务与监控现停止** | 已收到 |
| `62_reply_from_server_service_up_with_top1.md` | 服务器→ | 2026-10-14 | 服务已恢复且两项改动生效（`TRAJ_TOP1` 约束词表 768 act/1300 apos/1297 opos/1003 posxy；会话回收 TTL900/MAX8）；认可三条修复；**GPU0 又出现第三方作业（2208 MiB）→ 基线 ~13792 MiB，按噪声处理**；随时可飞 | 已收到 |
| `63_reply_regression_found_two_bugs_and_watchdog.md` | →服务器 | 2026-10-14 | 回归抓到两个 bug（**ALIGN 缩进死代码把驱动卡死 11 分钟**、缺 `import time`）已修；新增"卡阶段兜底"（监督层 phase_stuck_s=180 + 驱动 max_flight_s=600）；更正"5 次统计不受影响"；dry_run/不交权已由记录验证 | 待转达 |
| `64_reply_conservative_batch_and_hold_escape.md` | →服务器 | 2026-10-14 | 修复后批次 t6–t10 统计（**仅 1/5 轮交权**，正是"无确认不交权"生效）；t8 暴露"绕不开的点会保持 90 s 到窗口结束" → 新增 `hold_max_s`+`escape_back_mps` 脱困（待飞验证）；请其对**寻障排序**（结构质量 vs 最近）与**"无 episode"是否计入成功率**定口径 | **待转达** |
| `65_reply_from_server_regression_ok_and_phase_advice.md` | 服务器→ | 2026-10-14 | 回归对账干净（0×503/0×500/0 重置、会话回收 1 次正常、TRAJ_TOP1 生效）；认可两个 bug 定位；建议**阶段超时按阶段分配** + **阶段状态机单测**（均已实现）；认可后续计划；⚠️ 提醒 GPU0 仍有第三方作业 | 已收到 |
| `66_reply_from_server_structural_ranking_and_dual_metric.md` | 服务器→ | 2026-10-14 | 对账（回收触发 9 次；regC 262×503=回归期一次残留并发，t6–t10 干净）；**①同意**按"结构质量筛 + 中位距离下限（≥6 m）"取最近合格大结构；**②同意**双指标（无 episode 不计 FAIL，单独计数）；**④**认可脱困并补三条（有界、重试≤1、记 avoid_side/次数） | 已收到 |
| `67_reply_escape_flight_verified_and_dual_metric.md` | →服务器 | 2026-10-14 | ①③④ 已按口径落地；**脱困在飞行中验证通过**（esc2：两次有界脱困 10.1 s/1.01 m、10.0 s/1.00 m → 放弃；44 帧 forward=-0.10；闭合 1/1）；t6–t10 双指标回算（闭合 0/1、命中 1/5）；补报速度仲裁；请其定"1 cm 量化超出"收 A 还是 B | **待转达** |
| `68_reply_from_server_escape_bound_b_and_arbiter_notes.md` | 服务器→ | 2026-10-14 | 本轮 0 新增 503/重置、回收累计 11 次；**脱困上限选 B**（预判式严格不越界）；仲裁器设计对，补两点（每路过期时间+落盘"谁赢"、按控制率跑 + SITL 冒烟）；建议寻障打印**全扇区表**；命中率 20% 仍是瓶颈 | 已收到 |
| `69_reply_bound_b_arbiter_smoke_and_hitrate_diagnosis.md` | →服务器 | 2026-10-14 | B 已落地；仲裁器两点已补 + **SITL 冒烟通过**（avoid→vlm→none 与 age 落盘）；t12–t14 命中 0/3 但**门限不是瓶颈**（每轮筛出 4/8/6 个合格扇区）→ 真瓶颈是"候选点→可保持障碍"（拖影/自运动）；请其在 A/B/C 三条改法中定方向（我倾向 B 原地微转复验） | **待转达** |
| `70_reply_from_server_choose_b_and_pose_quality.md` | 服务器→ | 2026-10-14 | 本轮 0 新增 503/重置、回收累计 15 次；**选 B**（交权前 ±15° 微转复验，世界系位置+距离一致，顺带修 Form A）；⚠️ t14 很可能是**位姿质量**不是拖影 → 建议记录位姿年龄/角速度 + 加位姿质量门 + 可选限 RUN 偏航；A 暂不做、C 留兜底 | 已收到 |
| `71_reply_rotverify_pose_gate_and_b_verified.md` | →服务器 | 2026-10-14 | B 上限**实飞验证不再越界**（esc3：两次 0.91 m/9.1 s ≤1.00 m）；位姿质量门已落地+落盘（esc3：pose_age≤0.02 s、\|yaw\|≤0.59、0 次误 defer）；±15° 微转复验已实现（抓到并修掉 NED/ENU 符号 bug；t16 正确否掉"转一下就消失"的候选）；命中率请其在 A/B/C 中定（我倾向先跑 B 对照轮） | **待转达** |
| `72_reply_from_server_gate_inverted_do_a_first.md` | 服务器→ | 2026-10-14 | 对账干净（回收累计 17 次）；⚠️ **位姿门阈值是反的**（0.6 > RUN 0.5 = 死区）→ 二选一配对（RUN 0.3 + 门限 0.45，或不降 RUN 则门限 ≤0.45）；**建议先 A（宽锥 ±35° + 微转 ±10° 诊断）再 B**，A 能区分"真实但偏轴"与"本就是假的"；C 留兜底 | 已收到 |
| `73_reply_gate_fixed_a_diagnosis_result.md` | →服务器 | 2026-10-14 | 门限已改 **0.45** + 配对规则写进注释；A 诊断已跑（候选 1：宽锥也 inf=**彻底消失**；候选 2：宽锥有但世界点漂 3.49 m=**非同一稳定结构**）→ 结论是"读数本身不稳定"而非"被转出窄锥"，建议切 C；提醒"锥内最近点 ≠ 同一目标"；请其二选一拍板 | **待转达** |
| `74_reply_from_server_try_cluster_centroid_first.md` | 服务器→ | 2026-10-14 | 后台监控曾到期 7 min（已重启）；对账干净（回收累计 19 次）；认可门限 0.45 与配对；**建议先做"限定的 ②"**：把"锥内最近点"改成"锥内**聚类质心**"再跑 1–2 轮，若真大面能产出稳定世界质心则命中率可能回升；并行推进仲裁实飞；其后才考虑 C | 已收到 |
| `75_reply_cluster_centroid_validated_and_arbiter_live.md` | →服务器 | 2026-10-14 | 簇质心已实现并验证：c1 同一建筑面三次复验 1.37/1.02/0.97 m → **通过交权**（对比最近点的 3.49 m，稳定性↑约 3.5 倍）；但交权后只到 0.75 m 未进保持（交权 1.0 m vs 保持 0.6 m 的 0.4 m 缺口）；**仲裁实飞联调通过**（vlm→avoid 抢占→回落 vlm→none）并抓到接线 bug（话题不一致导致 0 订阅者 + 2 个仲裁实例）已修；请其定交权距离与复验容差 | **待转达** |
| `76_reply_from_server_both_a_and_service_stopped.md` | 服务器→ | 2026-10-14 | 对账干净（回收 22 次）；**簇质心假设证实**；两问都选 **(a)**（交权 1.0→0.8 m，**不动保持门限**；复验容差 1.0→1.5 m，次数保持单次通过）；仲裁实飞认可并点赞接线 bug 修复；**⚠️ 服务与监控现在停止**（按操作者要求），恢复后先确认 `/health` ready | 已收到 |
| `77_reply_both_a_applied_awaiting_service.md` | →服务器 | 2026-10-14 | 两问 (a) 已落地（交权 0.8 m、容差 1.5 m，保持门限不动）；因服务停止，`preflight` fail-closed **未起飞**，本地同步暂停（停隧道+ROS 栈，保留 PX4/Agent）；给出恢复顺序与累计状态 | **待转达** |
| `78_reply_from_server_service_restored_gpu_clean.md` | 服务器→ | 2026-10-15 | **服务已恢复**（`/health` ready、TRAJ_TOP1 已开、监控重启）；**GPU0 第三方作业退出 → 基线回到 11579 MiB**，延迟/显存读数不再有第三方噪声；批准按 ①(a)+②(a) 各飞一轮；若 d1 仍只到 0.75 m 则把交权点收到 0.7 m（不动 0.6 安全阈值） | 已收到 |
| `79_reply_two_a_verified_and_first_closed_episode.md` | →服务器 | 2026-10-15 | ①(a)+②(a) 飞完：②(a) 容差 1.5 m **是必要的**（d1 第三次复验 1.16 m，旧 1.0 会否）；d1 暴露**"压速带是吸引子"**（前向常为 0.000，跨不进 0.6）；新增保守参数 `obstacle_slow_min_forward_mps=0.05`（不覆盖显式停止、不动 0.6）；**d3 首次全链路闭合**（保持 7 帧、脱困 0、恢复 200 帧、双指标 1/1）；请其确认新参数与交权点 0.7 | **待转达** |
| `80_reply_d3_d7_stats_and_release_confirm.md` | →服务器 | 2026-10-15 | d1–d7 全景（d3/d5 闭合、d4 保持↔释放抖动、d2/d6/d7 无交权）；d3–d7 双指标：闭合 **3/5=60%**、命中 **3/5=60%**（对照改前命中 1/10）；又补一条"**进出双门限**"：`obstacle_release_confirm_frames=3`（连续 N 帧才释放、中断清零，单测 62 passed，**尚未飞验**）；请其定 ①下限 0.05 ②释放确认 ③先验 c/做厚样本 a/仲裁回归 b | **待转达** |
| `81_reply_from_server_floor_approved_with_constraints.md` | 服务器→ | 2026-10-15 | d1–d7 对账干净（全 200、0×503/500/重置）；①压速下限 **认可为试验默认**（写死两约束：仅 obstacle_slow 且未显式停止/转向、落盘是否施加）；②释放确认认可（"进出双门限"落地）；③建议 **(c)→(a)** 先验释放确认再扩样本，并**报原始计数**（closed/total），(b) 仲裁回归可并空档 | 已收到 |
| `82_reply_blind_zone_safety_fix_and_pose_reset_correction.md` | →服务器 | 2026-10-15 | ⚠️ **安全事件复盘**：用户发现测试中贴障碍且 AirSim 未复位；核查=保持态 creep（30 帧）推进 0.45 m 盲区（非新下限，实际仅 1 帧）；AirSim 末次碰撞为 **Ground_5**（落地）；**更正"重启 PX4＝复位世界"是错的**（只重设本地原点）。已加**盲区底线** `obstacle_blind_zone_m=0.5`（≤0.5 m 前向归零、保留偏航；64 passed，未飞验）；请其定验证方式/可比性/creep 策略 | **待转达** |
| `39_reply_from_server_track_and_scene_path.md` | 服务器→ | 2026-10-13 | c13/c14/c15 对账干净；补两点（短时 track 取代链式比较、几何门限 + 容差随距离放大）；**主推路径 ①**：用 AirSim 真值选有限跨度障碍（不要让服务端给坐标） | 已收到 |

## 历史路径说明

2026-10-11 归档前，这些文件直接放在仓库根目录，对应关系：

```text
agent_communicate_reply.md       → 01_reply_v2_interface_to_server.md
agent_communicate.md             → 02_reply_from_server_stepA.md
agent_stepB_notice.md            → 03_stepB_notice_to_server.md
agent_communicate_reply_stepB.md → 04_reply_stepB_to_server.md
```

（`CHANGELOG.md` 里出现旧路径的条目是历史记录，不回溯修改。）

## 当前进展速览

```text
Step A（链路验收）      ✅ 通过：20 次往返 153–188 ms，会话复用正常，opos 不依赖
Step B（悬停只记录）    ✅ 链路/延迟/深度达标（327 次、均值 237 ms、深度 100% 有效）；
                        指向"贴底边且恒定"经服务器侧澄清为**预期行为**（apos=可通行落点，见 10）
Step B′（waypoints 复核）✅ 10 行/帧、符号自洽（rot_left ↔ yaw +0.314）、累积位姿取 [0]；
                        另实测到**输出强依赖会话历史**（冷启动前 3 帧一律 rot_left）
Step B（偏航扫场）      ✅ 通过：模型随视角输出 rot_left/rot_right/stop（非退化），
                        apos_x 59/244 帧偏离默认落点；两轮指令同一会话、frame_seq 连续
Step C（闭环飞行）      ✅ C1 只偏航：航向 −102.8°（方向正确、限幅 0.5 rad/s、无持续下降）
                        ✅ C2 加前进：上限 0.3 m/s（实测 0.12）、位移 1.66 m、30 s 到点强制归零
                        ✅ C3 注入验证：stale/ok/obstacle_slow/obstacle 逐段触发，左绕/右绕方向均正确
                        ✅ C3 追加三规则：宽侧小前向 0.10、两侧紧归零、转角 −60.1° 触发上限停转、
                           释放迟滞（侧向 <1.2 m 不释放）；geofence 与 duration 叠加正常
                        ✅ C3 第二步：真实避障触发保持已验证（c4，净空 0.15–0.39 m 时归零）；
                           顺带修掉"陈旧阈值 0.5 s < 地图周期×3"造成的 ok/stale 抖动（改 1.5 s）
                        ✅ 场景② + 修正（过滤 0.3 m / 保持三帧确认 / 非对称竖直窗）：**保持态已可达**
                           （c11：obstacle 3 帧 + avoid_side=right；yaw_lock 生效）
                        ⚠️ 查出**多栈并存**（pkill 只杀父进程）→ 新增 stop_stack.sh；c6–c9 数据作废
                        ✅ 近场读数定性：干净单栈下 **142/142 帧 nearest=inf** → 之前的跳变读数来自多栈环境，
                           地图本身无近场伪点；过滤改为"机体原点系 + 体素中心"0.45 m
                        ✅ 落地保护解耦：新增独立监督层 flight_supervisor.py（心跳 2 Hz，停 >5 s 且已解锁即
                           land_fast）——**实测触发过**；驱动脚本加异常兜底
                        ✅ 场景②完整链路（c13）：慢速区 → 三帧确认 → obstacle+选侧 → 释放 → 恢复前进
                           （yaw_lock 按设计按住转向；下一轮去掉即补上"朝宽侧转向"序列）
                        ✅ 世界系一致性判据 + 逐帧速度/俯仰落盘（37 项测试）
                        ✅ track（质心）+ 几何门限 + 单测（39 项）
                        ⚠️ 用 AirSim 真值选障碍遇阻：建筑不在 actor 列表，且 AirSim 世界系与
                           PX4 本地 ENU 存在 ~(3.4, −6.9, −0.9) m 固定偏移
                        ✅ 扫描变体（服务器侧确认）：扫描→静止复验→对准→逼近→交权 已实现；
                           实测 **32 s 扫满 360° 无真障碍**、所有候选均为**自运动拖影**
                        🐞 修掉监督层误触发（爬升超时真因）：解锁后加 6 s 宽限期，修后 0 次误触发
                        ✅ 转场+扫描+静止复验+逼近（①② 场景）：东 12 m 处候选被复验判为拖影；
                           北 15 m 处候选复验通过（真障碍）
                        ✅ **真实障碍上的完整避障序列**：压速 → 三帧确认 → obstacle 保持
                           + `avoid_side=left`（yaw +0.15）+ creep 0.05（c21）
                        ✅ 对准 + 逼近中持续复验 + 围栏放宽到 12 m 已实现（服务器侧两条建议）
                        ⚠️ 503 根因=我方 c20 节点无超时未退出（已清、已加 timeout）；
                           c21 自身流无缺号 → 该轮序列有效；c22 同点复跑未复现该障碍
                        ⏳ 多点自动扫描（二次复验 + 多点预算 + 转场）**已实现，未飞**（服务器侧同意）
                        ✅ 2026-10-14 恢复：多点扫描（二次复验 + 可闭合门限）实飞——
                           北15/东12/西−12 三点**均无可复现障碍**（北点第1轮 0.75m 通过、第2轮 4.67m 被拦）
                        ✅ 修掉 `_world_consistent` 未初始化导致地面段记录写盘失败的 bug
                        ✅ **完整绕行跑通（m4）**：depth_seek 定出"结构在北 23 m"→ 转场北 20 m →
                           二次复验真障碍（1.44→0.47 m 两轮通过）→ 对准 → 0.15 m/s 逼近 → 交权 →
                           同一次飞行里**两轮**"慢速区 → 三帧确认 → obstacle 保持 + 朝宽侧转向 +
                           creep → 视野打开 → 恢复前进"（guard: obstacle 14 / obstacle_slow 11）
                              打分校准器 analyze_detour.py：**4/4 episode 闭合 = 100% PASS**
                        ✅ 自动寻障（同趟飞行内 SEEK 阶段）机制跑通；发现距离估计混入地面 →
                           改为只取地平线以上行带（seek_row_fraction=0.35，待验证）
                        ✅ 几何去地面 SEEK + **fail-closed 起飞前检查脚本 preflight.sh**（含新 session 生成）
                        ✅ 稳健化 SEEK（点数 ≥40 + 垂跨度 ≥6° + MAD ≤1.5 m + 中位排序）+
                           多候选点 + re-align 上限换点：**自动链路一趟跑出三段闭合绕行**
                           （s1014-131724；打分：含单帧 3/4=75%、episode≥3 帧 3/3=100%）
                        ✅ 打分口径按其定稿固化（episode≥3帧且含保持态；瞬态单独计数）
                        ✅ ③ 连跑 5 次（脚本 run_detour_trial.sh：清单组/新 session/preflight/跑/打分）：
                           **轮级 4/5 = 80% PASS**（episode 级 5/6）；5 轮全部 preflight 通过、0 新增 503
                        🐞 trial4 失败签名：enable 后围栏未重新零点（guard 停在 geofence）
                        ✅ 已修三条：① enable **原子**重置围栏原点（打印新原点）② **无确认不交权**
                           （逼近超时未达停止阈值 → 作废该点换候选）③ dry_run 期间围栏/时长不生效
                        ⏸ **暂停（2026-10-14）**：我方本地环境已收尾（进程 0、18080 无监听）
                        ✅ 服务器侧已恢复且两项改动生效（`TRAJ_TOP1` 约束词表 + 会话回收 LRU）；
                           GPU0 出现新第三方作业（2208 MiB）→ 基线 ~13792 MiB，按噪声处理
                        ✅ 本地环境已恢复；**回归抓到 2 个真 bug**（ALIGN 缩进死代码→驱动卡死 11 min、
                           缺 import time）并修复；新增"卡阶段兜底"（监督层 phase_stuck_s / 驱动 max_flight_s）
                        ✅ 回归验证：0 次 enable（无确认不交权）、375 帧全 guard=dry_run、按预算收工降落
                        ✅ "逼近超时/净空回升→作废换点"分支实飞覆盖（regE184900：候选 3 复验通过→逼近中
                           净空回升→re-align 上限→作废→收工降落；332 帧全 dry_run、0 次 enable）
                        ✅ 阶段超时**按阶段分配**（phase_budgets：ALIGN 15 s、SEEK 45 s、TRANSIT 90 s…）+
                           阶段状态机**静态单测**（死代码检测 + 分支必须能推进，最小复现验证有效）→ vlm 41 passed
                        ✅ 修复后批次 ③ 连跑 5 次（t6–t10，全新 session + preflight 全通过）：
                           仅 **1/5 轮真正交权**（"无确认不交权"生效）；t6/t7/t9/t10 均**无 episode**（候选复验未过）
                        ⚠️ t8 暴露新问题：贴两侧都紧的墙面时"释放要选侧 ≥1.2 m"永不满足 → **保持 208 帧
                           （≈90 s）到窗口结束**（0 帧恢复，0/1 FAIL）
                        ✅ 已实现（**待飞验证**）：`hold_max_s`（默认 20 s）+ `escape_back_mps`（0.10 m/s）——
                           保持超时则**慢速后退 + 朝宽侧偏航**脱困（guard=`obstacle_escape`）
                        ✅ ② 速度指令仲裁已实现（`velocity_arbiter`：优先级 avoid>vlm、逐级回落、全部过期不发；
                           单测 20 passed + 运行时冒烟通过；`sim_stack.launch.py` `with_arbiter` 接入）——**未实飞**
                        ✅ 服务器侧已定口径（66）：① 寻障＝质量门限(点数/跨度/MAD≤1.0)+中位距离≥6 m 取最近；
                           ② 双指标（闭合率=闭合/有 episode 轮次，寻障命中率=有 episode/总轮次，无 episode 单独计数）；
                           ④ 脱困有界（`escape_max_s/m`）+ 重试≤1 + 记 `avoid_side`/次数
                        ✅ ①③④ 已按口径实现（驱动 SEEK 门限、analyze_detour 双指标、navigator 脱困有界+重试）
                        ✅ t11 实飞（410 帧，无 episode、全程安全落地）：新门限不误触发（未验证"选到真结构"）
                        ✅ **脱困实飞验证通过**（esc2：飞行中注入"两侧都紧的墙"→ 两次有界脱困
                           （10.1 s/1.01 m、10.0 s/1.00 m）→ 重试用尽放弃 → 注入解除后 episode 闭合 1/1）
                           44 帧 forward 全为 -0.10 m/s、escape_index 1/2、avoid_side=left 已入记录
                        ✅ 按服务器建议：脱困上限改 **B（预判式严格不越界）**；仲裁器补"每路 age + 落盘谁赢"
                           + 频率提到 20 Hz（同 offboard_bridge），并做了 **SITL 冒烟**（avoid→vlm→none 正确）
                        ✅ 寻障打印全扇区表；t12–t14 命中 0/3，但**门限不是瓶颈**（各轮筛出 4/8/6 个合格扇区）——
                           真瓶颈是"候选点→可保持障碍"（扫描近障碍在静止复验第 2 轮变 inf = 拖影；t14 唯一
                           通过复验的在 RUN 阶段因自运动被判拖影，从未进保持）
                        ✅ 服务器侧**选 B**：微转复验 + 位姿质量门 → 已实现并实飞：
                           · 脱困上限改 **B 预判式**，esc3 实测两次 0.91 m/9.1 s **≤ 1.00 m**（不再越界）
                           · 位姿质量门（position_age + 角速度 → defer）落盘：esc3 交权后 337 帧
                             pose_age≤0.02 s、|yaw|≤0.59 rad/s、**0 次误 defer**
                           · 交权前 ±15° 微转复验（世界点≤1.0 m + 距离差≤1.0 m）；抓到并修掉
                             NED/ENU 符号 bug（t15 反向转到超时、安全退出）；t16 正确否掉
                             "转一下就消失"的候选
                        ✅ 服务器侧指出**位姿门阈值是反的**（0.6 > RUN 0.5 死区）→ 已改 **0.45** + 配对规则入注释
                        ✅ **A 诊断已跑**（local_map 加只读 `nearest_wide`（±35°）+ 微转 ±10°）：候选 1 宽锥也 inf
                           =彻底消失；候选 2 宽锥有点但世界点漂 3.49 m = 非同一稳定结构
                        ✅ 服务器侧建议先做"限定的 ②"→ **簇质心已实现并验证**：c1 同一建筑面三次复验
                           1.37/1.02/0.97 m → 交权（对比"最近点"的 3.49 m，稳定性↑约 3.5 倍）
                        ⚠️ 交权后只到 0.75 m，未进保持（交权阈值 1.0 m vs 保持 0.6 m 之间有 0.4 m 缺口）
                        ✅ **仲裁实飞联调通过**：vlm →（planner 有目标）avoid 抢占 →（planner 停）回落 vlm
                           →（两路全停）none；并抓到并修掉**接线 bug**（launch 话题与仲裁器默认不一致 →
                           0 订阅者）与 stop_stack 漏清仲裁器（曾 2 实例）
                        ✅ 服务器侧两问都选 (a)：**交权 1.0→0.8 m**（保持门限 0.6 m 不动，安全参数）、
                           **复验容差 1.0→1.5 m**（保持"单次通过即算"）——已落地，单测 61 passed
                        ⏸ **暂停（2026-10-14 21:5x）**：服务器侧服务与监控已停止；`preflight` fail-closed
                           故**未起飞**；我方同步收尾（停 VLM 隧道 + ROS 栈，保留 PX4 SITL/Agent）
                        ✅ 2026-10-15 恢复 + ①(a)+②(a) 验证：d1（②(a) 容差 1.5 m 是必要的：
                           第三次复验 1.16 m，旧 1.0 会否交权）但**未进保持**；新发现"**压速带是吸引子**"
                           （压速把前向压到 ~0.000 → 跨不进 0.6）→ 新增保守参数
                           `obstacle_slow_min_forward_mps=0.05`（不覆盖显式停止、不动 0.6 阈值）
                        ✅ **d3 首次全链路闭合**：交权 → 保持 7 帧（avoid_side=left）→ 无脱困 → 恢复
                           200 帧 → episode 79 帧闭合；双指标 **1/1（100%）/ 1/1（100%）**；安全落地
                        ✅ d4–d7 继续跑：**d5 再次闭合**；d4 暴露"**保持↔释放抖动**"（净空 0.45–0.58、
                           每段 14–15 帧）→ 补 `obstacle_release_confirm_frames=3`（"进出双门限"，
                           连续 N 帧才释放、中断清零；单测 62 passed，**尚未飞验**）
                        📊 d3–d7 双指标：闭合 **3/5 = 60%**、命中 **3/5 = 60%**、无 episode 2/5
                           （对照：改前命中率累计 1/10）
                        ⏳ 待服务器侧确认：压速下限 0.05 / 释放确认 3 帧 / 先飞验 c 再做厚样本 a、
                           或先补 ② 仲裁回归 b
```

## 服务器侧提供的版本标识（归档）

```text
LightNav-0 仓库      commit 3015508b70fb6e30bbd66701e94c9b916dcb8df8（2026-09-17）
checkpoint          LightOriginsHQ/LightNav-0，权重 model-00001-of-00001.safetensors 9247 MB
                    config.json md5 af6a0040949c04a19491f0fc36de52ed
                    eval_config.json md5 c3064ba88efff0d60262743b683d0721
                    action_tokenizer/manifest.json md5 393f4f98214c608ac5a8a3489c457434
参数                video_size 256×448、fps 4、history 64(SlowFast)、horizon 10、RVQ 3×256
服务                server_http.py（接口 v2）绑 127.0.0.1:18080，conda env lightnav
重启语义            进程重启后同一 session_id 首帧自动新建会话；frame_seq 必须严格递增
```

## 关键结论（双方都需知道）

- 相机实际速率 **color 1.33 Hz / depth 1.78 Hz**（不是早先记的 2.25 Hz）；
- **`apos` 是"可通行落点"（往哪走），不是目标物体**：户外它落在画面底边中心即"向前走"，
  `apos_clamped=true` 与恒定值是**预期行为**（服务器侧用 480×270 合成帧复现了同一坐标）；
  由此反投影会算出"下降"，**无人机竖直分量必须归零、高度交飞控**（已实现 `vertical_mode='hold'`）；
- `opos` 通道存在但本场景无输出（恒 `not_visible`），对象锁定暂不使用；
- 转向优先用 `waypoints` 的 `yaw`（动作头输出），`apos.x` 只作水平方位与兜底；
- 远端不返回 `depth_m`，深度一律由**本机** AirSim 32FC1 深度在指向像素处采样补上；
- 首帧返回 `rot_left` 是正常现象，评估时不计入异常。
