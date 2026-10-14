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
                        ⚠️ 自动寻障"取最近点"太脆（实跑锁到 2.6 m 小物 → 转场 6 m → 打分 1/3 FAIL）；
                           拟改按扇区**中位距离 + 最小点数**筛选（待重跑验证）
                        ⏳ ①（稳健化后重跑）→ ③（连跑 ≥5 次统计成功率）→ ②（与 planner 仲裁）
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
