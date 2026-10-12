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
                        ⏳ C3 接本机避障层（设计已发服务器侧确认阈值）
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
