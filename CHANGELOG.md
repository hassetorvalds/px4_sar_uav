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
