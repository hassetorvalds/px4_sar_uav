> **历史交接记录**：保留原文；当前能力/未决项见 [INDEX](INDEX.md)、[RESULTS](RESULTS.md)、[OPEN_QUESTIONS](OPEN_QUESTIONS.md)。

# 下一轮从这里开始（最近更新：暂停于 2026-10-14）

## 最新（2026-10-15 深夜）：链路已全停；下一步走 **C**

```text
一键起停（新）：scripts/dev_chain.sh start | status | stop
  暂离流程：stop（WSL 可直接关）→ 回来：先起服务器侧服务 → start → run_detour_trial.sh <标签>
  当前状态：WSL 侧已全部停干净（0 残留）；Windows 侧 UE4/AirSim 由用户决定是否保留

服务器侧 87 号来信的结论（已收到）：**走 C**
  · 主线转 ②（VLM×local_planner 仲裁实飞联调）+ 回归
  · **闭合率统计改在"已知有结构的点位"上做**（像 m4 那样摆位后交权），这样 episode 可复现、n 才能做大
  · 自动寻障**命中率降为副指标**（照旧记录，但不作主线门禁）
  · 限速 0.5 → 0.8 **暂不动**（切到已知点位后转场都很短）；将来做长距离转场再上，并要求
    `transit_stop_clearance_m ≥ 制动距离`
  · 将来若回头攻命中率：优先"到达后小幅偏航重新捕获 + 按簇质心方位对准 + 转场点放更近（×0.6）"，
    而不是只调 scan_found 门限
  · 特别理由：最近的 `释放连续3帧确认`/`盲区底线`/`creep 双闸` **还没在真障碍的"保持→释放→恢复"闭合上验证过**
    （c1/d3/d5 都在这些改动之前）→ 已知点位回归正好验证它们

待转达：docs/agent_comm/84_*.md、86_*.md（87 是收到的来信）
待撤销（实验全部结束后）：Windows 端口静态保留 → scripts/win_port_exclusion.sh revert（README 4.6）
```

## 〇、2026-10-14 晚：暂停时的准确状态（先读这段）

> ⚠️ **2026-10-15 待撤销的环境改动**：为让 AirSim 启动（Windows 保留段 44986–45085 压住了
> `ApiServerPort=45000`），在 Windows 上把 45000 设成了**持久静态保留**
> （`scripts/win_port_exclusion.sh apply`，详见 README 第 4.6 节）。
> **实验结束、用户确认后必须执行 `scripts/win_port_exclusion.sh revert` 撤销。**

> ⚠️ **2026-10-15 更正：`scripts/restart_px4_sitl.sh` 不会把飞机在世界里搬回出生点。**
> 实测 AirSim 世界坐标仍在 `(北 23.4, 东 -22.6)`，而 PX4 只把**本地原点**重新设在当前位置
> （于是 PX4 报 x≈0,y≈0，看着像复位，其实没有）。它仍值得用（复位估计器/原点、清电池阈值），
> 但**每轮可比性**要靠：① 重启 AirSim 场景（世界回出生点），或 ② 每轮只读记录 AirSim 世界位姿用于归因。
> 另外：**不要在项目里调用 AirSim `reset()`**（会打断 lockstep；PX4 自带的启动复位是另一回事）。

> ⚠️ **2026-10-15 新增安全参数 `obstacle_blind_zone_m=0.5`（盲区底线）**：净空 ≤0.5 m 时前向一律归零
> （保持态也不再 creep），只保留偏航。**尚未飞验**；建议用 esc 式注入（2 m 悬停 + 注入 0.45 m）
> 验证"前向归零、偏航保留"，不要靠近真实结构。

### 2026-10-15 暂停状态（先读这段）

```text
为什么停：用户现场发现"测试中飞机多次贴近/疑似蹭到障碍物"，且 AirSim 并未复位。
         我方已停飞核查，并加了盲区底线（见上）。**下一批飞行前需要用户确认安全策略。**

已停（本机）：VLM 隧道、ROS 栈（bringup 及节点，含 velocity_arbiter）、flight_supervisor、
              以及所有 VLM/驱动/注入进程
仍在跑（本机）：PX4 SITL + Micro XRCE-DDS Agent（无人机已上锁、在地面）；
              AirSim/UE4 由用户决定是否保留

恢复前**建议先做**（用户/服务器侧待定）：
  1) **重启 AirSim 场景**（把飞机世界位置放回出生点）——注意：重启 PX4 只重设 PX4 本地原点，
     **不会**把飞机搬回出生点（2026-10-15 更正，见上）
  2) 确认盲区底线的验证方式：建议 esc 式注入（2 m 悬停 + 注入 f=0.45, l=r=0.4）
     期望"前向恒 0、偏航保留"→ 完全不靠近真实结构
  3) 决定保持态 creep（0.5~1.2 m 段仍会 creep）是否要更保守

恢复顺序（确认后照旧）：
  1) scripts/vlm_tunnel.sh start → 确认 /health ready
  2) scripts/restart_px4_sitl.sh（复位估计器/原点 + 清电池阈值）
  3) bash scripts/bringup.sh camera_rate_hz:=2.5
     setsid nohup python3 scripts/flight_supervisor.py --ros-args -p timeout_s:=5.0 -p arm_grace_s:=6.0 &
  4) scripts/preflight.sh（fail-closed 全过才飞）
  5) 跑 ③ 统计 / 盲区底线注入验证 / ② 仲裁回归

待办（与代码无关）
  · **转达**：docs/agent_comm/82_reply_blind_zone_safety_fix_and_pose_reset_correction.md（待转达）
  · **撤销**：Windows 端口静态保留（README 4.6；用户确认实验结束后 scripts/win_port_exclusion.sh revert）
  · 用户自行 push 到 GitHub（本轮提交见 git log）
```

### 2026-10-15 追加（同日稍晚）：**"高速撞障碍"真凶已定位并加固**

```text
真凶：**转场用位置目标点、没有任何限速** → PX4 以巡航速度（3–12 m/s）冲向 20–25 m 外的目标，
      路径中间隔着建筑就是高速撞击（d9 数据：0.5 s 内局部坐标跳 36 m = 位置阶跃签名）。
      SCAN/ALIGN/APPROACH/RUN 都是速度指令（0.15–0.3 m/s），不存在这个问题。

已落地（**均未飞验**；服务器侧服务现已停止）
  1) 转场限速 + 前方净空硬停：transit_speed_mps=1.0、transit_stop_clearance_m=1.0
     （每周期只推进 min(限速×dt, 剩余)；净空≤1.0 m 且距目标远 → 就地停住）
  2) 盲区覆盖所有前向路径（creep / 前向下限 / VLM forward）+ 触底标志
     local_map 新增 fwd_floor_m=；navigator 落盘 at_min_range / map_floor_m
  3) creep 双闸：creep_open_side_m 1.5→2.0；预算 creep_budget_m=0.5 / _s=10（超了只转不推）
  4) 固定起始世界位姿（不调 reset()）：start_pose_enabled/start_world_north/east，
     起飞后用 AirSim 只读位姿算差值、复用限速转场飞过去

恢复后验证顺序
  1) esc 式注入（2 m 悬停 + f=0.45, l=r=0.4）→ 期望前向恒 0、偏航保留 → 20 s 后脱困后撤；
     边界：f=0.55（应仍 creep）、VLM 显式 forward>0（应被盲区压成 0）
  2) 一轮常规试验：确认转场段地速 ≤1.0 m/s、"就地停住"日志
  3) 再按 ③ 连跑统计
待用户/服务器确认：转场限速 1.0 是否合适、固定起始点用哪个世界坐标、creep 预算 0.5 m/10 s
```

### 2026-10-15 晚间追加：转场改**速度控制** + "落地后不退出"修复（d13–d19 已飞验）

```text
今天最后一批的结论（都已飞验、全部安全落地）
  1) 转场：位置目标点(高速风险) → 位置追踪点(只有 0.11 m/s、到不了) → **速度控制**（最终）
     publish_velocity_enu：transit_speed_mps=0.5 精确限幅 + transit_yaw_rate 对齐 +
     transit_stop_clearance_m=1.0 硬停 + 贴地板读数"后退 0.25 m 判别自机体回波"
     d19：三次候选转场全部到位（+12.0/+10.3/+16.8 m），随后按预算正常收工
  2) "落地后终端一直 running"：两处真 bug，已修
     · 驱动新增安全闸：曾解锁→现在上锁 ⇒ 立即收工退出（d17 飞验：ERR 检测到飞机已上锁 → 收工）
     · SCAN 计时不再被复验失败重置（否则 SCAN↔ROTVERIFY 死循环）
     · scripts/run_detour_trial.sh 加了 timeout 900 硬超时兜底
  3) 参数对齐：驱动 transit_timeout 120→45 s（先于监督层 90 s 预算）；
     监督层 phase_budgets SCAN 35→120 s（否则正常扫描被误杀）
  4) 起始位姿：d14 修了"跳过 SEEK"（DESCEND 后回到深度寻障）；d17/d19 SEEK 正常（合格扇区 4–9）

仍待解决（下一轮重点）
  · 命中率：SEEK 能筛出合格大结构，但到达候选点后 SCAN 很少复验出"可保持障碍"（d18/d19 无 episode）
  · 转场遇真障碍：现在只"停住→45 s 超时→就地扫描"，**不做绕行**（已请服务器侧定）
  · 待确认：转场限速 0.5 → 1.0？SCAN 预算 120 s 是否接受？
```

```text
为什么停：服务器侧服务与监控**按操作者要求停止**；preflight 是 fail-closed（要远端 /health ready）
          → 本轮**未起飞**。本地也同步收尾。

已停（本机）：VLM 隧道（scripts/vlm_tunnel.sh status 应为"没有在跑的隧道"）、
              ROS 栈（bringup + offboard_bridge/airsim_camera/local_map/local_planner/velocity_arbiter）
仍在跑（本机）：PX4 SITL、Micro XRCE-DDS Agent（PX4 于 21:0x 重启过一次以接上重启后的 AirSim；
              电池阈值 BAT_LOW/CRIT/EMERGEN_THR 已清零，PX4 日志 /tmp/px4logs/px4.log）

恢复顺序
  1) 服务器侧起服务（带 VLN_EVAL_TRAJ_TOP1=1 + SESSION_TTL_S/MAX_SESSIONS）→ 确认 /health ready
  2) scripts/vlm_tunnel.sh start
  3) 确认 PX4（连 AirSim 4560）+ Agent 在线；脚本 run_px4_sitl.sh 会自动补 MAVLink 手动接管实例
  4) source scripts/env.sh && bash scripts/bringup.sh camera_rate_hz:=2.5   （含 velocity_arbiter）
     setsid nohup python3 scripts/flight_supervisor.py --ros-args -p timeout_s:=5.0 -p arm_grace_s:=6.0 &
  5) **待验证的两项**（服务器侧 2026-10-14 选 a，本轮只落地未飞）：
     ① pre_stop_clearance_m 1.0 → **0.8 m**（交权点贴近保持带 0.6 m；保持门限不动）
     ② rot_tol_m / rot_dist_tol_m 1.0 → **1.5 m**（微转复验容差；次数仍是"单次通过即算"）
     → 各飞一轮：run_detour_trial.sh d1 / d2，看交权后能否较快跨过 0.6 m 进保持、拿到 episode
  6) 拿到 episode 后按 ③ 双指标连跑统计（analyze_detour.py 支持多轮聚合）

本日已验证的关键能力（都有飞行取证）
  · 脱困（保持超时）：**有界 + 重试≤1 + 记数**；esc3 两次 0.91 m/9.1 s ≤ 1.00 m，用尽后放弃
  · 位姿质量门：位置陈旧>0.5 s 或 |yaw|>0.45 rad/s → defer（暂缓判定）；每帧落盘 pose_age/yaw
  · 交权前 ±15° 微转复验（世界点/距离一致性）；**簇质心**为世界点基元
    → 同一大面漂移 3.49 m（最近点）→ ~1.0 m（质心），c1 第三次复验 0.97 m 通过并交权
  · 速度仲裁（velocity_arbiter）：**实飞联调通过**（vlm → avoid 抢占 → 回落 vlm → none）；
    bridge 速度话题现为 1 发布者（仲裁器）+ 1 订阅者
  · 全程 0 次误交权、0 次误触发围栏、全部安全落地

本日踩到并修掉的坑（都已进 CHANGELOG）
  · 微转复验用 NED 解卷绕角 → 方向相反（越转越远）；改 ENU 口径
  · 仲裁器输入话题与 launch 里发布者话题不一致 → **0 订阅者**，planner 速度被丢
  · stop_stack.sh 漏清 velocity_arbiter → 曾同时 2 个仲裁器实例
  · ROT_DEG=10 这种整数写法会被 ROS 当成 INTEGER（参数是 DOUBLE）→ 节点构造即崩（未起飞）
  · `ros2 topic echo` 默认截断长字符串，核对 `nearest_*` 字段要加 `--full-length`
```

## 一、当前状态一览

```text
已实现并验证（有飞行取证）
  px4_interface / sensor_bridge / mapping / planner / mission / vlm / bringup  全链路可用
  手动接管（WASD/F9/F10）、offboard 丢失策略（保持/RTL）、land_fast 收尾
  VLM 远端联调：Step A/B 通过；waypoints/apos 语义已确认；竖直分量不交视觉
  闭环：C1 只偏航、C2 加前进、C3 注入验证、真实避障序列（c21，单侧可用）
  安全层：fail-safe 陈旧、三帧确认、世界系 track、几何门限、迟滞、偏航覆盖、
          转角上限、宽侧小前向、对准（ALIGN）、逼近中持续复验、围栏/时长包络
  运维：stop_stack.sh（父子同停）、flight_supervisor.py（心跳落地，含解锁宽限期）、
        vlm_tunnel.sh、analyze_stepB / analyze_yaw_sweep / vlm_session_to_csv

2026-10-14 的进展（重要）
  · **完整绕行跑通**（m4，手工目标点）：真实障碍上两轮"保持→朝宽侧转向→视野打开→恢复前进"，
    打分 4/4 = 100%、服务端 0×503
  · **自动链路成型**：depth_seek/几何去地面 SEEK（四重门限：点数/垂直跨度/MAD/中位）
    → 多候选转场 → 二次复验 → 对准 → 逼近 → 交权 → 绕行
  · **纪律脚本化**：`scripts/preflight.sh`（fail-closed，7 项各 1 + 链路 + 地图 + 远端 health）
    + 每趟**全新 session id** → 连续多轮 0 新增 503 / 0 会话重置
  · **③ 连跑 5 次**：轮级 4/5 = 80%，episode 级 5/6 = 83%（判据：episode ≥3 帧且含保持态；
    结束后 20 帧内 ≥5 帧前向非零 = 闭合）
  · 一键试验：`scripts/run_detour_trial.sh [标签]`（清节点→新 session→单组 VLM→preflight→跑→打分）
  · `scripts/analyze_detour.py`：按定稿口径打分（含瞬态/仅慢速单独计数）

待完成
  ① trial4 回归：三条修复后（enable 原子重置围栏 / 无确认不交权 / dry_run 不执法）重跑同一布局
  ③ 连跑 ≥5 次统计（用 run_detour_trial.sh）
  ② C3 集成：VLM 与 local_planner 都写同一速度话题，需要仲裁（安全/规划层覆盖 VLM、同帧同坐标系）
  ③ 降落偏差（AUTO_LAND 估计器发散）仍用 land_fast 规避；长期方案待定

标注（诚实）
  c21 的避障序列：单侧可用（保持+转向已验证；完整绕行待复现）
  c6–c9 的净空/摆位数据作废（当时多栈污染）
```

## 二、恢复步骤（按顺序）

```text
1) 服务端（由服务器操作者执行）
   # 2026-10-14 之后启动请带上两项改动：
   #   ① VLN_EVAL_TRAJ_TOP1=1（把生成约束到动作词表，消除 0.1% 畸形 500）
   #   ② 会话回收 SESSION_TTL_S=900 / MAX_SESSIONS=8（避免会话堆积把显存推过 +4 GiB 告警线）
   source /home/sol/miniforge3/etc/profile.d/conda.sh && conda activate lightnav
   cd /home/sol/lightnav-workspace
   VLN_EVAL_TRAJ_TOP1=1 setsid nohup python server_http.py --model_path /home/sol/lightnav-workspace/LightNav-0/checkpoints/LightNav-0 > server_http.log 2>&1 &
   curl -s 127.0.0.1:18080/health      # 期望 ready:true

2) WSL 侧隧道
   scripts/vlm_tunnel.sh start         # 幂等；会打印 /health

3) PX4 链路（建议连续实验间隔重启 PX4）
   scripts/run_agent.sh &              # Micro XRCE-DDS Agent
   scripts/run_px4_sitl.sh &           # 连 AirSim 4560
   # 在 pxh 里清零电池阈值（否则 failsafe 干扰）：
   param set BAT_LOW_THR 0 / BAT_CRIT_THR 0 / BAT_EMERGEN_THR 0

4) ROS 栈（只此一份！）
   scripts/stop_stack.sh               # 先确认没有残留
   bash scripts/bringup.sh camera_rate_hz:=2.5 &
   检查：bridge/cam/map 各 1 个；/fmu/out/vehicle_local_position_v1 ~50 Hz；地图 frames>0

5) 落地监督层
   setsid nohup python3 scripts/flight_supervisor.py --ros-args -p timeout_s:=5.0 -p arm_grace_s:=6.0 &

6) 直接跑"一键试验"（含清节点、新 session、单组 VLM、preflight、寻障+绕行、打分）
   bash scripts/run_detour_trial.sh trialN
```

## 三、起飞前检查清单（务必逐条）

```text
[ ] 服务 /health ready:true（GPU0 若有 hsy 的 UnrealEditor，读数按噪声源处理）
[ ] 隧道在线（scripts/vlm_tunnel.sh status）
[ ] PX4 与 Agent 在线，/fmu/out 有数据（~50 Hz），电池阈值已清零
[ ] ROS 栈单实例：bridge/cam/map 各 1，地图 frames>0
[ ] 监督层在线（含 arm_grace_s=6）
[ ] **只有一个 VLM 节点，且带 timeout**（本轮 503 的根因就是这条没守住）
[ ] 无人时不动 UE4；实验结束 land_fast + 确认 landed/arming_state=1
```

## 四、下一轮第一件事

```text
跑多点扫描（scan_points="0,15;12,0;0,-15;-12,0"，verify_rounds=2），
命中第一个"二次复验通过"的障碍点后自动对准→0.1 m/s 逼近→交权，
目标拿到完整"保持 → 朝宽侧转向 → 视野打开 → 恢复前进"序列；
四点全空则按预算收工，转 C3 集成（VLM 与 local_planner 仲裁）。
```
