# 下一轮从这里开始（最近更新：暂停于 2026-10-14）

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
