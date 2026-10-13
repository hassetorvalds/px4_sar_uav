# 下一轮从这里开始（暂停于 2026-10-13 晚）

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

待完成
  ① 多点自动扫描找"可复现障碍" → 完整"保持→绕行→恢复"（代码已就绪，未飞）
  ② C3 集成：VLM 与 local_planner 都写同一速度话题，需要仲裁（否则互相抢）
  ③ 降落偏差（AUTO_LAND 估计器发散）仍用 land_fast 规避；长期方案待定

标注（诚实）
  c21 的避障序列：单侧可用（保持+转向已验证；完整绕行待复现）
  c6–c9 的净空/摆位数据作废（当时多栈污染）
```

## 二、恢复步骤（按顺序）

```text
1) 服务端（由服务器操作者执行）
   source /home/sol/miniforge3/etc/profile.d/conda.sh && conda activate lightnav
   cd /home/sol/lightnav-workspace
   setsid nohup python server_http.py --model_path /home/sol/lightnav-workspace/LightNav-0/checkpoints/LightNav-0 > server_http.log 2>&1 &
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

6) VLM 节点（**必须带 timeout**，且只一个）
   timeout -s INT 400 ros2 run vlm vlm_navigator --ros-args \
     -p provider:=remote_http -p dry_run:=true -p session_id:=cN -p geofence_max_distance_m:=12.0 ...

7) 驱动（多点扫描 + 对准 + 逼近 + 交权）
   python3 scripts/vlm_closed_loop.py --ros-args -p pre_position:=true -p pre_scan:=true \
     -p scan_points:="0,15;12,0;0,-15;-12,0" -p verify_s:=2.5 -p verify_rounds:=2 \
     -p align_tolerance_deg:=8.0 -p pre_speed_mps:=0.10 -p pre_stop_clearance_m:=1.2 \
     -p output_csv:=logs/2026-10-14/detour.csv
   # 交权时脚本会自动向 /vlm_navigator/enable 发 enable（解除 dry_run 并重置包络/围栏原点）
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
