# README 旧“未实现/已知问题/下一步”原文（保留）

> 当前状态见 README 第 4.4/4.5/22 节与 [INDEX](INDEX.md)。

## 4.4 尚未实现（2026-10-18）

## 4.4 尚未实现

```text
□ 自动降落（根因已定位：EKF 触地发散；已加静置上锁兜底，见 4.5）
□ 人工接管与安全：键盘接管、任务暂停/恢复、隔离验证、offboard 丢失行为、failsafe 动作感知语义（保持类→暂停续飞，恢复类→中止）均已通过
□ 传感器接入：相机/深度/点云已完成（深度 2.25 Hz，受 LockStep 每次 RPC 330 ms 限制）；LiDAR 与时间同步待做
□ 实时避障与局部重规划
□ 自主探索 / 搜索区域覆盖
□ VLM 自然语言任务解析（当前只有“指向式导航”，尚未做任务级结构化输出）
□ 语义地图与语言—空间定位
□ 人员检测、跟踪、疑似被困人员确认
```



## 4.5 已知问题（旧）

## 4.5 已知问题

```text
自动降落未通过：下发 land 后进入 AUTO_LAND，但 40 s 内未落地（任务按超时中止）。
2026-09-16 现场读到的 vehicle_land_detected：
  in_ground_effect=true, in_descend=true, close_to_ground_or_skipped_check=true,
  has_low_throttle=false, ground_contact=false, at_rest=false, landed=false
即飞机在近地面悬停而非触地，落地检测链路无法置位。
取证日志：logs/2026-09-14/10_54_06.ulg、logs/2026-09-16/05_45_12.ulg
处理计划：路线图阶段 A 首要调试项（分析 .ulg，必要时改用近地判定 + 显式 disarm 收尾）

2026-09-19 更新：根因是**高度估计漂移**而非落地检测本身——AUTO_LAND 实测下降速率仅约 0.04 m/s
（3 m 用约 80 s），期间 EKF 的 z 估计在 -2.8 ~ +2.0 m 之间摆动；PX4 最终确实报出
Landing detected 并自动上锁。仿真时间倍率实测约 1.0，排除“仿真慢于墙钟”的可能。
当前用 `land_fast` 通道兜底（按高度定时下降 + 强制上锁，实测 2.63 m 用 7.3 s 完成），
但它按时间而非真实触地判定，实机不可用。

2026-10-08 更新（根因进一步定位）：用 `.ulg` 解析 + AirSim 真值对照后确认——
**飞机在 AUTO_LAND 后约 4 秒就已实际触地并静止**（真值 z 稳定在 0.68 m），
而 PX4 的高度估计继续漂到 +1.8 m、垂速估计谎报约 +1.4 m/s，
落地检测基于这个发散的速度估计判断“是否静止”，因此永远无法置位。
关闭气压融合（EKF2_BARO_CTRL=0）后现象完全一致，已排除气压计因素。
当前收尾方案：`offboard_bridge` 的 `land_autodisarm_s`（默认 10 s）在 AUTO_LAND 后静置到时间即强制上锁，
实测 t=10.3 s 上锁、任务可正常收尾；该方案按时间判定，**仅适用于本仿真，实机不可直接沿用**。

航段速度偏慢：5 m 直线航段耗时约 15–19 s（约 0.3 m/s），
远低于 PX4 默认 MPC_XY_VEL_MAX，需确认限制来自轨迹生成还是 AirSim 锁步时间。

VLM 指向式导航：静态图像下目标点不动、偏航不收敛，机体会持续转圈；
闭环收敛需要真实相机图像（下一步 sensor_bridge）。真实 VLM API 尚未验证（本机无 key）。
参考实现 SeePointFly 为 Proprietary 许可，本项目只借鉴思路、自写代码与提示词
（见 docs/reference-see-point-fly.md）。

2026-09-21 更新：已接入实时相机（sensor_bridge）并跑通实时图像闭环——
指向像素随图像移动、偏航指令随之衰减与反转。当前偏航纯比例且无死区，会来回震荡，
下一步按 SeePointFly 的做法加死区并降增益。真实 VLM API 仍未验证（本机无 key）。
```

---



## 22 当前下一步（旧）

# 22. 当前下一步

当前已经完成：

```text
✅ PX4 + AirSim
✅ ROS 2
✅ uXRCE-DDS
✅ Offboard
✅ ARM
✅ 3 m Hover
✅ 航点状态机 + 5 m 方形航迹
```

当前进行中：

```text
⏳ 自动降落（AUTO_LAND 后近地悬停不触地，见 4.5 与 docs/roadmap.md）
```

后续步骤：

### Step 1（已完成）

实现：

```text
5 m 方形航迹
```

验证：

```text
多航点 + waypoint state machine
```

### Step 2

状态：进行中。

实现：

```text
自动降落
```

### Step 3

实现：

```text
自主飞行 ↔ 人工接管
```

### Step 4

加入：

```text
RGB-D / Depth / LiDAR
```

### Step 5

加入：

```text
Local Planner + 实时避障
```

### Step 6

加入：

```text
自主探索 / FUEL 思路
```

### Step 7

加入：

```text
VLM Natural Language Mission
```

### Step 8

加入：

```text
Semantic Mapping / VLMaps 思路
```

### Step 9

最后加入：

```text
Person Detection
+
Tracking
+
VLM Verification
```

---

