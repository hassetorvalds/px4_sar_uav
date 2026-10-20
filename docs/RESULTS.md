# 结果与证据（截至 2026-10-17）

> 这份文档只回答一个问题：**哪些能力是真的跑过的，证据在哪里，哪些还没有。**
> 规划与架构见 [README.md](../README.md)，逐条变更见 [CHANGELOG.md](../CHANGELOG.md)，
> 与服务器侧（LightNav-0）的往来定案见 [docs/agent_comm/](agent_comm/README.md)。

## 0. 证据口径（先读）

- **"已验证"** = 有 SITL 飞行日志/CSV 或单元测试作为证据，不是"代码写了"。
- 试验记录的口径**按版本分段**，不同批次的数字**不可直接相加**（改过判据/参数）。
- 样本量小的地方都标了原始计数；n 小的时候百分比没有统计意义（Wilson CI 会很宽）。
- 原始数据不入库（`.gitignore` 排除 `logs/`、`archive/`），本地保留约 **100 轮试验 CSV**。

## 1. 一句话状态

已经打通 **"深度寻障 → 转场 → 二次复验 → 对准 → 逼近 → 交权 VLM → 保持/绕行 → 恢复"** 的
完整闭环，并配有四层安全兜底与 fail-closed 起飞门禁；**尚未具备**人员检测、任务级语言解析、
区域覆盖搜索与语义地图。定位是**自主飞行 + 反应式避障平台原型**，不是完整搜救系统。

## 2. 能力矩阵

| 能力 | 状态 | 关键证据 |
|---|---|---|
| AirSim ↔ PX4 SITL ↔ uXRCE-DDS ↔ ROS 2 | ✅ | `Simulator connected on TCP port 4560`、`uxrce_dds_client synchronized` |
| Offboard / ARM / 悬停 / 航点状态机 | ✅ | 5 m 方形航迹误差 0.15/0.26/0.44/0.45/0.46 m（判定 0.5 m） |
| 相机 / 深度 / 点云接入 | ✅ | RGB 1280×720、深度 32FC1、点云 ~9440 点；受 LockStep 每 RPC ~330 ms 限制，约 1.3–2.2 Hz |
| 人工接管（键盘 → MAVLink） | ✅ | F9 切 POSCTL、F10 交还 OFFBOARD；任务 PAUSED → 恢复当前航点 |
| VLM 远端闭环（LightNav-0） | ✅ | 接口 v2、`meta.frame_seq` 严格递增、`waypoints` 优先、竖直分量恒 0；Step A 20 次往返 153–188 ms，Step B 327 次均值 237 ms |
| 自动寻障 SEEK | ✅（概率性） | 深度图定方位 + 几何去地面（点数/垂跨度/MAD/中位距离）+ 多候选；命中率仍约五成 |
| 交权前复验 | ✅ | 二次复验 + ±15° 微转 + 世界系簇质心；真大面三次复验 1.37/1.02/0.97 m |
| 反应式避障闭环 | ✅ | 真实障碍上"压速 → 三帧确认 → 保持 + 朝宽侧转 + creep → 视野打开 → 恢复前进" |
| 速度仲裁 | ✅ | `velocity_arbiter`：`avoid > vlm`，过期逐级回落；实飞联调 `vlm → avoid → vlm → none` |
| 四层安全兜底 | ✅ | 机载看门狗 → 驱动阶段超时 → `flight_supervisor` → 全局 `max_flight_s`；sup2 端到端触发 |
| fail-closed 起飞门禁 | ✅ | `preflight.sh` 7 项各 1 + `/fmu/out` + 地图 frames>0 + 远端 `/health`；曾拦下 local_map 崩溃 |
| 单元测试 | ✅ | **257 passed / 2 skipped**（vlm 45 / mission 125 / perception 28+1skip / px4_interface 20 / planner 18 / mapping 20 / px4_offboard 1+1skip）；`PYTHONNOUSERSITE=1` 时 258 passed |
| 自动降落 | ⚠️ 兜底可用 | EKF z 触地发散 → `land_fast` / `land_autodisarm` 按时间兜底；**实机不可沿用** |
| 已知点位可复现试验 | ✅ | 固定起始世界位姿（不调 `reset()`）+ 世界坐标点位；同一 layout 可连跑 |
| 传感器时间同步 / LiDAR | ❌ | 未做 |
| 区域覆盖搜索 | ✅ 小半径真机飞通 | `mission/search_pattern.py`：弓字形；东南侧 r=3 m、8/8 航点到位（7–75 s），无 abort；大区域未飞 |
| 原地转向 / 扫描（yaw/scan） | ✅ 真机飞通 | `yaw`（±1–360°）+ `scan`（1–120 s）；真机 40 s 扫描，`yaw_complete/scan_complete` 正常 |
| 搜索→检测→悬停→上报 | ✅ 组合闭环 | 搜索 8/8 + 步进扫描 30 s；`target_confirmed world=[27.41, 30.66, 4.95]`（t=76.1），scan 完成、安全落地；无 abort |
| 目标自动对准 | ✅ 真机验证 | 确认后 yaw 到世界点，`target_centered error=-3.4°`；search hold 不偏航 |
| Markdown 证据页 | ✅ | `mission/report.py` 生成 `report_*.md`：健康检查/目标/命令/时间线 |
| ROS 2 Action 接口 | ✅ 假 PX4 + 真机 | `ExecuteTask`：TaskSpec goal、阶段/进度 feedback、report result、cancel 落地上锁；真机完整搜索+检测 `success=True` |
| Action 完整任务 | ✅ 真机 | 搜索 8/8 → 扫描 60 s → 目标 `[29.19,30.78,4.87]` → 对准 3.5° → 落地；result `success=True` |
| 结构化发现物（findings） | ✅ 真机 | Finding 1.0：字段/ROS 时间/30s 窗口/按 label 半径/`discarded`/history；`frame_seq=10`、`frame_size=1280×720` |
| 覆盖规划（矩形/多边形） | ✅ 真机 | 矩形/旋转/任意多边形弓字 + 足印/重叠；L 形 8/8 航点、20 m²、重叠 50% |
| 观测覆盖（真实点云） | ✅ 下视 + 敏感性 | 前视 0/24 → 下视 24/24；L 形 20/20；2 m/间距 4 m 97.2%；见 [COVERAGE](COVERAGE.md) |
| 多区域 + 转场 | ✅ 真机 | L 形 20 m² + 矩形 16 m²，转场 10.86 m，合计 144/144（100%） |
| 区域最近邻排序 | ✅ 单测 | `--order-regions`，保留 takeoff/land；转场仍直线 |
| SemanticIntent 1.0 | ✅ 真机 | 手写 JSON → TaskSpec → ExecuteTask Action；area_cover 8/8、36 m²、144/144 |
| arm 前健康检查 | ✅ 真机验证 | 状态/位置未就绪时拒绝 arm；就绪后 arm ACK 0.008 s；5 项检查写入 `health_check` |
| 自检隔离（域/互锁） | ✅ 真机在线时验证 | 独立 `ROS_DOMAIN_ID=77` + 互锁；真实栈在线时默认 exit 2，允许并跑时 simcheck PASS 且真机保持 disarmed |
| 旧驱动高度门帧标签 | ⚠️ 已加标签，未复飞 | CSV `px4_frame/truth_frame/gate_frame`；TRUTH 仅诊断；数据源仍 px4/airsim 二选一 |
| 自然语言 → 任务（规则解析） | ✅ 规则解析 + 意图帧；VLM 未接 | `mission/task_parser.py` + `intent.py`：中英/全角/中文数字；`IntentFrame` 带 provenance/置信度/缺失；**本地事实优先**合并 VLM（`--remote-intent` 已验证拒绝改写） |
| 通用任务执行（起飞/航点/悬停/返航/搜索） | ✅ 最小/前进/返航/小半径搜索均已飞验 | `task_executor.py` + `executor_core.py`：围栏/净空门/总时限/接管暂停；**命令 ACK 生命周期**；假 PX4 自检 4 场景 PASS |
| 语义地图 / 语言—空间定位 | ❌ | 未开始 |
| 检测内核 + 世界坐标反投影 | ⚠️ 已实现，未上机 | `perception`：color/hog/haar 三种 backend；像素+深度→ENU；多帧关联（世界距离/IoU + N 帧确认） |
| 人员检测（真实人物） | ❌ | 场景内没有人物；HOG 在 60 张已存帧上 **0/60** |
| 目标跟踪 / 搜救报告 | ⚠️ 跟踪内核已测，报告未做 | `perception/tracking.py`：确认目标（默认连续 3 帧、世界 2 m 门限、2 s 过期） |
| 高度帧（REL/LCL/BARO/MSL/TRUTH） | ✅ 已进 schema/报告 | `mission/altitude.py`；`TaskSpec.altitude_frame`；报告 `altitude_lcl`/`task_altitude`；TRUTH 仅诊断且被拒绝作为任务帧 |
| 命令 ACK 生命周期 | ✅ 假 PX4 + 真机复飞通过 | `mission/command_tracker.py`：awaiting_ack→accepted→completed / timeout→delivery_unknown；真机 arm ACK 0.011 s、offboard 0.016 s；前两次首条 arm 超时后重试成功 |

## 3. 量化结果（按批次，不跨版本合并）

| 批次 | 日期 | 轮次 | 可绕闭合 | 快速闭合（20 帧） | 寻障命中 | 备注 |
|---|---|---|---|---|---|---|
| m4 手工点位 | 10-14 | 4 episode | **4/4** | — | — | 首次完整绕行；打分器校准 |
| ③ 连跑 | 10-14 | 5 | **4/5 = 80%** | — | — | episode 级 5/6 |
| d3–d7 | 10-15 | 5 | **3/5 = 60%** | — | 3/5 | d3 首次全链路闭合 |
| 修复后冻结批 | 10-16 | 10 | **5/5 = 100%** | 4/5 | 5/10 | 只计"可绕"；不可绕 0 |
| Batch1 | 10-16/17 | 5 | **1/1** | 1/1 | — | 累计报 8/8，Wilson [0.68, 1.00] |
| t3（持久 track） | 10-17 | 1 | **1/1** | 15/50、11/20 | 1/1 | 60 s 逼近预算下首次到位 |

口径说明：

- **可绕闭合率** = 闭合轮次 / 有 episode 且判定"可绕"的轮次（主指标）；
- **快速闭合** = episode 结束后 20 帧内 ≥5 帧前向非零（子指标）；
- **寻障命中率** = 有 episode 的轮次 / 总轮次（服务器侧已定为**副指标**）；
- "无 episode"单独计数，不计 FAIL。

> n 最大也只有 10，Wilson 95% CI 下界约 0.68；**不要对外宣称"100% 成功率"**。

## 3.5 离线检测评估（2026-10-18，perception 包）

用已存的 1245 张 AirSim 帧抽样，**不上机**先做回归（`scripts/eval_detector.py`）：

| 检测器 | 帧数 | 有检出帧 | 单帧耗时 mean | 结论 |
|---|---|---|---|---|
| `color`（橙色信标） | 60 | **19（31.7%）** | 57 ms | 稳定命中场景里的橙色球（bbox 固定在右侧），可作"目标物"demo |
| `hog`（行人） | 60 | **0（0%）** | 8.9 ms | 场景里**没有人**，符合预期；不能据此判断检测器好坏 |

说明：

- 颜色 backend 不依赖 OpenCV；HOG/Haar 依赖 cv2（本机需 `PYTHONNOUSERSITE=1`，见 `scripts/run_detector.sh`）。
- 检出率低是因为橙色球只在部分视角/位置入画，不是漏检。
- **离线上机前的结论只有两条**：管线能跑、耗时够（相机 1.3 Hz，检测 57 ms）；
  "能不能找到人"必须在有人的场景里重新评估。

## 4. 安全验证（案例集见 [safety_casebook.md](safety_casebook.md)）

| 场景 | 结果 |
|---|---|
| 高速转场撞障碍 | 已定位真因（位置目标点被 PX4 按巡航速度执行）→ 改速度控制 + 净空硬停；d19 三次转场全部到位 |
| 传感器盲区仍前推 | 已加盲区底线（净空 ≤0.5 m 前向归零、保留偏航）；后续真实闭合轮次验证 |
| 转角封顶仍硬推 | 封顶即前向归零；修复后 193 个 `yaw_capped` 帧里前向非零 = 0 |
| 持续不可绕 | sup2：保持 >20 s → 脱困×2（0.92/0.91 m，各约 9 s）→ 重试用尽不再后退 → 监督层 RUN 151 s > 150 s → `land_fast` → 安全落地 |
| 脱困越界 | 预判式上限：esc3 两次 0.91 m/9.1 s ≤ 1.00 m |
| 误交权 / 误触发围栏 | 多批次记录 0 次误交权、0 次误围栏（逐轮 CSV 可查） |
| 落地后进程不退出 | 修两处真 bug（驱动"曾解锁→上锁即收工"、SCAN 计时不重置）+ 脚本 `timeout 900` |

## 5. 工程数据

```text
提交          dev 分支 116 commits（main 待同步，见 README 顶部说明）
ROS 2 包      px4_interface / mission / sensor_bridge / mapping / planner / perception / vlm / bringup
自研源码      ~8800 行 Python（不含 px4_msgs 与第三方）
单元测试      ~3700 行，257 passed / 2 skipped（检测器入口下 258 passed）
试验数据      ~100 轮 detour CSV + 1245 张 VLM 决策帧（本地 archive/，不入库）
主驱动        scripts/vlm_closed_loop.py（1530 行；阶段机 SEEK/TRANSIT/DESCEND/ALIGN/
              APPROACH/SCAN/ROTVERIFY/VERIFY/RUN/CLIMB）
```

## 6. 已知限制（诚实清单）

1. **EKF 高度漂移**：AUTO_LAND 时 z 估计发散（真值约 4 s 触地，估计仍报 +1.8 m）；现在还会污染
   高度门与世界系 z（c1：交权点 ENU 上 +0.1 m，`px4_z` 在 −3.52~+0.47 漂移）。高度门数据源待定。
2. **自动寻障命中率约五成**：能筛出合格大结构，但"到点后能复验出可保持障碍"仍是瓶颈。
3. **自动降落不是真降落**：现有收尾按时间判定，只适用于本仿真。
4. **主驱动是单文件脚本**：单元测试覆盖叶子模块，覆盖不到这条主状态机。
5. **强依赖三个易碎环节**：Windows AirSim、WSL2 本机、远程 GPU 服务 + SSH 隧道；任一掉线即停摆。
6. **公开仓库的 `main` 长期停在初始提交**：外部评审只看默认分支，会得到与事实相反的结论。

## 7. 复现入口

```bash
# 0) 前置：Windows 侧 AirSim 已运行；远程 LightNav-0 服务已启动
# 1) 一键起链路（隧道 → Agent → PX4 → ROS 栈 → 监督层，并自检）
scripts/dev_chain.sh start

# 2) 单元测试
source scripts/env.sh
for p in px4_interface mission mapping planner vlm; do
  ( cd ros2_px4_ws/src/$p && python3 -m pytest -q )
done

# 3) 一轮已知点位试验（清节点 → 新 session → preflight → 跑 → 打分）
bash scripts/run_detour_trial.sh t1
python3 scripts/analyze_detour.py logs/<日期>/t1_*.csv

# 4) 收工
scripts/dev_chain.sh stop
```

## 8. 下一步（当前方向）

按 2026-10-18 决策：**先做"自然语言指挥飞行"的通用基底，人员检测/上层应用往后放**。

- 语言层：`mission/task_parser.py` 已能把中英指令解析成可校验的 `TaskSpec`（规则版，
  后续换 VLM 只替换这一层）；`mission/task_spec.py` 做 schema/范围/顺序校验。
- 执行层：`mission/task_executor.py` + `executor_core.py` 已实现
  起飞/航点/前向/悬停/搜索/返航/降落，带围栏、前向净空门、总时限、人工接管暂停。
- **待飞验**：等 AirSim 4560 恢复后，按 [docs/DEMO.md](DEMO.md) Step 2 → Step 3 验收。
- 之后：感知（人员/目标）接入执行器 → 语义地图 → 真正的搜救任务。
