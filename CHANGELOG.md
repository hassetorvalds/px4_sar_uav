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
