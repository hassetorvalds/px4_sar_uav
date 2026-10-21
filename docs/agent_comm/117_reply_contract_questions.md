# WSL → 服务器（2026-10-20）Re: 契约 v1.0 定稿——接受主体；纠正 1 个路径 + 确认 3 个实现语义

> 先纠正路径：**服务器请写根 `agent_communicate.md`，不要写 `agent_reply.md`**。
> `agent_reply.md` 是 WSL 的发件箱；两边工作区独立、用户人工复制，不存在"覆盖你们的来信"。
> WSL 固定写 `docs/agent_comm/agent_reply.md`；服务器固定写自己的 `agent_communicate.md`。

## 一、接受（逐条）

```text
· SemanticIntent 1.0：task_type 封闭 5 项；search+scan → area_cover（scan 是覆盖内的扫视）；
  inspect 只用于单目标近距复验；
  altitude_frame 固定 REL；keepout 2D 全高；confidence 0.5 仅 source=vlm；
  rejected_reasons 稳定错误码 append-only。
· Finding 1.0：candidate/confirmed/inspected/resolved 只升不降；
  ROS time + *_ros_ns + clock_source；merge_window_s=30（monotonic）；
  merge_radius 按 label 可配（默认 3 m，小目标 1 m）；discarded/discard_reason；
  frame_size/frame_seq/epoch_id/revisit_of/pose_at_first_seen/history。
· 高度 C：map_z=REL（起飞点 z=0），簇顶/机体同一局部地图实例；
  arm 时一次性 REL/LCL 静态变换，飞行中不重估；变化→untrusted→不飞越；
  gate 记录 10 项 + anchor_id/transform_id；测量轮不飞越。
· Action：WSL 唯一入口；cancel 幂等、result 只发一次；落地单所有者
  executor 正常 / supervisor 仅 heartbeat 丢失 / bridge 去重 ≥2 s；
  simcheck 域不可达 fail-closed；旧驱动未知高度源硬错误。
· 覆盖：默认 25% 重叠；报告高度/FOV/重叠/位姿误差；正对照 + 热力图。
```

## 二、需要确认的 3 个实现语义

### 1) 模型产不出 JSON，先用手写 SemanticIntent 验证

你们上一封明确说模型是导航策略、产不出 JSON；这一封又说"等你把 SemanticIntent
落进 `--remote-intent` 并跑通一轮（真实飞行）"。

我们的计划：**先实现 SemanticIntent 1.0 的本地适配器/校验器，用手写 JSON
（source=local/operator）跑通一轮真实 `area_cover`**，验证编译→TaskSpec→Action；
**不等模型 JSON**。等你们有 JSON 约束解码 + schema，再把 `source=vlm` 接进来。
请确认这样符合你们的验收口径。

### 2) 旧 `vlm_closed_loop.py`：legacy 硬禁飞越，还是必须改 C

新执行器已经是 REL/LCL + C 方向；旧驱动是 90 KB 的回归脚本，仍有 A/B 高度门。
我们建议：

```text
旧驱动标 legacy：高度门一律判障碍、硬禁飞越；未知高度源硬错误；
不再保留 "A 判定可飞越" 的活路径。
```

这满足"不留 A 判定活路径"，但不是完整 C。若你们要求旧驱动也做 C，
需要单独排期（改门 + 三路对比 + 回归）。请拍板选 **legacy 硬禁飞越** 还是 **改 C**。

### 3) scan 的 task_type 映射

1.0 没有 `scan`。我们的映射是：**search + scan 都编成 `area_cover`**，
scan 只是覆盖任务内的传感器扫视；`inspect` 只给"单目标近距复验"。
请确认，或把 `scan` 明确留到 1.1。若把 scan 当 `inspect`，会把它误当成"单目标"。

### 4) 终态后再 cancel 的 result 语义

ROS 2 Action 终态后再 cancel，客户端通常只拿到终态，不会收到新的 result。
我们能保证：**cancel 只生效一次、result 只发一次**；终态后 cancel 返回终态/拒绝。
请确认这个语义即可，不要要求"终态后再回一条新 result"——那会违反 Action 状态机。

## 三、字段来源（避免又造一套）

```text
frame_seq   ← detector best 的 Header.seq（我们会在 best 发布时填）
frame_size  ← CameraInfo width/height（随 best_meta 一起发）
epoch_id    ← 每次任务运行 id（report_* 的 stamp）
pose_at_first_seen ← 首次 confirmation 时的 PX4 ENU/heading
history     ← 合并/状态升级的 (t, status, world) 列表
```

findings 时间：`first_seen_s/last_seen_s` 用 monotonic 相对秒；
`*_ros_ns + clock_source` 用 ROS time；`merge_window` 用 monotonic。三者都落盘。

## 四、实施顺序（WSL）

1. `SemanticIntent 1.0` 适配器 + 稳定错误码；手写 JSON 跑一轮真实 `area_cover`；
2. Finding 1.0 字段/时间/合并窗口/per-label 半径/discarded；
3. 旧驱动 legacy 硬禁飞越 + 未知高度源硬错误；
4. cancel/land 单所有者 + bridge 去重 + simcheck 域 fail-closed；
5. 覆盖正对照 + 热力图 + 位姿误差。

不等回信，先做 1；有真实数据再同步。
