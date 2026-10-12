`waypoints` 已开启，服务重启就绪，监控也重新挂上了。本地自测通过，而且**模型自己验证了 yaw 符号**。

## 自测结果（合成帧，3 帧）

```
seq=0  apos_state=rot_left  wp0=[0.000, 0.000, +0.314]   ← 左转状态 ↔ yaw=+0.314
seq=1  apos_state=rot_left  wp0=[0.002,-0.001, +0.303]
seq=2  apos_state=point     wp0=[0.148, 0.002, +0.021]   ← 向前 0.148 m
```

`rot_left` 状态与 **正的 yaw** 同时出现 → **正 yaw = 左转（CCW）**，符号自洽。返回 10 行路点，`latency 124–131 ms`。

## 符号约定确认（从代码，逐条核对）

| 你们的假设 | 结论 | 依据 |
|---|---|---|
| `forward_m` 机头前方为正、米 | ✅ 正确 | `traj_vocab.py:42` x=forward |
| `lateral_m` **左为正** | ✅ 正确，**无需取反** | `PROTOCOL.md:152`、`AGENTS.md:120`、`tracking.py` "+lateral = LEFT" |
| `yaw_rad` **逆时针/左转为正**、弧度 | ✅ 正确，**是弧度不是角度**（你们的 `|yaw|>4π` 兜底用不上，但无害） | `PROTOCOL.md:152` "+yaw = counter-clockwise" |
| 单位米 | ✅ 正确 | 下述量级提醒 |

**补充两点**：
1. **行是"累积位姿"**（chunk 起点的 ego 系绝对位姿），不是每步增量 → **执行 `waypoints[0]`**，其余是前瞻。
2. **量级提醒**：每步平移上限约 **0.25 m**、偏航约 30°（`DEPLOYMENT.md`）。所以自测里 `wp0.forward≈0.15 m`。你们"`≥0.5 m` 取巡航"的阈值对 `row[0]` 来说**太高了**，几乎不会触发巡航。建议照参考客户端归一化：`vx=clip(forward/0.375)`、`vy=clip(lateral/0.25)`、`vyaw=clip(yaw/(π/20))`，再乘平台上限。路点无时间基准，参考端按每步≈0.1 s 处理。

## 当前状态

- 服务：`ready:true`、`sessions:0`、pid 1485721，`waypoints` 已返回（`[[forward,lateral,yaw], ...]`，10 行），`pointing` 保留。
- 监控：已重启（日志 `stepb_monitor.log`，pid 1486031），窗口 90 分钟，会盯 latency 与显存。
- 已按约定重启一次（清 session），现在是干净起点。

## 可转达 WSL 侧

**服务器侧 → WSL 侧：`waypoints` 已开，服务已重启就绪，实验 B 可开始。**
- 符号确认：`forward` 前正(米)、`lateral` **左正**、`yaw` **CCW/左转正(弧度)**——你们四条假设全对，**lateral 不用取反**。
- `waypoints` 是 10 行**累积位姿**，**用 `waypoints[0]`**。
- 量级提醒：每步平移 ≤~0.25 m、偏航 ≤~30°，别用 0.5 m 当巡航阈值；建议 `vx=clip(f/0.375)`、`vy=clip(l/0.25)`、`vyaw=clip(yaw/(π/20))`。
- 自测旁证：`apos_state=rot_left` 时 `wp0.yaw=+0.314`（正=左），符号自洽。
- 监控已开；跑的时候有排队/显存增长我会喊。

随时可以让她起飞跑实验 B。

---

