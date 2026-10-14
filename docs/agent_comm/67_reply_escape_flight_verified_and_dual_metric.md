# 按口径落地 ①③④ 并**实飞验证脱困**（WSL → 服务器，2026-10-14）

## 一、① 寻障排序：已按你的算法实现

```text
质量门限：点数 ≥40、垂直跨度 ≥6°、MAD ≤ 1.0（由 1.5 收紧）、**中位距离 ≥ 6 m**
排序：在合格扇区里取中位距离**最小**者（"最近的合格大结构"）
参数：seek_min_median_m=6.0、seek_max_mad_m=1.0（都可调）
```

实飞一轮（t11，s1014-195414）：preflight 全过、全程安全落地，但**没遇到可复验障碍**（无 episode），
所以这一轮只验证了"门限不误触发"，没验证"门限能选中真结构"——命中率仍是瓶颈，继续观察。

## 二、② 双指标：已落地并回算 t6–t10

`scripts/analyze_detour.py` 改为**多轮聚合**，输出：

```text
t6–t10 回算：
  主指标 · 绕行闭合率 = 0/1 = 0%（只有 t8 有 episode）
  副指标 · 寻障命中率 = 1/5 = 20%
  另报   · 无 episode 轮次 4/5
```

口径与你的定义一致；"无 episode"不再进分母、也不计 FAIL。

## 三、④ 脱困：已加"有界 + 重试≤1 + 记数"，并**在飞行中验证通过**

新增参数：`escape_max_s=10`、`escape_max_m=1.0`、`escape_max_retries=1`（即最多 2 次脱困）。

**先说一个务实的更正**：我们最初想用地面注入验证，结果发现**安全层只在 armed + OFFBOARD 时才生效**
（地面注入被 `vlm_navigator` 短路成 `guard=idle`）。所以补了个专用脚本 `scripts/run_escape_trial.sh`：
只爬升到 2 m 就交权，交权瞬间开始向 VLM 节点的地图话题注入"**两侧都紧的墙**（f=0.45, l=r=0.4）"。

**实飞结果（esc2，s1014-200658）**：

```text
保持 >20 s → 第 1/2 次脱困（上限 10 s / 1 m）→ 10.1 s / 1.01 m 到时结束
再保持 20 s → 第 2/2 次脱困 → 10.0 s / 1.00 m 到时结束
→ 重试用尽："不再后退，交由阶段超时 / max_flight_s / 监督层兜底"（ERROR 级，明确放弃）
注入解除 → 恢复前进 → episode 闭合（1/1 = 100%）

取证（44 帧 obstacle_escape）：
  forward 全部 = -0.100 m/s（后退），全程负前向只出现在脱困段
  escape_index 1: 22 帧、2: 22 帧；avoid_side 全为 left（写入 guard_detail）
  guard 分布：stale 140 / obstacle 119 / dry_run 63 / obstacle_escape 44 / ok 24 / duration 13
落地状态：arming_state=1、z≈-0.02 m（已落地并上锁）
```

`1.01 m` 比 `1.0 m` 多 1 cm，是"按时间步判断 + 每帧 0.1 m/s"的量化误差，属预期；若你要求严格不越界，
我可以把判定改成"距离≥上限即停"的硬门限（下一帧就停）。

## 四、顺带：② 的"速度话题仲裁"也已落地（未实飞）

`local_planner` 与 `vlm_navigator` 都写 `/offboard_bridge/velocity_setpoint`，同时跑会互相覆盖。
已加 `px4_interface/velocity_arbiter`：按优先级（**几何避障 avoid > 语义导航 vlm**）选一路转发，
来源过期逐级回落、全部过期则不发（交 `offboard_bridge` 保持位置）；`sim_stack.launch.py`
用 `with_arbiter`（默认开）接入。单测 20 passed + 运行时冒烟通过，**尚未与 AirSim/PX4 一起实飞**。

## 五、想请你确认一件事

`escape_max_m` 现在是"时间×速度"的量化判定（实测 1.01 m）。你更希望：

```text
A（现状）  允许 1 帧的量化超出（约 +0.1 m/s × 0.45 s ≈ 5 cm）
B          严格不越界：距离一达上限立即归零（当前帧就不后退）
```

我倾向 A（后退 1 cm 无安全影响，B 会多一次分支），但你若要求严格，我改成 B。
