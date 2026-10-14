# 你指出的三条已全部修好（含 dry_run 期间围栏不生效）；我方暂停（WSL → 服务器，2026-10-14）

## 一、三条修改（代码已改、单测通过，**本轮未飞**——你那边服务已停）

```text
1) enable **原子**重置围栏原点
   在 `_on_enable` 里用"enable 那一刻"的 PX4 位姿直接写入 `_guard_origin`（不再置空等下一帧），
   同一处（锁内）重置 `_guard_started_at`；日志打印
   "围栏原点**原子重置**为当前位姿 (x, y) NED"，便于事后复核。
   → 消除 trial4 那种"首个 post-enable 帧按旧原点判 geofence"的窗口。

2) **无确认不交权**（你第 4.2 条）
   逼近超时且**从未到达停止阈值**（即没有二次复验确认的障碍）→ **不交权**：
   作废该候选点 → 回 SEEK 选下一个候选；候选用尽则收工降落。
   （此前是"超时仍交权"，导致 VLM 对着空处绕行又撞围栏——双重失败。）

3) dry_run 期间**围栏/时长包络不生效**（你第 4.3 条）
   `_guard_command` 开头加 dry_run 短路：未交权时 guard 记为 `dry_run`，
   不按时长/围栏执法（此时由摆位脚本控制飞机），记录不再被"起降点旧原点"污染。
```

## 二、你的两项服务端改动：已知晓，等待恢复

```text
· W1 首次真实告警（+4132 MiB）：会话堆积导致（每趟新 session 是对的，但旧会话要回收）
  → 你已加 SESSION_TTL_S=900 / MAX_SESSIONS=8 的 LRU 回收，下轮生效；
· VLN_EVAL_TRAJ_TOP1=1：你下次启动带上（从源头消除 0.1% 畸形 500）。
  恢复后我们会先确认 /health ready，再按"新 session + preflight fail-closed"开跑。
```

## 三、恢复后的计划（我方已就绪）

```text
① 回归：把 trial4 当回归用例（同一候选点布局），确认"geofence 不再误触发 + 无确认不交权"；
③ 连跑 ≥5 次（scripts/run_detour_trial.sh，自动含 preflight/新 session/打分）；
② 最后做 VLM 与 local_planner 的仲裁（安全/规划层覆盖 VLM、同帧同坐标系）。
```

## 四、暂停说明

按操作者要求我们这边也停：VLM 节点、监督层、ROS 栈、PX4、Agent、SSH 隧道全部停掉并逐项核对
（18080 无监听、`/health` 不可达）；接续清单更新在 `docs/next-session-handoff.md`。
恢复时你说一句，我按清单把环境拉起来，先跑 preflight 再飞。
