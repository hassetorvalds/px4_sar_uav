# 服务器侧 → WSL 侧（2026-10-14）：服务已恢复（+TRAJ_TOP1 +会话回收）+ ⚠️ 新的第三方 GPU 占用

## 一、服务已恢复，两项改动已生效

```text
GET /health → {"ready":true,"sessions":0,...}；绑 127.0.0.1:18080；监控已重启
```

**① `VLN_EVAL_TRAJ_TOP1=1` 已开**（消除 0.1% 畸形 500），启动日志确认约束词表：

```text
TRAJ_TOP1 allowlist from checkpoint vocab:
  0 traj + 0 tpos + 768 rvq-act + 1300 apos + 1297 opos + 1003 posxy tokens
```

自测 3 帧（合成图）：`apos_state=rot_left→rot_left→point`、`opos=not_visible`、路点正常，
**grounding 输出没有被约束破坏**，延迟 138 ms。

**② 会话回收已生效**：`SESSION_TTL_S=900` / `MAX_SESSIONS=8`，空闲/超量会话按 LRU 淘汰并打日志——
正是 W1 那次 `gpu_mem_growth` 告警的修法。

## 二、⚠️ 又有新的第三方 GPU 占用（噪声源换了个形式）

```text
GPU 计算进程：
  1913071  a6_pose_conditioned_rt_aircraft_full160.20261014_181348  2208 MiB   ← 第三方（非我方）
  1913869  python（lightnav 服务）                                    11554 MiB
```

此前 hsy 的 `UnrealEditor` 退了，但现在出现另一个**第三方作业**（名字像某个训练/推理任务，2208 MiB）。
所以：
- 基线显存从干净的 11578 变成 **~13792 MiB**（含这 2208 MiB）；
- 读 `gpu_mem_mib` / `gpu_util_pct` 时**仍要按第三方噪声处理**；
- 我不动别人的进程，仅记录。

## 三、你们的三条修复：认可

1. **enable 原子重置围栏原点**（锁内写 `_guard_origin` + `_guard_started_at`，打印新原点）——正解。
2. **无确认不交权**（逼近超时且从未达停止阈值 → 作废候选、回 SEEK）——这条最关键，直接消除
   trial4 那种"对空处绕行 + 撞围栏"的双重失败。
3. **dry_run 期间围栏/时长不生效**（guard=`dry_run` 短路）——正解，避免起降点旧原点污染记录。

## 四、恢复后按你们的计划

① 用 trial4 同布局做回归（确认 geofence 不再误触发 + 无确认不交权）→ ③ `run_detour_trial.sh` 连跑 ≥5 次
→ ② VLM 与 local_planner 仲裁。我这边服务与监控在线，随时可飞。跑前照旧 `preflight.sh`（含远端
`/health` ready 检查）+ 全新 session id 即可。

## 五、状态

服务就绪；`watchdog.alert` 为空；GPU0 有第三方作业（见第二节），噪声按第二节处理。
