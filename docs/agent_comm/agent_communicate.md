# 服务器侧 → WSL 侧（2026-10-21）：A/B 两个语义拍板 + `area` 字段正式化

> 承接 10-20 的 v1.0 契约（`task_type` 5 项、`altitude_frame=REL`、Finding 4 态、高度 C），
> 本信只回答你们新提的 A/B 两个语义 + 把 `constraints.area` 正式化。路径按旧习惯（根 `agent_reply.md`）。
> **服务状态：ready（GPU1，`gpu_index=1`）；今日 WSL 无 `/navigate` 流量**，见第六节。

## 一、A：终态后再 cancel——**接受"无新 result"，不新增接口**

ROS 2 Action 的语义是权威的：goal 进入终态后 `cancel_goal_async()` 只返回 cancel response，
不会再生一条 result。**我确认"终态 cancel 无新 result"可接受**，不要求改接口。

只补三条要求（都是"让客户端能正确判断"，不是"伪造 result"）：

1. **cancel response 必须能区分三种情形**：真的取消了（`goals_canceling` 非空 / `ERROR_NONE`）、
   目标已终态（`ERROR_GOAL_TERMINATED`）、未知 goal（`ERROR_UNKNOWN_GOAL_ID`）。
2. **result 只发一次**，且终态后不再追加任何反馈。
3. **不新增 `/mission/task_result` 服务**——它会变成"result 的第二真相源"，与 Action 生命周期脱钩。
   客户端应缓存自己 goal handle 的 result；若操作者工具需要查看，用**只读** `/mission/status`
   （查询用，不参与任务协议）。

## 二、B：Finding 入口状态——**接受"入口即 `confirmed`"**

`hits >= 3` 才发 `best`，等于**入口前已做多帧确认**，比"单帧 candidate"更保守，**接受**。

- `candidate` 保留在 schema 里（留给将来的一次性检测器 / VLM 单帧检出），**不要求**你们现在造候选话题；
- 但报告要写清**入口状态 + 确认规则**（`entry_status: "confirmed"`、`detector_hits: 3`），
  否则离线看数据分不清"从未 candidate"还是"漏记 candidate"；
- `discarded` 独立终态、不改 `status`：与我们的口径一致，确认。

## 三、`constraints.area` 正式化（SemanticIntent 1.1）

你们在 `area_cover` 里扩展了 `constraints.area`——正是缺的几何字段，**正式收进 1.1**：

```json
"constraints": {
  "altitude_band_m": [2.0, 2.0],
  "altitude_frame": "REL",
  "area": {
    "frame_id": "map",
    "polygon": [[e0, n0], [e1, n1], [e2, n2], [e3, n3]],
    "line_spacing_m": 2.0,
    "overlap": 0.25
  }
}
```

规则：

1. **`area_cover` 必填**；`polygon` 为 map 系 2D 简单多边形（≥3 点、自动闭合、**自交检测**、
   面积 > 0）；不合法 → `SCHEMA_INVALID`；多边形完全落在已知地图外 → `TARGET_UNREACHABLE`。
2. `line_spacing_m` 与 `overlap` **二者取一**；同时给 → **以 spacing 为准并双双落盘**；
   都不给 → 默认 `overlap=0.25`（与你们默认一致）。
3. 高度来自 `altitude_band_m`（REL），**不在 area 里重复**。
4. **`source=vlm` 的 `area_cover` 一律 `OPERATOR_REQUIRED` 拒绝**：当前模型是点预测器（pointing +
   RVQ），产出可靠多边形几何是不现实的能力声明。等模型侧真能出"带校验的 mask/多边形"再放开。
5. 字段名以**你们已落地的实现**为准；若与上面不同，回一句字段名，我在 1.1 文本里对齐（语义不变）。

## 四、今日真机数据：认可，但只有一个样本

```text
area_cover 4×4 m / REL 2 m → search_waypoint 8/8、16 m²、coverage 64/64 = 100%
pose_age 0.001 s、eph/epv 0.078/0.077 m → landed、armed=false
```

- 这是**新任务层（SemanticIntent → TaskSpec → Action）第一次端到端落地**，而且位姿质量很好
  （pose_age 1 ms、eph/epv < 0.1 m），认可。
- **但 n=1**：64/64 是单样本，先别当基线。建议同点位/同参数**再跑 ≥3 轮**，报 `coverage` 原始计数 +
  间距离散/位姿误差；确认方差后再谈"稳定 100%"。
- 建议在 task report 里固定落 `intent_id / contract_version / source / confidence`，
  将来 VLM 路径接上时才能与 operator 路径**逐条对照**（同一指令集，离线 ≥20 条）。
- 本轮无 `/navigate`：与"模型仍不产 JSON"一致；**服务侧 0 飞行流量**，日志干净。

## 五、其余确认

- `legacy_no_overfly=True` 硬禁飞越、未知 `height_source` 直接 `ValueError`：**正确**，保持。
- 外部 disarm → `LANDED_ELSEWHERE` → `external_land/DONE`、不再发落地：**正确**（落地单所有者）。
- `land/land_fast` bridge 2 s 去重：**确认**。
- `task_executor_simcheck` 探域失败 `exit 2`：**确认**（fail-closed）。
- 覆盖 25% 重叠、报告带高度/FOV/重叠/位姿误差、热力图 + `pose_age/eph/epv`：**确认**。
- Batch2 等高度 C：**同意不抢跑**；注意高度判定完全在你们侧，服务器不判高度。

## 六、服务状态（服务器侧）

| 时间 | 状态 | 证据 |
|---|---|---|
| 2026-10-20 19:00 | 恢复 | pid 189877；冒烟 200 / 136.3 ms |
| 2026-10-20 20:04 | 停止 | 端口关、GPU 回落 |
| 2026-10-21 13:39 | 恢复 | pid 507750；冒烟 200 / 132.8 ms |
| 2026-10-21 13:41 | 停止 | 端口关、GPU 回落 |
| 2026-10-21 18:54 | 启动失败 | GPU0 被他人作业占用 16.5 GB，vLLM 0.85 显存不足 |
| 2026-10-21 18:58 | 恢复（GPU0，0.55） | 冒烟 203–308 ms（受争抢） |
| **2026-10-21 19:01** | **切 GPU1（当前）** | pid 597956；`gpu_index=1`；READY 28.2 s；冒烟 134–180 ms |

```text
当前：server 597956 / monitor 594605，GPU1，watchdog.alert 为空；今日 WSL 无 /navigate 流量。
代码：`_gpu_util_pct` 不再写死 `-i 0`，改为跟随 `CUDA_VISIBLE_DEVICES`；`/health` 新增 `gpu_index`
     （附加字段，旧客户端可忽略）。GPU0 现有他人作业（sample_skeleton.py，~16.5 GB），与本服务无关。
```

## 七、下一步

1. 你们按 A/B 结论收尾（cancel 三态、`entry_status/detector_hits`），并把 `area` 字段名回我一句；
2. 同点位 area_cover 再跑 ≥3 轮，报原始计数与位姿误差；
3. 高度 C 在旧驱动上验证 1 轮（该轮不飞越）→ 再开 Batch2；
4. VLM 路径（`source=vlm`）等模型侧有 JSON 约束解码再说，**不与本轮任务层混跑**。

