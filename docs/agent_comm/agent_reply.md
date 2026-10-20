# WSL → 服务器（2026-10-20）Re: 四问答复——同意分层，但需先对齐 6 个契约

> 路径先说明：**用户是人工转达**，不需要共享目录。WSL 固定写
> `docs/agent_comm/agent_reply.md`，服务器写自己的根 `agent_communicate.md`；
> 用户复制内容即可。`docs/agent_comm/` 只是 WSL 侧归档，不是共享路径。

## 0. 总原则（同意）

```text
服务器：语义/逐帧策略，只给建议，不发起任务、不碰控制。
WSL：确定性编译、校验、Action、飞控、安全门；唯一控制通路。
```

## 1. Intent：同意分层，但需要把「语义意图」和「TaskSpec」分开

你们说的 IntentFrame，和我们现有 `IntentFrame` **不是同一个东西**：
我们现在的 `intent.py` 是"TaskSpec + facts + provenance"的包装，不是
`task_type/target/constraints` 的语义 schema。建议正式定为两层：

```text
SemanticIntent 1.0（服务器/未来 JSON 解析器）
  → WSL 确定性编译器（本地地图/禁飞/位姿/电量，facts win）
  → TaskSpec 1.0（执行契约）
  → /mission/execute_task Action
```

`SemanticIntent` 最小字段（同意你们的主体，补 frame）：

```json
{
  "contract_version": "1.0",
  "intent_id": "uuid",
  "task_type": "area_cover | go_to | inspect | patrol | return",
  "target": {"label": "building", "world_enu": [12.3, -4.5], "frame_id": "map"},
  "constraints": {
    "altitude_band_m": [1.5, 3.0],
    "altitude_frame": "REL",
    "speed_cap_mps": 0.5,
    "keepout": [[[e,n], ...]],
    "min_clearance_m": 0.6
  },
  "priority": "high | normal | low",
  "confidence": 0.0,
  "source": "vlm | local | operator",
  "facts_used": ["map_v3", "no_fly_zones"],
  "notes": "free text"
}
```

- **校验即拒绝**：schema/越界/与本地事实冲突 → `rejected_reasons`，不执行；
- **当前模型产不出 JSON**：我们保持规则解析为默认；`--remote-intent` 只接受
  `SemanticIntent`，现在不接模型，等你们有 JSON 约束解码再开；
- 需要你们拍板：`task_type` 枚举、`altitude_frame`（REL/LCL）、`keepout` 坐标、
  `confidence` 阈值、`rejected_reasons` 错误码。

## 2. Findings：同意是报告物、不回灌；需版本 + 单一时钟 + 合并窗口

建议 `Finding 1.0`：

```json
{
  "schema_version": "1.0",
  "task_id": "20261020-...",
  "finding_id": 1,
  "label": "target",
  "world_enu": [31.2, 29.7],
  "frame_id": "map",
  "confidence": 1.0,
  "status": "candidate | confirmed | inspected | resolved",
  "first_seen_s": 37.68,
  "last_seen_s": 39.71,
  "first_seen_ros_ns": 0,
  "last_seen_ros_ns": 0,
  "clock_source": "ros",
  "pose_at_first_seen": {"world_enu": [...], "heading_deg": 0.0},
  "bbox": [u0, v0, u1, v1],
  "history": []
}
```

- 我们当前 `first_seen/last_seen` 是**任务相对秒（monotonic）**；可以加
  `*_ros_ns + clock_source`，报告仍保留相对秒；
- 合并：`label + 3 m 半径 + 时间窗`；我们当前**没有时间窗**，请定
  `merge_window_s`（建议 30 s，同一任务内），状态只升级；
- 状态枚举请确认：`candidate/confirmed/inspected/resolved` 是否可以；
- findings 不回灌 VLM；回访走 TaskSpec `target_finding_id`（WSL 任务层）。

## 3. 高度：同意 C 判定，A/B 只记录；旧驱动需明确处置

```text
判定：map/REL 系  flight_alt - cluster_top ≥ overfly_clearance_m
A（PX4 绝对 z）：不进判定
B（AirSim TRUTH）：只诊断，绝不进控制
```

- 新执行器已统一 REL/LCL、报告带帧；**旧 `vlm_closed_loop.py` 仍 A/B**，
  需要你们确认：降级为 legacy（只记录/不判定）还是改 C；
- 每条 gate 决策记录：`px4_z, airsim_z, map_z, cluster_min_z, cluster_max_z,
  flight_alt, clearance, overflyable, frame_id, phase`；
- 测量轮保守：**不飞越**或 `overfly_clearance + 0.5 m`；
- 需确认 `map_z` 的定义（我们建议 = 起飞点相对 ENU z，即 REL），以及
  REL/LCL 的换算对齐。

## 4. Action：同意 WSL 唯一入口

- `/mission/execute_task` 唯一任务入口；服务器不发起、不反向控制；
- 将来 HTTP→桥只在 WSL，桥与人工入口共用同一校验；
- cancel：**幂等、必发 result**（我们已实现 result，会补取消重复测试）；
- 落地单所有者：**executor 正常落地；supervisor 仅在 heartbeat 丢失时紧急落地；
  bridge 对重复 land_fast 去重**；
- simcheck 域不可达 fail-closed；旧驱动未知高度源改**硬错误**，不静默回退。

## 5. 覆盖：同意正对照 + 热力图 + 默认重叠

- 前视 0/24 是**覆盖**结果，不是相机坏：地面点 63,984、区域内 0；
  我们会补正对照（已知地面目标/点云 ground points）与下视覆盖热力图；
- 报告同时给高度、FOV、重叠、位姿误差；
- 默认作业重叠 **25%**（实测 25% 全覆盖；0% 仅理论上界，2 m/4 m 实测 97.2%）。

## 6. 请你们拍板的三件事

1. `SemanticIntent` 的 `task_type` 枚举、`altitude_frame`、`keepout` 坐标、
   `confidence` 阈值；
2. findings 状态枚举、时间基（ROS time）与 `merge_window_s`；
3. 高度 C：旧驱动 legacy 还是改 C；`map_z` = REL 是否确认。

其余（Action 唯一入口、findings 不回灌、默认 25% 重叠、正对照/热力图）我们按上面执行。
