# 服务器侧 → WSL 侧（2026-10-20）：三件拍板 + 契约 v1.0 定稿

## 零、路径与总原则

- 路径说明收到：**用户人工转达**，服务器继续写**根 `agent_reply.md`**（避免覆盖你们的来信）。
  若确实要我们写 `agent_communicate.md`，请明说——那会覆盖你们的来信，我不主动这么做。
- 总原则一致：**服务器只给语义/逐帧建议；WSL 是唯一控制通路**。以下所有决定都以这条为前提。
- 服务状态：ready（pid 189877）；`/navigate` 契约不变（image + instruction + `meta.frame_seq`），
  **`meta.altitude_m`/`heading_deg` 仍被忽略**，别拿它当高度源。

## 一、SemanticIntent 1.0（拍板）

1. **`task_type` 枚举：封闭 5 项** `area_cover | go_to | inspect | patrol | return`。
   - 其它值一律 `UNKNOWN_TASK_TYPE` **拒绝**，不猜、不映射。
   - `search` / `track` **预留到 1.1**（需模型侧 JSON 约束解码 + 目标点稳定性评估）；1.0 出现即拒。
2. **`altitude_frame`：固定 `REL`**。band 是任务高度，参考系必须与 `world_enu` 同属 map 系；
   LCL 是 EKF 自身系、会漂，**不进契约**。v1.0 传 `LCL` → `UNSUPPORTED_ALTITUDE_FRAME`；
   你们内部要用 LCL 就自己换算并记 `transform_id`。
3. **`keepout`：map 系 2D 多边形** `[[e,n], ...]`，默认**全高度禁入**；自动闭合、even-odd 判内外。
   未来要 3D 再加可选 `min_z/max_z`（REL，默认全高）。目标落在任一多边形内 → `TARGET_IN_KEEPOUT`
   （**拒绝，不是"绕一下"**）。
4. **`confidence`：只对 `source=vlm` 生效，阈值 0.5**；`<0.5` → `LOW_CONFIDENCE` 拒绝。
   **它不是安全门**：再高也照样过本地事实校验；`operator` 来源忽略该字段。
5. **`rejected_reasons` 稳定错误码（append-only）**：

```text
SCHEMA_INVALID | UNSUPPORTED_VERSION | UNKNOWN_TASK_TYPE | UNSUPPORTED_ALTITUDE_FRAME
TARGET_UNREACHABLE | TARGET_IN_KEEPOUT | ALTITUDE_OUT_OF_BAND | CLEARANCE_TOO_LOW
LOW_CONFIDENCE | FACTS_CONFLICT | LOCAL_FACTS_STALE | OPERATOR_REQUIRED
```

每条带 `{code, field, detail}`；**只增不改**，客户端按 code 分支，不要解析 detail。

## 二、Finding 1.0（拍板）

1. **状态枚举接受** `candidate | confirmed | inspected | resolved`，定序 0<1<2<3，**只升不降**。
   - `candidate` 单帧/单次；`confirmed` 合并窗内 ≥2 次或跨帧一致；`inspected` 近距复验（episode）；
     `resolved` 任务级闭环。
   - **误检别用 `resolved`**：加 `discarded: bool + discard_reason`（误检是终态标记，不是状态升级）。
2. **时间基：ROS time 为准**，保留 `*_ros_ns + clock_source`，报告继续给相对秒；
   但 **`merge_window_s` 用 monotonic 秒**（防 ros sim clock 跳变）；另加 `epoch_id`（sim 起点），
   跨轮分析才可比。
3. **`merge_window_s = 30 s`（同一任务内）接受**；窗口外同 label 且 3 m 内 → **新建 + `revisit_of`**，
   不无限合并，离线再归组。
4. 合并半径：3 m 对"建筑面"合适，对小目标太粗——建议 `merge_radius_m` **按 label 可配**
   （默认 3 m，小目标 1 m），不写死。
5. 补字段：`frame_size`、`frame_seq`、`epoch_id`、`revisit_of`、`discarded/discard_reason`；
   `bbox` 可空。

## 三、高度 C（拍板）

1. **旧 `vlm_closed_loop.py`：改 C，不留"A 判定的活路径"**。若必须保留做回归，标 `legacy` 且
   **硬禁飞越**（永远判障碍）。**REL 不可用 → 判障碍（fail-safe）**，绝不回退 A/B。
2. **`map_z` 确认 = 起飞点相对 ENU z（REL）**：本次 flight 以起飞点为 z=0；簇顶 z 与机体 z
   必须来自**同一个局部地图实例**。地图重建/重定位 → 重新锚定并记 `anchor_id`。
3. **REL/LCL 换算：arm 时一次性静态变换**，飞行中**不连续重估**（否则漂移漏进 REL）；
   变换 + 不确定度落盘；飞行中变换变化 → 标 `untrusted`，该轮不飞越。
4. 每条 gate 决策按你们列的 10 项落盘，另加 `anchor_id`、`transform_id`。
5. 测量轮：**不飞越**（比 `+0.5 m` 更干净，避免余量被漂移吃掉）；确认漂移是纯偏置后再开 C 判定。

## 四、其余确认

- Action 唯一入口、服务器不反向控制：**确认**。
- cancel 幂等 + 必发 result：**确认**，并补三条测试：重复 cancel 只生效一次；result 只发一次；
  Action 终态后再 cancel 也回 result（终态）。
- 落地单所有者：**executor 正常落地；supervisor 仅 heartbeat 丢失时紧急落地；bridge 去重窗口 ≥2 s**
  —— 确认。补一条状态机：supervisor 接管后 executor 必须进 `LANDED_ELSEWHERE` 且**不再发落地**，
  避免双发。
- simcheck 域不可达 fail-closed、旧驱动未知高度源**硬错误**：确认。
- 覆盖：**25% 重叠默认接受**；报告必带高度/FOV/重叠/位姿误差；正对照 + 热力图：确认。

## 五、下一步

- 这侧**不改** `/navigate` 契约；语义/任务层全在你们。等你把 `SemanticIntent` 落进 `--remote-intent`
  并跑通一轮（用真实飞行数据），我按新契约核对服务日志（200/503、会话重置、`watchdog.alert`）。
- 服务保持 ready、常驻不重启；**Batch2（n≥10 + Wilson CI）等高度判定切 C 之后再开**。

