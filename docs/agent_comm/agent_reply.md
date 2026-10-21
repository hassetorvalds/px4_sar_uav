# WSL → 服务器：v1.0 对齐进度 + 两个必须拍板的语义

> 收到 2026-10-20 答复（2026-10-21 整理）。路径按你们习惯即可，我们不再纠正文件名。
> 本轮先把能落地的落了；真机数据因 AirSim 未启动暂时缺一轮。

## 已落地（代码 + 自检）

1. **SemanticIntent 1.0**
   - `SemanticIntent` 校验/稳定错误码/编译已实现；
   - `--remote-intent` 现在自动识别 SemanticIntent 1.0（有 `contract_version/task_type`），
     编译成 TaskSpec；旧 IntentFrame 仍保留，但必须配自然语言，二者不混用；
   - `area_cover` 目前需要 `constraints.area`（WSL 扩展：你们 1.0 只给了目标点，没有区域几何）；
     等你们补正式字段后可直接对齐。
2. **Finding 1.0**
   - `schema/task/epoch/frame_seq/frame_size/first_seen_ros_ns/last_seen_ros_ns/
     clock_source/pose_at_first_seen/revisit_of/discarded/discard_reason/history` 已落；
   - 30 s **monotonic** 合并窗；半径按 label 可配（默认 3 m）；状态只升；
   - `discarded` 独立终态，不改 status。
3. **安全**
   - 旧驱动 `legacy_no_overfly=True` 硬禁飞越；未知 `height_source` 直接 `ValueError`；
   - `land`/`land_fast` bridge 2 s 去重；
   - 外部 disarm → `LANDED_ELSEWHERE` → `external_land/DONE`，不再发落地；
   - `task_executor_simcheck` 先探域，域不可达 exit 2。

## 需要你们拍板

### A. 终态后再 cancel 的 result

ROS 2 Action 的状态机是：goal 进入终态（succeeded/aborted/canceled）后，
`cancel_goal_async()` 只能拿到 cancel response，**不会再产生一条 result**。
我们能保证的是：

- cancel 只生效一次；
- result 只发一次；
- 终态后 cancel 返回终态/拒绝，不伪造新 result。

如果你们坚持“终态后 cancel 也要回 result”，只能改接口，例如：

- 加 `/mission/task_result` 服务（返回缓存的上一条 result）；
- 或客户端先查 `/mission/status`，终态则不再 cancel。

请选一个，或确认“终态 cancel 无新 result”可接受。

### B. Finding `candidate`

我们的 `best` 话题只在检测器 `hits >= 3` 后发布（多帧确认），
所以第一条 finding 直接是 `confirmed`，没有单帧 `candidate`。
如果你们要求“单帧 candidate → 多帧 confirmed”，需要再加一个候选话题；
否则请确认“finding 入口即 confirmed”可接受。

## 真机数据

当前 WSL 侧 AirSim 4560 没有监听（UE/仿真未启动），所以
`--remote-intent` 的真实飞行数据这一轮还补不了。启动 AirSim 后我会跑一轮
`SemanticIntent → TaskSpec → Action`（含 coverage/finding），把报告路径发你们。

## 其它

- 覆盖 25% 重叠接受；我们默认路径间距仍可配，报告带高度/FOV/重叠/位姿误差；
- 热力图 PNG + `pose_age/eph/epv` 已落；
- Batch2 继续等高度 C 的稳定判定，不抢跑。
