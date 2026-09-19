# 参考实现：See, Point, Fly（SPF）

本地位置：`~/SeePointFly/see-point-fly`（CoRL 2025，论文 arXiv:2509.22653）

## 许可（重要）

该仓库的 `LICENSE` 为 **Proprietary Software License（保留所有权利）**，不是开源协议。
因此本项目**只借鉴设计思路，不复制其代码或提示词原文**：

- 借鉴：把自然语言任务转成“图像上的一个指向点 + 粗略深度”，再由几何反投影得到运动指令，
  避免让模型直接输出控制量；
- 本项目自行实现：解析、几何、提示词、ROS 2 节点、与 PX4 的接口；几何部分为通用针孔模型。
- `vlm/pointing.py`、`vlm/vlm_client.py`、`vlm/vlm_navigator.py` 均为本项目原创代码。

如果后续要直接复用其代码或提示词，需要先获得其作者授权。

## 我们借鉴了什么

| SPF 的做法 | 本项目的落地 |
|---|---|
| VLM 输出 `[{"point": [y, x], "depth": d, "label": ...}]`，坐标 0-1000、深度 1-10 | 同一接口约定，解析在 `vlm/pointing.py` |
| 深度等级线性映射为 0.2–2.0 m 参与反投影 | 同一映射（`depth_score_to_meters`），上限可配 |
| 归一化坐标 + 深度 → 机体坐标系向量 | 通用针孔反投影（`PointingGeometry.reverse_project`） |
| 由 AirSim Python API 直接执行 `moveByVelocityBodyFrameAsync` | **改为**发布 ROS 2 速度指令，经 `px4_interface` 转成 PX4 Offboard 设定点 |
| 先转向再前进（两段式，依赖阻塞式 API） | **改为**同时给前进速度与偏航角速度（PX4 速度控制支持 yawspeed） |

## 与 SPF 的有意差异

1. **控制路径**：SPF 直控 AirSim；本项目走 `VLM → 机体速度 → ENU → offboard_bridge → PX4`，
   大模型永不直接接触飞控。
2. **竖直视场**：SPF 水平/竖直使用同一 FOV；本项目按图像宽高比推导竖直 FOV。
3. **转向策略**：SPF 两段式；本项目速度与偏航同时给，闭环靠“目标点随图像回到中心”收敛。
4. **安全层**：本项目增加速度/偏航限幅、指令超时、非 OFFBOARD 或 failsafe 时不发布指令。

## 尚未借鉴（后续可评估）

- 他们的 obstacle_mode（额外要求模型报告障碍物）与 keepalive 机制；
- 他们把决策可视化成 `decision_*.jpg` 的做法（本项目目前只记录 JSON）。

## 已知限制

用静态图片 + mock provider 验证时，目标点在图像里永不移动，
因此偏航指令不会收敛，无人机将持续转圈（2026-09-19 的验证即为该情形）。
闭环收敛需要真实相机接入（后续 `sensor_bridge`）。
