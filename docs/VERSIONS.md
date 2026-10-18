# 版本基线（2026-10-17 冻结）

> 目的：任何一次"升版本/换分支/重建"前后，都能回答"到底换成了什么、和谁兼容"。
> 规则：**升级任一组件后必须重跑 `scripts/check_stack.sh` 与一次最小 Offboard 冒烟**，
> 并把结果补到本文件末尾的"变更记录"。

## 1. 仿真与飞控

| 组件 | 版本 / 提交 | 分支或标签 | 说明 |
|---|---|---|---|
| PX4-Autopilot | `d0c6c0df7f31e56704daa1309f392241f6fe0d5a` | `main`（= `v1.18.0-beta1-442-gd0c6c0df7f`，2026-08-27） | SITL 构建，**不切分支、不改提交** |
| `px4_msgs` | `598c7aad7b2386f9406ebd2a2f841619fddc3c78` | `release/1.18`（2026-07-20） | 消息定义基线 |
| Micro XRCE-DDS Agent | `57d086216d01ec43121845d385894a25987f8a2c` | tag `v2.4.2`（2023-11-22） | `px4_ros_uxrce_dds_ws` |
| AirSim | 1.8.1 | — | Windows 侧，UE4.27 |
| Unreal Engine | 4.27 | — | 场景由用户维护 |

**已知版本错位**：PX4 在 `main` 的 2026-08 提交，`px4_msgs` 锁在 `release/1.18` 的 2026-07 提交；
固件自报 `993f542f`。目前通信与控制验证通过，但新增消息/字段前必须核对定义。

## 2. 主机与运行时

| 组件 | 版本 |
|---|---|
| 宿主机 | Windows 11 + WSL2 |
| WSL 发行版 | Ubuntu 22.04.5 LTS |
| ROS 2 | Humble（`/opt/ros/humble`） |
| Python | 3.10（系统解释器；无本地 GPU 直通） |
| 构建 | `colcon`（`--symlink-install`） |

## 3. 远端 VLM 服务（LightNav-0）

| 项 | 值 |
|---|---|
| 仓库 | LightNav-0，commit `3015508b70fb6e30bbd66701e94c9b916dcb8df8`（2026-09-17） |
| checkpoint | `LightOriginsHQ/LightNav-0`，`model-00001-of-00001.safetensors` 9247 MB |
| config md5 | `af6a0040949c04a19491f0fc36de52ed` |
| eval_config md5 | `c3064ba88efff0d60262743b683d0721` |
| action_tokenizer md5 | `393f4f98214c608ac5a8a3489c457434` |
| 服务 | `server_http.py`（接口 v2），`127.0.0.1:18080`，conda env `lightnav` |
| 关键环境变量 | `VLN_EVAL_TRAJ_TOP1=1`、`SESSION_TTL_S=900`、`MAX_SESSIONS=8` |
| 重启语义 | 进程重启后同一 `session_id` 首帧自动新建会话；`frame_seq` 必须严格递增 |

> ⚠️ 2026-10-14 起必须带 `VLN_EVAL_TRAJ_TOP1=1`（把生成约束到动作词表，消除畸形 500）
> 与 `SESSION_TTL_S/MAX_SESSIONS`（会话回收，避免显存越线）。

## 4. 本机入口

```bash
# 统一环境（PX4_SAR_ROOT / PX4_DIR / ROS_WS / DDS_WS、ROS 2、两个工作区、AirSim 对端地址）
source scripts/env.sh

# 版本自检（与上表逐项比对）
git submodule status
git -C PX4-Autopilot describe --tags --always
git -C ros2_px4_ws/src/px4_msgs describe --tags --always
git -C px4_ros_uxrce_dds_ws/src/Micro-XRCE-DDS-Agent describe --tags --always
```

## 5. 变更记录

| 日期 | 组件 | 变更前 | 变更后 | 冒烟结果 |
|---|---|---|---|---|
| 2026-09-14 | 目录 | `/home/sol/*` 分散 | 整合到 `/home/sol/px4_sar_uav` | 重建后端到端通过（见 migration 文档） |
| 2026-09-14 | PX4 | 同提交 | 不变 | 20 m 悬停 / 3 m 高度通过 |
| 2026-10-17 | 冻结 | — | 本表为当前基线 | 单测 102 passed；飞行验证见 [RESULTS.md](RESULTS.md) |

## 6. 待补

- `params/px4_sitl_baseline.params`（路线图第 0 项）：尚未导出。导出命令：PX4 shell 内 `param save`。
- `scripts/check_stack.sh` 的频率阈值化（当前"话题存在即 OK"）；见本文件同日的修复记录。
