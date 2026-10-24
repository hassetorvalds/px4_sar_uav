# 自然语言导航：LightNav-0 离线 grounding 基线

> **状态：基线（2026-10-24），未实施新模型，未飞。**
> 目的：按 README 现行规划，先回答“VLM 作为 grounding provider 能不能用”，
> 而不是直接换模型/上真机。

## 0. 边界

- 图像：AirSim 前视相机，2 m 高、原点附近，yaw 0/30/45/60/90；
- 目标：场景里橙色球（**代理目标，非人员**）；
- 指令：英文（远端 `/navigate` 只接受英文）；
- 服务：LightNav-0，`127.0.0.1:18080`，GPU1；
- 这不是 TaskSpec 生成，也不是区域覆盖；只测 grounding/指向。

## 1. 方法

```bash
# 抓帧（只读 AirSim 相机，不起 PX4）
# logs/grounding/frames/yaw*.png

# 可复现评测（5 帧 × 1 指令 × 3 次）
python3 scripts/eval_grounding.py \
  --manifest examples/grounding_manifest.json --repeat 3
```

橙色目标 bbox 由颜色阈值离线计算（仅用于评估，不进入飞行），
得到目标中心的归一化坐标作为“视觉中心”；误差为指向点与中心的归一化 L2。

## 2. 结果（`lightnav_20261024-151402.json`）

| 帧 | 橙色目标中心 | 平均指向误差 | 延迟 | 动作 |
|---|---|---|---|---|
| yaw000 | 0.958, 0.371 | 0.654 | 126–159 ms | point ×3 |
| yaw030 | 0.623, 0.403 | 0.397 | 129–133 ms | point ×3 |
| yaw045 | 0.487, 0.406 | 0.339 | 124–133 ms | point ×3 |
| yaw060 | 0.349, 0.425 | 0.323 | 122–164 ms | point ×3 |
| yaw090 | 0.033, 0.347 | 0.569 | 123–129 ms | point ×3 |

汇总：**平均误差 0.456（中位 0.457，范围 0.195–0.794）**；
延迟 **122–164 ms（均值 133.4 ms）**。

早期用 `vlm_remote_check.py` 手动跑 yaw045 时首帧给过 `rot_left`，
本次系统评测五次都是 `point`；说明会话/首帧行为不稳定，不能只取第一次。

## 3. 解释（不要过度解读）

- LightNav 是**导航动作/指向模型**，`point` 不保证是目标 bbox 中心；
  y 误差 0.2–0.8，指向点常落在地面/非目标处，不能直接当“橙色目标世界坐标”；
- `rot_right/left` 作为导航动作可能合理，但不是 3D grounding；
- 因此：**LightNav 可用于短时视觉导航 baseline，不能作为唯一目标定位源**；
- 本轮没有 Qwen/Molmo 对比：服务器只起了 LightNav，未部署其他权重；
  不在本机擅自拉模型/起 vLLM。

## 4. 下一步（按 README 规划）

1. 建一个 provider-agnostic 离线 manifest（同帧、同指令、同评估）；
2. 若服务器能起 Qwen3-VL-8B，按相同帧比较指向/bbox 与延迟（24GB 需实测）；
3. Cosmos-Reason2-8B 在 24GB 下先排除（官方最低 32GB/Hopper-Blackwell）；
4. Molmo-7B 只作为 pointing 备选，先确认 serving 方式；
5. 基线稳定后，才选一个 provider 跑 1 轮 `go_to`；不先烧真机。

## 5. 证据

- 帧：`logs/grounding/frames/yaw000/030/045/060/090.png`（本地，不入库）；
- 调用日志：`logs/grounding/lightnav_20261024-151402.json`/`.csv`（122–164 ms）；
- 目标 bbox：橙色阈值离线计算，中心见上表；
- 服务：LightNav-0 `/health ready`，GPU1。

**结论：LightNav 基线可用但指向不是目标级 grounding；先做 provider 对比，再决定是否替换。**
