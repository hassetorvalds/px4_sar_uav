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

# 基线调用
python3 scripts/vlm_remote_check.py --ros-args \
  -p image_path:=logs/grounding/frames/yaw045.png \
  -p instruction:="go to the orange ball ahead" -p repeat:=3
```

橙色目标 bbox 由颜色阈值离线计算（仅用于评估，不进入飞行），
得到目标中心的归一化坐标作为“视觉中心”。

## 2. 结果

| 帧 | 指令 | 往返 | 状态/动作 | 指向点 (x,y) | 橙色目标中心 | Δy |
|---|---|---|---|---|---|---|
| yaw045 | go to the orange ball ahead | 158/154/203 ms | 首次 `rot_left`/无坐标；后两次 `point` | 0.510, 0.982 | 0.487, 0.406 | 0.576 |
| yaw060 | fly toward the orange ball | 167/155 ms | 首次 `rot_left`/无坐标；后 `point` | 0.385, 0.648 | 0.349, 0.425 | 0.223 |
| yaw030 | go to the collapsed building ahead | 236/157 ms | 两次 `rot_right`/无坐标 | — | 0.623, 0.403 | — |

延迟：**154–236 ms**，与历史 153–188 ms 同量级。  
首帧存在会话预热差异：第 0 次常给 `rot_left/right`，后续才给 `point`。

## 3. 解释（不要过度解读）

- LightNav 是**导航动作/指向模型**，`point` 不保证是目标 bbox 中心；
  y≈0.98 的点在地面附近，不能直接当“橙色目标世界坐标”；
- `rot_right` 对“目标在画面右侧”是合理的导航动作（yaw030 目标 x=0.623），
  但这不是 3D grounding；
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

- 帧：`logs/grounding/frames/yaw030/045/060.png`（本地，不入库）；
- 调用日志：本次 `vlm_remote_check` 输出（154–236 ms）；
- 目标 bbox：橙色阈值离线计算，中心见上表；
- 服务：LightNav-0 `/health ready`，GPU1。

**结论：LightNav 基线可用但指向不是目标级 grounding；先做 provider 对比，再决定是否替换。**
