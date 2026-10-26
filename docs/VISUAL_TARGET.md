# 通用 VLM 视觉导航目标（不依赖 AirSim 物体坐标）

> 目标：任意图像物体 → bbox/point + 深度 → ENU → standoff → TaskSpec。
> **不读取 `simGetObjectPose`/`Cone_5` 等仿真物体坐标。**

## 1. 管线

```text
图像 + 文本 prompt
  → DINO /detect（bbox）或 Qwen（bbox/point）
  → bbox 中心 + depth
  → 相机航向/无人机位姿 → 世界 ENU
  → 沿“当前→物体”向量回退 standoff_m
  → TaskSpec goto
```

## 2. 用法

```bash
source scripts/env.sh
python3 scripts/visual_target.py \
  --target-prompt cone --backend dino --standoff-m 3.0 \
  --instruction "fly to the cone" --out logs/tasks/cone_visual_task.json

FLY_TASK_CONFIRM=1 scripts/fly_task.py \
  --task-json logs/tasks/cone_visual_task.json --execute
```

- `--target-prompt`：DINO 用英文（cone / orange ball / red roof）；Qwen 可中文；
- `--standoff-m`：从物体中心沿视线回退，避免把 goto 点放在物体本体；
- `--backend dino|qwen`；
- 目标坐标只来自图像/深度/位姿，**不读 AirSim 物体坐标**。

## 3. Cone 验证（2026-10-26）

```text
DINO prompt=cone, threshold=0.4
bbox score 0.491, depth 47.31 m
object rel ENU (-33.56,33.35)
standoff 3m → target (-31.43,31.24)
真机 final (-31.12,30.95)，误差 0.43 m
abort 0，landed
```

报告：`logs/tasks/report_20261026-130525-390.md`

## 4. 边界

- 直飞物体本体仍会被净空门阻止（cone 直飞 abort），standoff 是必要安全距离；
- DINO 英文 prompt；中文需映射或 Qwen backend；
- 单次目标，不是视觉伺服；
- 不是人员检测。

## 5. 参考

- [Project-TAR](https://github.com/tvpian/Project-TAR)：ROS2 多模态检测/分割/跟踪；
- [VLM-Nav](https://paperity.org/p/371048647/vlm-nav-mapless-uav-navigation-using-monocular-vision-driven-by-vision-language-models)：
  单目 + VLM 的无地图导航；
- 只借鉴“视觉 grounding → 度量航点”的分层，不直接复制代码。
