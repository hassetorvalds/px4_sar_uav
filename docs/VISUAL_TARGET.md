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


## 6. 自然语言目标槽（2026-10-26）

`fly_nl.py` 现在把“飞到前方锥桶附近”的自由文本目标传给 visual_target：

```bash
python3 scripts/fly_nl.py \
  "起飞到 2 米，飞到前方锥桶附近，悬停 2 秒，降落" \
  --visual-backend dino --execute
```

链路：

```text
中文 target slot=锥桶
→ Qwen 翻译 traffic cone
→ DINO bbox + depth
→ standoff 3m
→ target (-31.42,31.25)
→ 真机 final (-31.11,30.96)，误差约 0.4m
→ abort 0，landed
```

未读取 AirSim 物体坐标。报告：`logs/tasks/report_20261026-131328-088.md`。

其他描述可直接替换 target slot（如“红色屋顶”“那辆车”“右侧灰色建筑”）；
中文目标由 Qwen 翻译，英文目标直接给 DINO。


## 7. 橙色球（P1，2026-10-26）

```bash
python3 scripts/fly_nl.py \
  "起飞到 2 米，飞到前方橙色球附近，悬停 2 秒，降落" \
  --visual-backend dino --visual-prompt "orange ball" --execute
```

结果：

```text
DINO conf=0.88, depth=42.28m
target=(27.35,28.20)
final=(27.07,27.92)
到点误差=0.30m，abort=0
```

诊断：球投影处深度约42.16m，与DINO 42.28m一致；场景 `OrangeBall` pose（约47.6m）
与渲染/深度投影不一致，**不能把物体 pose 当 truth**。
“到 DINO 目标”成立；精度只能用感知/深度闭环评估，不用 AirSim 物体坐标验真。


## 8. 多帧稳定目标（2026-10-26）

单帧 bbox/depth 易受遮挡；`visual_target_avg.py` 连续采样多次，目标取中位数，
偏离中位数超过 `--max-spread-m`（默认1.5m）则拒绝。

```bash
python3 scripts/visual_target_avg.py \
  --target-prompt cone --backend dino --samples 3 --standoff-m 3 \
  --out logs/tasks/cone_avg_task.json

# 自然语言入口默认 3 帧；--samples 1 可回退单帧
python3 scripts/fly_nl.py "起飞到 2 米，飞到前方锥桶附近，悬停 2 秒，降落" \
  --visual-backend dino --samples 3 --target-only
```

锥桶实测：

```text
samples = (-31.42,31.24), (-31.43,31.24), (-31.43,31.24)
median  = (-31.43,31.24)
spread  = 0.01 m
```

仍是 image/depth/standoff，不读取 AirSim 物体坐标。

飞行回归（2026-10-26）：

```text
fly_nl --samples 3
target=(-31.43,31.25), spread=0.06m
final=(-31.15,30.95), abort=0
报告 report_20261026-205839-503
```
