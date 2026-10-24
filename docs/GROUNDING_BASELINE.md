# 自然语言导航：LightNav-0 vs Qwen3-VL-8B 离线 grounding 基线

> **状态：离线基线（2026-10-24），未接飞控、未跑 `go_to`。**
> 目的：按 README 现行规划，先回答“VLM 作为 grounding provider 能不能用”，
> 而不是直接换模型/上真机。

## 0. 边界

- 图像：AirSim 前视相机，2 m 高、原点附近，yaw 0/30/45/60/90；
- 目标：场景里橙色球（**代理目标，非人员**）；
- 指令：LightNav 只测英文；Qwen 测英文 + 中文；
- 服务：LightNav-0 `18080`（GPU1）、Qwen3-VL-8B `18081`（GPU0，vLLM 0.19.1，BF16）；
- 只测 grounding/指向，不产 TaskSpec、不做区域覆盖。

## 1. 方法

```bash
# 5 帧 × 3 次 × 2 provider；Qwen 中文另用 manifest_zh
python3 scripts/eval_grounding.py \
  --manifest examples/grounding_manifest.json --repeat 3
python3 scripts/eval_qwen_grounding.py \
  --manifest examples/grounding_manifest.json --repeat 3
python3 scripts/eval_qwen_grounding.py \
  --manifest examples/grounding_manifest_zh.json --repeat 3
```

橙色目标 bbox 由颜色阈值离线计算（仅用于评估，不进入飞行）；
误差为指向点与 bbox 中心的归一化 L2。

Qwen 输出是 **[0,1000] 归一化坐标**（不是像素），脚本先按 `/1000` 转成
[0,1] 再算误差；中文响应可能包 ```json 围栏，脚本先剥围栏。

## 2. 结果

### LightNav-0（EN，5 帧 × 3 次）

| 帧 | 目标中心 | 平均误差 | 延迟 |
|---|---|---|---|
| yaw000 | 0.958, 0.371 | 0.654 | 126–159 ms |
| yaw030 | 0.623, 0.403 | 0.397 | 129–133 ms |
| yaw045 | 0.487, 0.406 | 0.339 | 124–133 ms |
| yaw060 | 0.349, 0.425 | 0.323 | 122–164 ms |
| yaw090 | 0.033, 0.347 | 0.569 | 123–129 ms |

汇总：**平均误差 0.456（0.195–0.794），延迟 122–164 ms（均值 133.4 ms）**。
所有样本都是 `point`，但点常落在地面/非目标处；首帧行为不稳定。

### Qwen3-VL-8B（EN / ZH，各 5 帧 × 3 次）

| 语言 | 可解析 | 平均误差 | 误差范围 | 延迟（稳态） |
|---|---|---|---|---|
| EN | 15/15 | **0.0258** | 0.0099–0.0525 | ~470 ms（首帧 471–705） |
| ZH | 15/15 | **0.0297** | 0.0146–0.0514 | ~467 ms（首帧 454–561） |

Qwen 中文示例：`{"x":672,"y":527,"confidence":0.99}` → 归一化 (0.672,0.527)；
坐标口径为 [0,1000]，已按服务端要求换算。

## 3. 解释

- **Qwen grounding 明显优于 LightNav**：误差约低一个数量级（0.03 vs 0.46），
  EN/ZH 都 15/15 可解析；适合做**一次性目标点/bbox provider**。
- **Qwen 延迟约 0.47 s，LightNav 约 0.13 s**：Qwen 不能跟上 4 Hz（250 ms）实时回路；
  但作为 0.5 m/s 低速任务的 one-shot target provider，可异步/降频使用。
- Qwen 的 `confidence=0.99` 过自信，不能当安全门限；必须用 WSL 的几何/围栏/时效校验。
- LightNav 仍是更快的短时导航 baseline；**不替换默认 provider**，先做 Qwen → `go_to` 验证。
- 本轮没有 Qwen 真机、没有 TaskSpec/行为树、没有区域覆盖。

## 4. one-shot `go_to` 真机验证（2026-10-24）

`scripts/qwen_target.py`：AirSim 图像 + 中文指令 → Qwen [0,1000] 点 → 深度/航向
→ 相对 ENU 目标 → TaskSpec（`go_to`，50 m 围栏）→ `fly_task` 执行。

```text
Qwen：pixel=(1199,270)，depth=32.53 m，proxy bbox 偏差=0.049，latency=673.5 ms
目标：ENU (east 28.43, north 32.53)，距离 43.2 m，在 50 m 围栏内
实际到点：ENU (28.65, 32.39)，水平误差 ≈ 0.27 m
一次 detour_start/done；无 abort；landed，armed=false
报告：logs/tasks/report_20261024-155204-639.jsonl
```

**边界**：这是 **one-shot 目标点 → go_to**，不是视觉伺服，也不是 4 Hz 实时导航；
目标仍是橙色代理物，不是人员。

## 5. 目标 3D 一致性（重要修复）

同一橙色目标、同一位置、yaw 0/30/45/60/90，各跑一次 `qwen_target.py`：

```text
修复前：目标 east 从 +28.4 变到 -30.7（随 yaw 翻转）——不是 Qwen，
        是 simSetVehiclePose 后 getMultirotorState().orientation 滞后，
        而相机图像已按新姿态渲染，导致航向/图像不同步。

修复：qwen_target.py 改用 simGetCameraInfo('0').pose.orientation 的航向。
```

修复后 5 个目标：

| yaw | ENU (east, north) | heading_enu |
|---|---|---|
| 0 | 28.37, 32.69 | 90° |
| 30 | 29.36, 30.83 | 60° |
| 45 | 29.48, 31.42 | 45° |
| 60 | 28.71, 31.95 | 30° |
| 90 | 32.19, 30.71 | 0° |

均值 **(29.62, 31.52)**；σ **(1.35, 0.73) m**；范围 x 3.8 m、y 2.0 m。
说明 Qwen + 深度目标在 yaw 变化下**基本一致**，但仍有约 1–2 m 的 3D 不确定性；
`go_to` 到点误差 0.27 m 是单次结果，不应解释成目标精度。

## 6. 决策与下一步

1. Qwen 列为 **grounding 候选 provider**，LightNav 保留为快速 baseline/fallback；
2. one-shot `go_to` 已通过；下一步可做异步 2 Hz 目标更新，但不要假装 4 Hz；
3. 若 Qwen 目标点稳定，再考虑视觉伺服；若需要 4 Hz，继续用 LightNav 或量化/更小模型；
4. 目标必须过 WSL 校验（frame_seq/时效/围栏/keepout），失败就拒绝，不猜坐标。

## 7. 证据

- 帧：`logs/grounding/frames/yaw000/030/045/060/090.png`（本地，不入库）；
- LightNav：`logs/grounding/lightnav_20261024-151402.json/.csv`；
- Qwen EN：`logs/grounding/qwen_20261024-154832.json/.csv`；
- Qwen ZH：`logs/grounding/qwen_20261024-154843.json/.csv`；
- 服务：LightNav 18080（GPU1）、Qwen 18081（GPU0，BF16，vLLM 0.19.1）；
- `go_to`：`logs/tasks/report_20261024-155204-639.jsonl` 与 `.md`；
- 目标一致性：`logs/grounding/consistency3/yaw*.json`（本地，不入库）。

**结论：Qwen 适合做 grounding 目标 provider；LightNav 继续做快速 baseline/fallback；
下一步是 Qwen 目标适配器 + 1 轮 `go_to`，不是直接替换。**
