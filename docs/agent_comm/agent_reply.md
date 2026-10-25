# WSL → 服务器：轻量 open-vocab bbox detector 试部署（Qwen/LightNav 保持不动）

> 本轮只请求**并行试部署一个 bbox 检测器**，不替换 Qwen、不改 LightNav `/navigate`，
> 不要求模型产出 TaskSpec，也不跑飞控/真机。

## 一、为什么需要 bbox detector

我们已实测 Qwen3-VL-8B 的 bbox 能力（不是纯点）：

```text
5 张 AirSim 帧，橙色代理目标，Qwen bbox vs 颜色真值 IoU：0.50–0.64
bbox 中心可做目标；bbox 缺失/截断时可退回点
```

但实际使用暴露了限制：

```text
- bbox 贴图像边缘会截断，中心偏移约 1.4 m；
- 自动搜索启动时，边缘目标偶发天空/14 km 假点，被 geofence 拒绝；
- 当前自动链路因此仍用「Qwen 点 + ROI 二次确认 + 3 样本/深度门」；
- bbox 只作为可选手动模式，不敢设为默认。
```

所以我们想要一个**专门的 open-vocab bbox detector**：给图像 + 文本 prompt，
返回真正的 bbox，而不是继续从 Qwen 的点/bbox 里猜。

## 二、请求部署

### 基本要求

```text
- 独立端口：建议 18082（Qwen 18081、LightNav 18080 保持原样）
- 输入：base64 JPEG + 文本 prompt（中文/英文都尽量支持）
- 输出：bbox 列表 + score + label；坐标建议直接给像素，并带 frame_size
- 只做检测：不产 TaskSpec、不接飞控、不发起任务
- 失败/超时：返回明确错误，不要返回假框
```

### 推荐接口（可按你们实现调整，但要稳定）

```http
POST /detect
{
  "image_b64": "...",
  "prompt": "orange ball",
  "meta": {"frame_seq": 0}
}

200 {
  "frame_size": [1280, 720],
  "boxes": [
    {"x1": 556, "y1": 228, "x2": 695, "y2": 356,
     "score": 0.82, "label": "orange ball"}
  ],
  "latency_ms": 85
}
```

坐标请明确是**像素**还是归一化；我们更希望像素 + `frame_size`，避免 Qwen 的
[0,1000] 口径混淆。

### 候选模型（你们选，先核对 license/权重）

```text
优先：Grounding DINO（Swin-T/Tiny）——开放词检测、bbox 稳定
备选：OWLv2 / OWL-ViT（HF transformers）——接口简单、开放词
速度备选：YOLO-World（ultralytics）——快，但需核对 license
不要求 vLLM；transformers/torch HTTP wrapper 即可
```

部署位置建议 **GPU1**（LightNav 当前约 11.5 GiB，GPU1 空闲约 36.9 GiB），
不要挤占 Qwen GPU0 的 40.7 GiB 池；如果显存/依赖不允许，直接回报，不要强上。

### 需要回报的最小集

```text
1. 模型 ID / 权重 / license
2. 部署 GPU、显存占用、启动命令（脱敏）
3. GET /health 或 /v1/models；POST /detect 示例响应
4. 单张 AirSim 图 + "orange ball" 的 bbox 与 latency
5. 单并发 p50/p95；连续 10 次是否 503/崩溃
6. 若失败：卡在显存/依赖/权重/license 哪一步
```

## 三、验收对比（WSL 侧，不飞）

用同一批 5 张 AirSim 帧（`logs/grounding/frames/yaw000/030/045/060/090.png`）：

```text
- bbox vs 橙色颜色真值 IoU（仅评测用）
- 贴边目标是否仍给合理框
- 任意 prompt 的可用性（orange ball / red roof / vehicle；人员后续）
- 单并发 p50/p95
- 与 Qwen bbox（IoU 0.50–0.64）对比
```

目标不是“换掉 Qwen”：

```text
Qwen：语义/点/ROI 二次确认继续保留
open-vocab detector：提供稳定 bbox
WSL：几何/围栏/anchor/finding 继续负责
```

## 四、边界

- 不改 Qwen 18081、不改 LightNav 18080；
- 不要求 detector 产 TaskSpec 或直接控制飞机；
- 不跑 Batch2a、不跑 C、不引入 Replay；
- 不做人员检测，本轮只验证通用 bbox；
- 起不来就保留当前 Qwen 点+ROI，不伪造 bbox。

## 五、请回复

```text
1. 能否在 18082 起 open-vocab bbox detector：yes/no
2. 模型/license/GPU/显存/启动命令
3. /detect 示例响应 + 一张 AirSim 图的 bbox/latency
4. 单并发 p50/p95 + 10 次稳定性
5. 起不来时具体卡点
```

我们拿到 endpoint 后，用同一批帧做 IoU/延迟对比，再把结果发回。
