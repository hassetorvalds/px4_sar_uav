# WSL → 服务器：Qwen3-VL-8B 试部署 + 同帧 grounding 对比（LightNav 保留）

> 本轮只请求**并行试部署**，不要求替换现有 LightNav-0，也不要求改 `/navigate` 契约。
> 服务状态按你们最新 `/health`（GPU1 ready）；我们这边只做离线帧评测，不飞。

## 一、先同步 LightNav 基线（为什么还要试 Qwen）

我们用 5 张 AirSim 前视帧（2 m 高，yaw 0/30/45/60/90，橙色代理目标）
对 LightNav-0 做了 3 次/帧的离线 `/navigate` 评测：

```text
延迟：122–164 ms（均值 133.4 ms）
平均指向误差（归一化 L2）：0.456；范围 0.195–0.794
9 次中 point 输出与橙色目标 bbox 中心偏差较大，常落到地面/非目标处
结论：LightNav 适合做短时视觉导航 baseline，但不是目标级 grounding
```

所以下一步不是“换掉 LightNav”，而是**同帧对比 Qwen3-VL-8B 的 grounding 质量**。

## 二、请求：Qwen3-VL-8B 试部署（独立端口）

请在不影响 LightNav 的前提下试部署 Qwen3-VL-8B（模型 ID/权重以你们实际可用为准）：

1. **端口**：建议 `18081`，OpenAI-compatible `/v1/chat/completions`；
   LightNav 继续在 `18080`，`/navigate` 契约不变；
2. **图像输入**：接受 base64 JPEG + 中文/英文文本指令；
3. **不要**要求它输出 TaskSpec JSON；本轮只验证 bbox/point/相对方位等 grounding；
4. **失败不强行**：若 24GB 总显存无法稳定承载，直接回报失败/量化选项，
   不要让 Qwen 挤掉 LightNav。

### 请回报这些信息

```text
- GPU 型号、总显存、当前空闲（nvidia-smi）
- vLLM 版本；是否原生支持 Qwen3-VL 图像输入
- 实际模型 ID / 权重路径 / license
- dtype（bf16/fp16/awq...）、max_model_len、gpu_memory_utilization
- 启动命令（脱敏即可）
- GET /v1/models 与 /health 的返回
- 单张 AirSim 图 + 中文/英文指令的示例响应
- 单并发 p50/p95 延迟、峰值显存；连续 10 次是否有 503/崩溃
```

> 你们 `/health` 显示 GPU1 free 约 36.9GB，但项目约束写 24GB；请以**实际总显存**为准。
> 若总显存是 24GB，8B BF16 + KV cache 很可能吃紧，请优先尝试量化/限制
> `max_model_len`/并发；仍不稳定就停，保留 LightNav。

参考启动形态（不是硬性命令，按 vLLM 实际版本调整）：

```bash
# 计划示例，未在你们机器上验证
vllm serve <Qwen3-VL-8B 模型ID> --port 18081 \
  --dtype bfloat16 --max-model-len 8192 \
  --gpu-memory-utilization 0.5
```

## 三、对比协议（WSL 侧执行，不飞）

- 帧：同一批 AirSim `logs/grounding/frames/yaw*.png`（5 张，橙色代理目标）；
- 指令：同一组英文 + 中文（例如“go to the orange ball ahead” / “飞到前方橙色目标”）；
- 指标：
  1. 指向点 vs 橙色目标 bbox 中心的归一化 L2；
  2. 输出可解析率（point/bbox/action）；
  3. 单并发 p50/p95 延迟；
  4. 中文成功率；
  5. 显存/稳定性；
- 不要求 Qwen 产 TaskSpec；不要求它替代几何覆盖规划；
- 不跑真机；不接飞控；不与 C/Batch2a 混跑。

## 四、选择规则

```text
Qwen 明显优于 LightNav（误差/中文/可解析）且延迟/显存可接受
  → 作为候选 provider，LightNav 保留为 fallback；
Qwen 只在英文好、中文差
  → 先做中→英适配，不直接切；
Qwen 显存/延迟不可接受
  → 保留 LightNav，Qwen 只做离线研究；
```

**本轮不切换默认 provider**；等对比数据出来再定。

## 五、边界

- 不改 `/navigate`；不要求服务器发起任务或反向控制；
- 不让 VLM 直接产 TaskSpec；`source=vlm` 的 `area_cover` 仍拒绝；
- Cosmos-Reason2-8B 不在 24GB 约束下本轮评估（官方最低 32GB/Hopper-Blackwell）；
- Openfly/Molmo 暂不部署；先把 Qwen 对比做干净；
- 不烧真机、不跑 Batch2a、不引入 Replay。

## 六、需要你们回复的最小集

```text
1. Qwen3-VL-8B 能否在独立端口起来：yes/no
2. 实际 GPU 总显存 / vLLM 版本 / 模型 ID
3. /v1/models + 一张图的示例响应
4. 单并发 p50/p95 + 峰值显存
5. 若起不来：卡在哪（显存/版本/权重/license）
```

我们拿到 endpoint 后，用同一 manifest 做离线对比，再把数据发回。
