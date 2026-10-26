# Grounding DINO tiny bbox 基线（18082）

> **状态：离线评测（2026-10-25），未接飞控、未替换 Qwen。**
> 服务：`grounding-dino-tiny`，GPU1，pixel bbox；prompt 英文。

## 1. 接口

```text
POST /detect
{"image_b64":"...","prompt":"orange ball","box_threshold":0.5,
 "text_threshold":0.25,"meta":{"frame_seq":0}}
→ {"frame_size":[1280,720],
   "boxes":[{"x1":..., "y1":..., "x2":..., "y2":..., "score":..., "label":"..."}],
   "latency_ms":...}
```

坐标是**像素**，不做 [0,1000] 换算。

## 2. 5 张 AirSim 帧（prompt=orange ball, threshold 0.5）

| 帧 | DINO IoU | Qwen bbox IoU | DINO 延迟 |
|---|---|---|---|
| yaw000 | 0.585 | 0.621 | 182.7 ms |
| yaw030 | 0.483 | 0.501 | 170.2 ms |
| yaw045 | 0.508 | 0.520 | 173.2 ms |
| yaw060 | 0.545 | 0.565 | 170.4 ms |
| yaw090 | 0.618 | 0.636 | 165.9 ms |

均值：DINO **0.548**，Qwen **0.569**；DINO 快约 3–4 倍（~170 ms vs ~0.6–0.8 s）。

## 3. 开放词

同一 yaw045：

```text
red roof      → 1 框，score 0.662（位置在右侧建筑）
grey building → 2 框，score 0.671 / 0.603
vehicle       → 0 框（场景无车）
orange ball   → 1 框，score 0.890
```

## 4. 结论

- DINO 是**真 bbox + 低延迟**，适合做候选框 provider；
- 对橙色目标 IoU 与 Qwen 同量级（~0.55），没有明显更准；
- 边缘目标仍会把地面/阴影包进框（y2 偏大），所以不能单独用 IoU 决定目标中心；
- 中文 prompt 不可靠，必须英文短语；
- 默认阈值 0.30 有误检，业务建议 ≥0.5，并保留 Qwen/ROI 交叉确认；
- 当前自动链接仍以“Qwen 点 + ROI + 稳定门”为默认，DINO 作为可选 bbox backend。

## 5. 下一步

1. 若接 DINO：`backend=dino`，bbox 中心 + depth/anchor + geofence，Qwen 只做语义/ROI 复核；
2. 做 **edge-aware bbox**：贴边时不用 bbox 中心，或对 bbox 做深度/图像边界裁剪；
3. 用 DINO 的 bbox 宽高/深度跨帧一致性替代单点 anchor；
4. 人员检测仍后置。


## 6. WSL 接入（可选 backend）

```bash
python3 scripts/fly_nl.py "起飞到 2 米，搜索...寻找橙色球，扫描 20 秒，降落" \
  --detector dino --execute
```

- `fly_nl` 把“橙色球”映射成 `orange ball`；
- `qwen_target_pub` 走 DINO `/detect`，bbox 中心 + depth + anchor；
- 边缘 bbox 用历史中位宽高补未截断边（edge-aware）；无历史时仍用 bbox 中心；
- 非截断 bbox 面积跨帧变化 >60% 拒绝；
- 真机（DINO backend）：搜索 8/8、扫描 20s、目标 3 次确认、无 abort；
  findings 28.73/31.61、29.74/30.27（#2 revisit）；报告 `report_20261026-121844-556`。
- 自动默认仍是 Qwen 点+ROI；DINO 需显式 `--detector dino`。


## 7. 尺寸校正验证（离线 + 在线）

离线：用 5 帧中线框的中位宽高（约 134×141 px），对超过中位 30% 的
bbox 收缩未截断边；中心误差 vs 颜色真值：

| 帧 | 原中心误差 | 校正后 |
|---|---|---|
| yaw000 | 33.9 px | **9.1 px** |
| yaw030 | 10.0 | 10.0 |
| yaw045 | 9.6 | 9.6 |
| yaw060 | 12.1 | 12.1 |
| yaw090 | 28.0 | **5.8 px** |

在线：先在 yaw45 建立非截断历史，再 teleport yaw0；DINO bbox 被收缩为
`(1155.8, 217.8, 1280.0, 351.9)`（原 y2≈391），说明边缘/底部过包有改善。
该 teleport 测试中固定 anchor 正确拒绝了跨 yaw 的世界跳变，所以没有发布新目标。
