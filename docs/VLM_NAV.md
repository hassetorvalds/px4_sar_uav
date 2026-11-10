# VLM 视觉搜索导航

本文档冻结当前已验证的 VLM 搜索导航行为、证据边界和未完成项。

## 1. 总体结构

```text
自然语言 / 目标 prompt
→ VLM grounding (DINO / Qwen)
→ TaskSpec
→ 搜索 / 目标发现
→ acquire (standoff)
→ 净空门 / 监督层
→ PX4
```

约束：

- VLM 只做语义 grounding；
- 不使用 AirSim 物体坐标做目标定位；
- 不修改场景布置；
- offboard 桥默认关闭，双授权；
- 净空门是最后安全兜底。

## 2. 已验证的真实飞行

环境：

```text
PX4 SITL + AirSim
默认场景：Cone_5, OrangeBall
```

### 初始不可见搜索 → 发现 → acquire → 落地

| 目标 | 成功 | 到点误差 |
|---|---|---|
| orange | 3/3 | 0.38–0.41 m |
| cone | 3/3 | 0.27–0.43 m |

每次流程：

```text
takeoff
→ search (目标初始不在视野)
→ DINO 检测
→ target_confirmed
→ search_acquire_start
→ 前向/对准接近
→ search_acquire_done
→ hover
→ land
```

安全：

```text
无显著碰撞 (Ground 轻接触已过滤)
```

### 目标丢失/失败返航

```text
acquire 目标超时
→ acquire_target_lost
→ 丢弃旧目标
→ hold + ±90° 步进扫描
→ 连续丢失达到阈值
→ search_failed_return
→ 返航到起点附近
→ land
```

已实测：

- 目标永久丢失 → 安全返航/落地；
- 注入目标短暂消失 → hold/扫描后重捕获成功；
- 真实 VLM 检测器中断 → 二次丢失，安全返航；
- **真实 VLM 重捕获尚未完成**。

### 跳变门

```text
单帧跳变 > 4 m
→ target_jump_rejected

同一新位置连续 3 帧且 ≥1.5 s
→ target_jump_accepted
→ 更新目标
```

已实测：

- 5 m 跳变先拒绝；
- 持续跳变后接受并 acquire 新位置；
- 抑制单帧误检，同时允许真实移动。

## 3. 搜索几何

成功条件不是任意搜索区域都能复现：

- orange：搜索区域前移到约 `(8,8)`，45° 矩形；
- cone：搜索区域约 `(-8,8)`，-45° 矩形；
- 检测器延迟启动，避免起飞阶段提前锁定；
- `reject_clipped_bbox=true`；
- cone 需要 `dino_box_threshold=0.4`、`min_confidence≈0.3`。

这些是当前经验参数，不是通用解析结果。

## 3.5 静态目标 open-loop 模式

目标确认一次后，允许按最后一次确认位置完成接近：

```text
acquire_require_fresh_target=false
```

用途：

- 静态目标；
- 检测器短暂丢失时，不放弃已经确认的静态目标。

实测：

```text
orange 检测器中断后
→ 按最后目标点接近
→ 到点误差约 1.87 m
→ 安全 land
```

边界：

- 这不是重捕获；
- 目标若真实移动，open-loop 会飞向旧位置；
- 动态目标必须保持 `true`；
- 该模式需要显式配置，默认仍为 `true`。

## 4. 已知限制

```text
真实移动物体：尚未测试
真实 VLM 目标丢失后重捕获：未完成
多目标选择：未实现
动态遮挡/复杂场景：未系统测试
local_map 高度/净空：仍有不可靠性
A* 执行/offboard：默认关闭
```

## 5. 测试口径

```text
全仓 pytest: 332 passed / 3 skipped
```

关键安全测试：

- 净空门候选拦截；
- A* 桥双授权；
- Ground 接触不报显著碰撞；
- 目标跳变拒绝/确认；
- 连续丢失失败返航。

## 6. 结论

当前系统已经能对两种默认目标完成：

```text
初始不可见
→ 搜索
→ 真实 VLM 发现
→ acquire
→ 安全落地
```

并且有安全失败路径。  
但“真实移动目标”和“真实 VLM 重捕获”仍未完成，不能宣称全场景通用。
