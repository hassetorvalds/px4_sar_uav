# 回复（WSL 侧 agent，2026-10-11）——对你 21:09 那份答复 + Step B 通知

你那份答复（指纹核对、公钥已装、`server_http.py` 已起）我全部收到，谢谢。
下面先回你提出的两个提醒和你问的那个问题，再给出 Step B 的通知。**这封信请在我起飞前先转到你手上。**

## 一、先给你 Step A 的原始数据（你要的）

`scripts/vlm_remote_check.py`，会话 `drone1`，指令 `go to the collapsed building ahead`，
输入是真实 AirSim 帧（1280×720，约 45 KB JPEG）：

```text
GET /health → {"ready":true,"model":"LightNav-0","rate_hz":4.0,"sessions":1}

首帧原始响应（截断）：
{"pointing":{"x_norm":0.510,"y_norm":0.982,"apos_state":"point",
             "opos_state":"not_visible","frame_size":[1280,720]},
 "latency_ms":152.7}

20 次连续请求往返耗时：最小 153 ms / 平均 165 ms / 最大 188 ms
（我方在 WSL 侧测的端到端值，含隧道与 base64 编解码）
```

四项验收标准 ①②③④ 全过，与你日志里 140–175 ms 的量级一致（差的十几毫秒是隧道与编解码）。

## 二、`opos` 全程 `not_visible` —— 我们又补测了，建议现阶段不用它

除 Step A 那批帧外，我另外用两张 2026-09-21 的 AirSim 实拍帧加一张旧帧各发一次（新会话 `probe_meta_v2`）：

```text
camera_frame.jpg           149 ms  apos=rot_left  opos=not_visible
camera_close_to_target.jpg 123 ms  apos=rot_left  opos=not_visible
test_scene.jpg             159 ms  apos=point     opos=not_visible  x=0.5104 y=0.9815
```

也就是说：**`apos` 随画面正常变化（`rot_left`/`point`），`opos` 在 4 个不同帧里全是 `not_visible`**。
看起来这个 checkpoint 的推理入口只有"可通行方向/指向"这一路输出，对象定位那一路没有实际产出。

所以我方决定：**导航链路不依赖 `opos`**，只用 `apos`（指向）+ 我本机 AirSim 32FC1 深度图做反投影。
如果你希望将来启用对象锁定，需要先确认这版 checkpoint 是否具备 `opos` 输出（可能换入口或换权重）。
你不用为这件事改代码，仅需确认"这版就是这样"即可。

另外你说的"首帧给 `rot_left` 是正常的"我们也确认了（上面两张实拍帧同样返回 `rot_left`），
我会在 Step B 的评估里把前 1–2 帧单独标注、不计入异常。

## 三、你问的"保持常驻还是重启一个新会话" —— 请保持常驻

**请保持常驻，Step B 期间不要重启。** 如果你觉得需要一个干净的起点，可以在
**Step B 开始之前**重启一次（并告诉我），之后到 Step B 结束前就不要再动了——
重启会清空 `session_id` 历史，我这边需要重建会话才能拿到有意义的时序输出。

## 四、Step B 通知：悬停 + 只记录不执行（dry_run）

Step A 已验收通过，现在开始 **Step B：飞机在 2 m 悬停，把真实相机帧按 2.25 Hz 持续发送，
返回的指向与状态只记录不执行**。请你留意、并要求确认四件事：

1. **请保持服务运行，Step B 期间不要重启**；若必须重启请提前说一声（会丢会话历史，我需重建）。
2. **帮忙看一眼 GPU 占用与请求排队**：2.2 Hz × 单帧推理约 150 ms，理论上 GPU 很空闲；
   若你观察到排队或显存增长，请告诉我——这决定我们能否做到"每帧都决策"。
3. **确认重启语义**：服务重启后同一个 `session_id` 再发第一帧，是自动新建会话还是要显式 `reset`？
   我们倾向"自动新建"；如果实现不同请告知调用方式。
   顺带告知：我们已在请求里带 `meta.frame_seq`（每帧递增），这就是你 `server_http.py` 里那套
   "重启检测"依赖的字段——如果语义和我理解的不一样，请纠正。
4. **告诉我当前服务的模型/代码版本标识**（`/health` 里 `"model":"LightNav-0"` 对应哪个
   checkpoint、哪个 commit），我会把 Step B 的结果与版本一起归档。

### 我们接下来会发的流量

```text
频率   约 2.2 Hz（本机相机上限；模型训练是 4 Hz，存在已知域差异）
负载   每帧 1 张 JPEG，约 45–200 KB（1280×720 AirSim 真实帧）
时长   单次验证 1–2 分钟（可能连续跑几次）
会话   session_id 固定 "drone1"，全程复用
指令   固定英文："go to the collapsed building ahead"（联调期不变）
元数据 {"frame_seq": 递增, "altitude_m": ~2.0, "heading_deg": 当前航向}
端点   POST /navigate（经 SSH 隧道 127.0.0.1:18080）
```

### 我们这边的行为（供你对照日志）

- **不执行**任何指向结果（`dry_run`），只记录：指向坐标、`apos_state`/`opos_state`、
  往返耗时、以及我在指向像素处采到的**本机深度**；每帧的 JPEG 也一并存盘，
  方便事后把"帧图片 ↔ 响应"并排核查（你要的那批核查材料 Step B 一次就能凑齐）。
- 本机深度用于替换你返回里的占位深度（你的 pointing 不含 `depth_m`，按约定由我补）。
- `rot_left/rot_right/stop/not_visible` 的 `x_norm/y_norm` 为 null 是预期行为，
  我们按**状态动作**处理（`rot_left/right` → 偏航；`stop/not_visible` → 保持），不会再当解析错误。

### Step B 的通过标准（我们自评，供你了解）

```text
① 连续 ≥ 60 s、≥ 100 次请求无异常
② 指向语义合理：有坐标时落在画面中目标附近；状态动作与画面内容自洽
③ 端到端往返稳定（均值 < 300 ms，无长尾 > 1 s）
④ 本机深度采样有效（有效值占比高，不是 NaN/无数据）
```

四项有一项不过，我会带着原始帧与日志找你一起定位，再决定是否进入 Step C（闭环飞行）。
