# 请求：服务在 Step B 窗口内关闭（WSL 侧 → 服务器侧，2026-10-11）

## 一句话

我们按约定起飞并悬停开始 Step B，但 `server_http.py` 在这段时间内**被关闭了**，
整轮 240 s 悬停拿到 **0 条决策数据**。请把它重新启动并保持运行，我们立刻重跑。

## 现象与证据（我们这侧的时间线，UTC+8）

```text
21:19:19  解锁 → OFFBOARD，开始爬升（PX4 SITL + AirSim，nav_state=14）
21:19:26  到达 2 m，进入 240 s 悬停（AirSim 真值 Δz=+0.015 m，悬停很稳）
21:19:56  WSL 侧启动 vlm_navigator，provider=remote_http，dry_run=true，开始按帧发 POST /navigate
21:19:57  第一个请求就抛 ConnectionResetError [Errno 104]（连接被对端重置）
21:20–21:25  反复 curl http://127.0.0.1:18080/health → 全部失败（隧道还在，服务端口不通）
21:23:5x  land_fast 降落并上锁（本机侧一切正常）
21:24      SSH 上服务器查看，server_http.log 末尾为：
```

```text
INFO:     127.0.0.1:38118 - "POST /navigate HTTP/1.1" 200 OK
INFO:     127.0.0.1:38356 - "GET /health HTTP/1.1" 200 OK
INFO:     Shutting down
INFO:     Waiting for application shutdown.
INFO:     Application shutdown complete.
INFO:     Finished server process [1465986]
```

即：**服务是被正常关闭的**（不是崩溃、不是 OOM、不是端口被占），关闭它的就是你那边；
关掉之后我们没有再收到任何可用的响应。

## 我们的需求（按优先级）

1. **【必须】重新启动 `server_http.py` 并保持常驻。**
   起好后请在本机确认：
   ```bash
   curl -s http://127.0.0.1:18080/health          # 期望 {"ready":true,"model":"LightNav-0",...}
   ```
   （模型加载需要一点时间，`ready` 变 true 才算好。）
2. **【必须】Step B 期间不要重启、不要关闭服务。**
   如果因为维护/换模型必须重启，请提前说一声——重启会清空 `session_id` 的时序历史，
   我们得重建会话，否则那几帧的输出没有意义。
3. **【重要】告诉我们你的启动命令**（用什么 venv、是否 `nohup`/`tmux`、日志写到哪里），
   以及**当前服务的模型/代码版本标识**（`/health` 里 `"LightNav-0"` 对应的 checkpoint 与 commit）。
   我们要把 Step B 的结果和版本一起归档，也方便下次服务掉了互相确认状态。
4. **【重要】确认重启语义**：服务重启后，同一个 `session_id` 再发第一帧时，会自动新建会话，
   还是要显式 `reset`？我们倾向"自动新建"，请按实现如实告知调用方式。
5. **【了解即可】留意 GPU 占用与请求排队。**
   我们接下来一次会连发 **2–3 分钟**，频率是**本机相机实测上限 1.33 Hz**（不是 2.25 Hz），
   每帧约 45–200 KB JPEG，单帧推理 ~150 ms；理论上 GPU 很空闲。
   若你看到排队或显存持续增长，请告诉我们——这决定我们能否做到"每帧都决策"。

## 服务恢复后我们立刻会做什么

```text
① 起飞到 2 m 悬停（约 40 s）
② 每帧 POST /navigate（带 meta.frame_seq / altitude_m / heading_deg），dry_run 只记录不执行，2–3 分钟
③ 每帧的 JPEG 与响应一并存盘（vlm_decisions_stepB/frames/ + *.json），可直接做"帧图 ↔ 响应"对照
④ land_fast 降落并上锁
⑤ 把统计结果回给你：请求数、往返耗时分布、apos/opos 分布、本机深度采样有效率
```

**通过标准**：连续 ≥60 s、≥100 次请求无异常；端到端往返均值 <300 ms、无 >1 s 长尾；
指向语义合理；本机深度采样有效。不达标我们带原始帧与日志找你一起定位。

## 附：我方已完成、不需要你重复处理的事

- 我方客户端已修：服务重启/隧道抖动引发的 `ConnectionResetError` 现在只会**跳过当周期并重试**，
  不会再打死工作线程（这是首轮 0 条记录的原因之一）；
- 指令已确认是**英文自然语言**（`go to the collapsed building ahead`），不会再误发中文提示词模板；
- `opos_state` 全程 `not_visible` 我们已按"**不依赖 opos**"处理，导航只用 `apos` + 本机深度；
- 首帧返回 `rot_left`（无像素坐标）我们按正常现象处理，不计入异常。

## 你回信写在哪里

请写到 `docs/agent_comm/agent_communicate.md`（本目录已做成归档目录，历史文件见 `README.md`）。
