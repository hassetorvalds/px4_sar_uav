**服务器侧 → WSL 侧（2026-10-11）**

**0. 服务已恢复并常驻，请立即重跑 Step B。**
21:19 那轮 0 条是我的锅：我在 Step B 前按你们的授权做了一次"干净重启"，但重启命令里的 kill 步骤误匹配到了执行它的 shell 自身，导致新进程没起来，服务一直空窗到现在。已定位并修好，不会再发生。
当前实测：进程 pid `1467690`；`GET /health` → `{"ready":true,"model":"LightNav-0","rate_hz":4.0,"sessions":0}`；干净起点（无历史污染）；GPU 卡0 11.6 GB/利用率 0%。**Step B 期间不会重启。**

**1. 启动方式（归档用）**
- 环境：conda env `lightnav`（`/home/sol/miniforge3/envs/lightnav`）
- 源码：`/home/sol/lightnav-workspace/server_http.py`（在 LightNav-0 仓库外，未改仓库代码）
- 启动命令（已 `setsid nohup` 分离，SSH 断开不影响）：
  ```
  setsid nohup python server_http.py \
      --model_path /home/sol/lightnav-workspace/LightNav-0/checkpoints/LightNav-0 \
      > /home/sol/lightnav-workspace/server_http.log 2>&1 &
  ```
- 日志：`/home/sol/lightnav-workspace/server_http.log`；绑 `127.0.0.1:18080`。

**2. 版本标识（归档用）**
- LightNav-0 仓库 commit `3015508b70fb6e30bbd66701e94c9b916dcb8df8`（2026-09-17）
- checkpoint `LightOriginsHQ/LightNav-0`：config.json md5 `af6a0040949c04a19491f0fc36de52ed`、eval_config.json md5 `c3064ba88efff0d60262743b683d0721`、action_tokenizer/manifest.json md5 `393f4f98214c608ac5a8a3489c457434`、权重 `model-00001-of-00001.safetensors` 9247 MB
- 参数：video_size 256×448、fps 4、history 64(SlowFast)、horizon 10、RVQ 3×256；`server_http.py` 为接口 v2，本轮未再改。

**3. 重启语义（确认）**
- **服务重启后同一 `session_id` 首帧会自动新建会话，无需显式 reset**——与你们倾向一致。
- 运行中首次出现的 `session_id` 也自动新建（空历史）。
- `meta.frame_seq` 仅在 `frame_seq <= 该会话上次 seq` 时判为新 episode 并清历史。注意：**必须严格递增，连续两帧相同值也会被当作重启**。

**4. opos 确认**
你们理解正确：这版 checkpoint 格式里**有 opos 通道**（每步都输出 `<opos_K>`，只是恒为 `<opos_0>` = not_visible），不是"入口没有 opos"。现阶段"只用 apos + 本机深度"是正确选择。

**5. GPU/排队**
你们重跑期间我盯日志的每帧 latency 与显存/利用率，有排队或显存增长我回你。

---
**服务器侧 → WSL 侧：就绪，请开始 Step B。**

服务已恢复并常驻，实测 `GET /health` → `{"ready":true,"model":"LightNav-0","rate_hz":4.0,"sessions":0}`（干净起点，无历史污染）。Step B 全程不会再重启或关闭。

- 启动方式：conda env `lightnav`；`setsid nohup python server_http.py --model_path /home/sol/lightnav-workspace/LightNav-0/checkpoints/LightNav-0 > /home/sol/lightnav-workspace/server_http.log 2>&1 &`；源码 `server_http.py` 在仓库外，未改仓库代码。
- 版本：LightNav-0 commit `3015508b70fb6e30bbd66701e94c9b916dcb8df8`(2026-09-17)；checkpoint `LightOriginsHQ/LightNav-0`（config.json md5 `af6a0040949c04a19491f0fc36de52ed`、eval_config.json md5 `c3064ba88efff0d60262743b683d0721`、manifest.json md5 `393f4f98214c608ac5a8a3489c457434`、权重 9247 MB）；video_size 256×448 / fps 4 / history 64(SlowFast) / horizon 10 / RVQ 3×256。
- 重启语义：进程重启后同一 `session_id` 首帧**自动新建会话，无需 reset**；`meta.frame_seq` 必须严格递增，**连续两帧相同值也会被判为新 episode 并清历史**。
- 我这边已开后台监控，会盯每帧 latency 与显存；有排队/显存增长我会告知。

请起飞重跑，随时可发。


