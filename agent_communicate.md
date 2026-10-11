## 我这边完成的三件事

**① 指纹核对：完全一致** ✅
```
ed25519 SHA256:smXunYpCGohmYLRaWJgMjHe7WX4dtOD1uqHeAG2oQlU   ← 与 WSL keyscan 一致
rsa     SHA256:MHM+r+2Q5YoOgZyLPWO6fFLeXsYxhUwTL2HfyK/3Ojg   ← 一致
```
确认 `10.255.248.205` 与 `192.168.31.21` 是同一台机器。

**② SSH 公钥已装** ✅
`~/.ssh/authorized_keys`（权限 700/600），sshd 用默认策略（pubkey 默认开启）。

**③ `server_http.py` 已写好并跑起来了** ✅
放在 `/home/sol/lightnav-workspace/server_http.py`（仓库外，未动 LightNav-0 任何代码），绑 `127.0.0.1:18080`，实现 `/health` + `/navigate`（含 `session_id` 历史、`frame_seq` 重启检测、pointing 含 `state`/`frame_size`/归一化坐标）。

## 关键发现：WSL 侧已经打通了

服务器 READY 于 20:58:01（加载 30.4s）。日志显示 `drone1` 会话在 **20:59 通过隧道发来了真实 1280×720 帧**，共 3 个 episode、其中一次**连续 20 帧**的压测：

| 指标 | 实测 |
|---|---|
| 请求来源 | 全部 127.0.0.1（隧道转发）|
| 服务端延迟 | **140–175 ms**（20 帧连续，无异常）|
| 第 1–2 帧 | `apos_state=rot_left` |
| 第 3 帧起 | `apos_state=point`（出现指向像素）|
| `opos_state` | **始终 `not_visible`** |
| 会话复用 | 正常，`frames` 平滑累加 1→20 |

看起来他们已经在跑 **Step A 验收的 ③④ 两项**（延迟 + 20 次连续压测）。我随后自己用合成帧复测，RTT 126–152ms，`frame_size`/`state`/归一化字段都正确返回。

## 两个需要提醒 WSL 侧的点

1. **`opos_state` 全程 `not_visible`**：模型在这批 AirSim 帧里**没有把目标物体框出来**（`opos_px=null`）。`apos`（可通行方向）是正常给出的，所以接口 A 仍可用；但如果他们指望用 `opos` 做目标锁定，需要复核——可能是目标不在视野、采样率 2.25Hz 的域差异，或指令/场景不匹配。建议让他们把一批真实帧的 `apos_px`/`opos_state` 截图核一下。
2. **首帧给 `rot_left`** 是正常现象（开机还没建立运动线索时会先转向），不是 bug。

## 当前状态

服务器**仍在运行**（pid 1465986，GPU 占用 ~11.7GB，端口 18080）。停止用 `kill 1465986` 或 `pkill -f server_http.py`；日志在 `server_http.log`。

下一步建议：让 WSL 侧贴出他们 `vlm_remote_check.py` 的原始响应和测得 RTT，确认四项验收标准全过；重点排查 `opos not_visible`。需要我保持服务器常驻、还是重启一个新会话方便他们复测？

---

