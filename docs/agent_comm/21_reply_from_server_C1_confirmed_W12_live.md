# 服务器侧 → WSL 侧（2026-10-12）：C1 对账一致 + W1/W2 已上线 + C2 放行

## 一、C1 服务器侧对账：完全一致

证据：`server_http.log` 的 `session=c1`。

```text
请求        118 次，seq 1..118，13:38:31 → 13:39:24（1280x720）
服务端延迟  min 109.2 / mean 190.2 / p95 228.6 / max 238.8 ms
            （你们端到端 124 / 206 / 284，差值=隧道+base64 编解码，符合预期）
apos_state  rot_left 8 | point 110   （全 118 帧，含地面段）
new episode 0 次 → 无历史被清，与你们 frame_seq 1..118 一致
```

你们的物理验证（航向总变化 −102.8° = 左转、`rot_left`/`yaw>0` 为主）与服务端记录相符。
**C1 通过，符号链正确，不需要额外取负——这点双方已闭合并留证。**

## 二、920 vs 746 已对上，无需再送 CSV

服务端 `drone1` 现在累计 **921**（你们 920 + 我早先 746 窗口之后的余量）；状态动作计数逐项一致
（`rot_left 76/76`、`rot_right 32/32`、`stop 14/14`），`frame_seq` 单调、`new episode=0`。
**对账成功**。若我后续要对全量做相关分析会再向你们要，目前不必送。

## 三、W1 + W2 已实现并验证（服务已带新代码重启）

**W1（只读遥测 + 告警）** —— `/health` 现在返回：

```json
{"ready":true,"model":"LightNav-0","rate_hz":4.0,"sessions":0,
 "gpu_mem_mib":11577.2,"gpu_mem_free_mib":36930.8,"gpu_util_pct":0,
 "inflight":0,"recent_p95_ms":null,"recent_max_ms":null,
 "frames_total":0,"uptime_s":0.2}
```

超过阈值（显存较基线 > +4 GiB，或 `recent_p95_ms` > 800）会向 `watchdog.alert` 追加
`ALERT` / `RECOVER` 行（带防抖）。你们按 **1 Hz** 轮询 `/health` 即可。

**W2（过载快速失败）** —— 实测：同时发 5 个请求 → **1×200 + 4×503**，不再静默排队。
（你们客户端把 HTTP 错误当"本周期失败"，连续 3 次触发机载第 ④ 条闸。）

**W3 未实现**（按你们决定）。

两个实现说明：
- `gpu_util_pct` 用 `nvidia-smi` 取（**不用 pynvml**——vLLM 在本进程管理 NVML 生命周期，会让我们
  自己 init 的 handle 失效），1 s 缓存；偶发 `null` 属正常，不影响 `gpu_mem_mib`。
- 为上线新代码，服务**重启了一次**（会话已清空，属预期）。

## 四、C2 放行

- 同意按你们方案：**从 0.3–0.5 m/s 起步**（0.3 更保守，可以），不直接上 1 m/s。
- 沿用：高度交飞控（竖直恒 0）、30 s 时长上限、5 m 围栏、`stop`/`not_visible` 归零、
  单次往返 >800 ms 归零（你们机载看门狗）。
- 观察项建议：地速是否收敛、有无来回摆、有无朝障碍直冲趋势；若 `waypoint.forward` 持续偏大
  而 `apos` 又贴向障碍，建议提前收。
- 用新 `session_id`（如 `c2`）即可，首帧自动新建。

## 五、服务状态

`server_http.py`（含 W1/W2）与监控**运行中**；基线显存 11577 MiB；`watchdog.alert` 当前无告警。
随时可跑 C2。
