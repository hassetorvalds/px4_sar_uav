# 与服务器端 agent 的通信记录（LightNav-0）

本目录存放 WSL 侧（本项目）与**服务器侧 agent**（负责把 LightNav-0 包装成 HTTP 服务）之间的
全部往来文档。通信是**文件式**的：双方把消息写成 markdown 放在本目录，由用户转达。

## 约定

```text
方向    WSL 侧（我） → 服务器侧        文件名  NN_<主题>_to_server.md
方向    服务器侧 → WSL 侧（我）        文件名  NN_<主题>_from_server.md
临时来信 服务器侧随手回复 / 粘贴日志     文件名  agent_communicate.md（我来归档成上面两种）
编号    两位递增序号，按时间顺序；同一个主题的后续讨论续用新编号
```

**请服务器侧把新回复写到 `docs/agent_comm/agent_communicate.md`。**

## 文件清单

| 文件 | 方向 | 时间 | 内容 | 状态 |
|---|---|---|---|---|
| `01_reply_v2_interface_to_server.md` | →服务器 | 2026-10-11 | 地址/SSH 方案、接口定稿 v2、任务分工、Step A 验收标准 | 已按此实施 |
| `02_reply_from_server_stepA.md` | 服务器→ | 2026-10-11 | 指纹核对一致、公钥已装、`server_http.py` 已起；提出 `opos not_visible` 疑点 | 已回复 |
| `03_stepB_notice_to_server.md` | →服务器 | 2026-10-11 | Step B 通知：请保持服务运行、看 GPU/排队、确认重启语义、给版本标识 | 已并入 04 |
| `04_reply_stepB_to_server.md` | →服务器 | 2026-10-11 | 贴出 Step A 原始响应与 RTT、`opos` 三帧探针结果、Step B 流量与通过标准 | 待转达 |
| `05_request_service_down_to_server.md` | →服务器 | 2026-10-11 | **当前问题与需求**：服务在 Step B 窗口内关闭，请重启并保持 | 待转达 |

## 历史路径说明

2026-10-11 归档前，这些文件直接放在仓库根目录，对应关系：

```text
agent_communicate_reply.md       → 01_reply_v2_interface_to_server.md
agent_communicate.md             → 02_reply_from_server_stepA.md
agent_stepB_notice.md            → 03_stepB_notice_to_server.md
agent_communicate_reply_stepB.md → 04_reply_stepB_to_server.md
```

（`CHANGELOG.md` 里出现旧路径的条目是历史记录，不回溯修改。）

## 当前进展速览

```text
Step A（链路验收）      ✅ 通过：20 次往返 153–188 ms，会话复用正常，opos 不依赖
Step B（悬停只记录）    ⏸ 首轮无数据：服务器服务在窗口内被关闭（见 05）
Step C（闭环飞行）      ⏳ 未开始
```

## 关键结论（双方都需知道）

- 相机实际速率 **color 1.33 Hz / depth 1.78 Hz**（不是早先记的 2.25 Hz）；
- LightNav-0 的 `opos_state` 在 4 个真实帧上始终 `not_visible`，**导航链路不依赖它**，
  只用 `apos` + 本机 AirSim 32FC1 深度；
- 远端不返回 `depth_m`，深度一律由本机在指向像素处采样补上；
- 首帧返回 `rot_left` 是正常现象，评估时不计入异常。
