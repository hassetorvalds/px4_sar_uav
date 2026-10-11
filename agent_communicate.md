读完 `agent_communicate.md` 了。对方的核心方案是：**SSH 隧道 + 服务器本地 HTTP 服务**（绑 127.0.0.1，不暴露端口），接口 `GET /health`、`POST /navigate`，响应优先选 A（指向点）以保留本机对速度的最终裁量权；实时回路留本机，远端只跑 0.2–0.5Hz 语义回路。方向是对的，但有几处必须修正。

## 一、先回答他要的三样东西

**② LightNav-0 调用入口（他没 clone 成功，这就是缺的那块）**

公开的 Python API 已存在，正好对应他的需求（`src/lightnav/tracking.py:241`）：

```python
from lightnav.tracking import build_tracking_agent
from lightnav.serving.protocol import pointing_payload   # 输出→协议
import numpy as np

agent = build_tracking_agent(model_path="checkpoints/LightNav-0",
                             backend="vllm_local", task_key="vlnce")  # 自动读 RVQ 解码器
agent.reset(instruction="go to the red sofa")
agent.observe(rgb)                       # HWC uint8 RGB，可反复调用累积历史
wp, raw_text, ms = agent.predict_waypoints(agent.instruction, task_type="vlnce_traj")
# wp: (10,3) float32 [forward_m, lateral_m(+=左), yaw_rad(+=逆时针)]
# pointing: pointing_payload(raw_text, width=W, height=H)
```

**一次真实调用的输入输出**（我在冒烟测试里实测到的，非编造）：
- 输入：12 帧 480×270 合成图 + 指令 `"follow the person in the red shirt"`
- `raw_text`: `<apos_1297><opos_0><act_l0_41><act_l1_92><act_l2_77><|im_end|>`
- 解码出的 `pointing`（这就是接口 A 的数据源）：
```json
{"mode":"grid","frame_size":[480,270],
 "apos_px":null,"opos_px":null,
 "apos_state":"rot_left","opos_state":"not_visible"}
```
- 返回 `wp[0]=[~0, ~0, 0.314]`（原地左转），稳态 **latency_ms ≈ 120–180ms**

也有一条**现成的 WebSocket 服务** `lightnav-serve`（协议见 `docs/PROTOCOL.md`），已经把这些逻辑、会话、微批、ViT 缓存都做好了——这点很重要，见第三节。

**③ 服务器环境（已实测）**

| 项 | 值 |
|---|---|
| GPU | 2× RTX 4090 D，48GB/卡，sm_89，CUDA 12.8 |
| Python | 3.11.16 |
| torch / vllm / transformers | 2.10.0+cu128 / 0.19.1 / 5.8.0 |
| 实测推理 | 稳态 ~120–180 ms/步；服务预热 252ms |
| 冒烟测试 | predict-hf / predict-vllm / serve+client 全 PASS |

**① SSH 访问**：本机 sshd **已在跑**（0.0.0.0:22），`~/.ssh` 目前为空。推荐他选 (a)：生成密钥对，把公钥发你，你 `cat >> ~/.ssh/authorized_keys` 即可，他就能建隧道。

⚠️ **但有个地址矛盾必须你确认**：他文档里写服务器是 `10.255.248.205`，而本机实际 IP 是 **`192.168.31.21`**（home 网段），本机没有任何 10.255.x 接口。请确认 10.255.248.205 是校园网/VPN 映射到这台机器，还是另一台机器。**隧道目标地址用错，后面全盘皆输。**

## 二、三个必须先对齐的设计问题

**1. 接口是无状态单帧，但 LightNav 是有时序记忆的策略——这是最大的坑。**
他提议的 `POST /navigate {instruction, image_b64, meta}` 每次只发一张图。但模型是按 4Hz、64 帧历史（SlowFast 甚至整段 episode）训练的。0.2–0.5Hz 单帧输入 = 分布外，轨迹输出基本不可信。

建议：
- 接口加 `session_id`，服务端用字典维护每路无人机的历史 buffer（`agent.reset/observe` 天然支持）；
- 他的判断"优先做 A（指向）"是对的——**指向本质是单帧感知任务**，比轨迹/速度对稀疏帧更鲁棒；
- 若要 B（速度）或轨迹，需让无人机以 ~4Hz 单独喂帧、只在 0.5Hz 要决策。

**2. 指向没有深度。** `pointing` 只给像素 + `frame_size` + `state`，**没有 `depth_m`**。所以接口 A 的 `depth_m` 必须由他本机的深度补，别指望服务器。另外 `x_norm = apos_px[0]/frame_size[0]` 即可。

**3. 接口 A 必须带上 `apos_state`/`opos_state`。** 因为 `rot_left`/`rot_right`/`stop`/`not_visible` **不是坐标**（像素是 null）。漏了这个字段，他的"照直飞"会把"左转""已到达停止""目标不可见"全当成同一个 null。

**另外**：他示例指令是中文"搜索前方倒塌建筑"，而 LightNav 训练数据是英文导航语料。**先统一发英文指令**（如 `"go to the collapsed building ahead"`），中文可靠性未知，别在联调阶段引入这个变量。

## 三、建议的下一步（Step A：离线打通）

不新造轮子。两条路线，我建议 **路线 1 起步**：

- **路线 1（推荐，Step A 用）**：我写一个 ~60 行 FastAPI（`server_http.py`），直接 `import build_tracking_agent`，绑 `127.0.0.1:18080`，实现 `/health` + `/navigate`（带 `session_id`），响应返回 **pointing（含 state）+ waypoints + latency** 三合一。单进程、最快验证、就是他 60 行计划的落地。
- **路线 2（Step B/C 毕业路径）**：把现成的 `lightnav-serve`（WS，已测通）跑在 127.0.0.1:8050，前面套一个薄 HTTP↔WS 桥。复用微批/会话/ViT 缓存/pointing 解码，天然支持多机。代码略多，但更稳。

他文档里说"你在服务器上跑我写的 FastAPI 包装"——但他没仓库、不知道 API。**建议由我在服务器侧写适配层**，他把本机侧 `scripts/vlm_remote_check.py` 和 `remote_http` provider 写好即可，两边按上面的 JSON 对齐。

## 四、需要你拍板的

1. **地址**：`10.255.248.205` 到底是不是这台机器？（最关键）
2. **SSH**：走 (a) 他给公钥你写 `authorized_keys`，还是 (b) 你自己跑部署命令？
3. **服务实现**：Step A 用路线 1（我写 FastAPI 适配层）还是路线 2？
4. 确认接口用 **`session_id` + 有状态历史**，响应 A 带 `state` 字段、`depth_m` 留给他本机补。

你定了这 4 点，我就可以动手写服务器侧包装（不动现有仓库代码，单开一个独立文件）。要不要我先按路线 1 把 `server_http.py` 草拟出来？

