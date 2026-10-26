# 历史 README 规划（2026-10-24，保留原文）

> 当前规划见 [README 第 0 节](../README.md#0-现行规划2026-10-26)；本文件仅保留旧版规划，不删除。

## 0. 现行规划（2026-10-24）

> 当前阶段：从“能飞、能避障”的原型，进入“自然语言导航 + SAR Demo”的集成阶段。
> 详细未决项/证据等级见 [OPEN_QUESTIONS.md](docs/OPEN_QUESTIONS.md)；C 的 real 级计划见 [REPLAY_PLAN.md](docs/REPLAY_PLAN.md)。

**优先级**

1. **自然语言导航（当前主线）**
   - 任务层：规则解析 / 未来 SemanticIntent → TaskSpec，WSL 做几何与安全校验；
   - 视觉层：VLM 只做 grounding（目标/方位/bbox），不直接产 TaskSpec；
   - 区域覆盖：继续由现有确定性几何规划负责，不交给 VLM；
   - provider 策略：LightNav-0 作为现有 baseline；Qwen3-VL-8B 作为候选挑战者；
     Cosmos-Reason2-8B 官方最低 32GB/Hopper-Blackwell，**不满足当前 24GB 约束**；
     Openfly 只作为路径先验/研究参考，不承担覆盖保证；Molmo 作为 pointing 备选；
   - 已完成 LightNav + Qwen3-VL-8B 同帧 grounding 对比（[GROUNDING_BASELINE](docs/GROUNDING_BASELINE.md)）：Qwen EN/ZH 平均误差 0.026/0.030，稳态约 470 ms；LightNav 0.456、约 133 ms。Qwen 列为 grounding 候选，LightNav 保留 fallback；Qwen one-shot `go_to` 真机已到点（误差约 0.27 m）；5 个 yaw 的目标 3D 一致性 σ≈1.35/0.73 m；3 样本中位数 spread 0.25 m、稳定化 go_to 到点 0.41 m；异步 dynamic go_to 到点 0.70–0.71 m（实测约 0.75Hz，非 2Hz；含 acquire/stale/围栏 abort）；Markdown 报告已含视觉目标来源/状态/失败分类/误差；真实 dynamic 失败路径已验证；入口 `scripts/run_qwen_nav.sh`（one-shot/dynamic）；Hybrid NL `scripts/fly_nl.py`；通用视觉目标 `scripts/visual_target.py`；`fly_nl` 目标槽（任意中文物体 → Qwen 翻译 → DINO bbox/depth/standoff，不读 AirSim 物体坐标）；Qwen 通用检测可由 `fly_nl.py` 从“寻找…”自动启动并发 `/perception/best`；
2. **SAR Demo v0 可靠性**：代理目标 3 轮搜索均 8/8、无 abort，但检测命中 2/3；
   先做离线检测评测/参数调整，不把 2/3 说成可靠；
3. **人员检测（后置）**：需要真实人员素材/模型与误报评估，等导航闭环稳定后再做；
4. **后续增强**：VLM 任务级 JSON（等模型侧约束解码）、行为树、LiDAR/时间同步、
   全局/局部规划升级；不在当前 Demo 中混跑。

**边界**

- 不追求“单一 VLM 直接输出 TaskSpec”；文本任务与视觉 grounding 分层；
- `source=vlm` 的 `area_cover` 按服务器契约拒绝；
- 当前 C 仍只声明 `model + injection`，real 级未验证；detour 仅 `goto` 段；
- `epoch_id` 为会话级，与 `task_id` 解耦；C 的 anchor/transform 仍是 stage1 占位符。

参考项目的使用边界：`aerial-autonomy-stack`（后续行为树）、
`px4-ros2-interface-lib`（后续 MissionExecutor 架构）、
`AhmedHisham1/UAV-Search`（后续人员检测/地理定位）、
PX4 Obstacle Avoidance/VFH+（局部规划升级）；当前不直接引入其重型框架。

---

