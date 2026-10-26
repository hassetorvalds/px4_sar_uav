# 文档索引（2026-10-26）

> 本页只做导航和状态标注。历史文档保留原文，不删除；当前结论优先看这里
> 和 [README 第 0 节](../README.md#0-现行规划2026-10-26)。

## 当前状态（权威入口）

| 文档 | 作用 |
|---|---|
| [README 第 0 节](../README.md#0-现行规划2026-10-26) | 当前优先级、已完成能力、边界 |
| [RESULTS.md](RESULTS.md) | 能力矩阵、真机证据、测试数；顶部为当前状态 |
| [DEMO.md](DEMO.md) | 当前推荐 demo/入口 + 历史步骤 |
| [OPEN_QUESTIONS.md](OPEN_QUESTIONS.md) | 未决/已拍板/已关闭 |
| [GROUNDING_BASELINE.md](GROUNDING_BASELINE.md) | LightNav/Qwen grounding 基线 + 后续真机 |
| [BBox_DINO.md](BBox_DINO.md) | Grounding DINO bbox/edge-aware |
| [VISUAL_TARGET.md](VISUAL_TARGET.md) | 通用 VLM 视觉目标 + standoff + NL 目标槽 |
| [SEMANTIC_INTENT.md](SEMANTIC_INTENT.md) | SemanticIntent → TaskSpec 契约 |
| [ACTION.md](ACTION.md) | ExecuteTask Action 接口 |
| [FINDINGS.md](FINDINGS.md) | Finding 1.0 / revisit |
| [COVERAGE.md](COVERAGE.md) | 下视覆盖热力图与指标 |
| [REPLAY_PLAN.md](REPLAY_PLAN.md) | C real 级 Replay 计划（未实现） |

## 当前已验证（摘要）

- **SAR Demo v0**：代理橙色目标 3 轮搜索 8/8、无 abort；检测命中 2/3（不是人员检测）。
- **自然语言**：Hybrid NL（起飞/搜索/扫描/悬停/降落 + 视觉目标槽）。
- **任意物体目标槽**：中文 → Qwen 翻译 → DINO bbox/depth → standoff；真机 cone 到点约 0.4 m。
- **通用搜索检测**：文本 prompt → `/perception/best`；单 finding、revisit 合并、目标轨迹报告。
- **DINO**：5 帧 IoU 0.48–0.62，~170 ms；edge-aware bbox 已验证。
- **安全**：dynamic target acquire/stale/围栏 abort；standoff 避免贴近物体。
- **测试**：294 passed / 2 skipped（mission 158）；px4_offboard flake8 受沙箱 semaphore 限制。

## 历史/背景文档（保留，不作为当前结论）

| 文档 | 说明 |
|---|---|
| [REFERENCES.md](REFERENCES.md) | 参考项目/论文；文中旧测试数（196）是历史快照 |
| [HISTORY_README_PLAN_20261024.md](HISTORY_README_PLAN_20261024.md) | 2026-10-24 README 规划原文保留 |
| [HISTORY_README_CURRENT_20261018.md](HISTORY_README_CURRENT_20261018.md) | 旧 README 4.4/4.5/22 原文保留 |
| [roadmap.md](roadmap.md) | 早期路线图；当前优先级见 README 0 |
| [next-session-handoff.md](next-session-handoff.md) | 早期交接记录 |
| [migration-2026-09-14.md](migration-2026-09-14.md) | 旧迁移记录 |
| [reference-see-point-fly.md](reference-see-point-fly.md) | 早期“看到-指向-飞”参考 |
| [safety_casebook.md](safety_casebook.md) | 安全案例，仍可参考 |
| [agent_comm/](agent_comm/) | 与服务器的往来信件；按日期归档 |

## 更新规则

1. 真机/单测结论先写 RESULTS，再更新 README 0 与 INDEX；
2. 历史结论不删除，标注“历史”或日期；
3. 未验证/失败路径必须保留，并写清边界；
4. 测试数、提交号、报告路径三者同步。
