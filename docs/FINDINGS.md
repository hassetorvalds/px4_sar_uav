# Findings：发现物存储与跨任务汇总

## 目录

```text
logs/findings/
├── index.jsonl                         # 每任务一行（追加）
└── findings_<task_id>.json             # 单任务完整记录（原子写）
```

`task_id` 默认取报告文件名去掉 `report_` 和 `.jsonl`，例如
`action_fstore_20261018-171431`。

## 单任务记录

```json
{
  "task_id": "action_fstore_20261018-171431",
  "task_summary": "起飞 2.0 m（REL） → 原地扫描 45 s → 降落",
  "report_path": "logs/tasks/report_action_fstore_20261018-171431.jsonl",
  "intent_path": "...",
  "started_at": 1792314872.0,
  "finished_at": 1792314936.66,
  "findings": [
    {
      "finding_id": 1,
      "label": "target",
      "world_enu": [31.22, 29.69, 4.76],
      "confidence": 1.0,
      "priority": "unknown",
      "status": "inspected",
      "source": "perception",
      "first_seen": 50.04,
      "last_seen": 50.11,
      "updates": 1,
      "step_type": "scan",
      "notes": "target centered"
    }
  ]
}
```

## 触发与出口

- 执行器 `_finish_done()` 在写 `done` 之前调用 `persist_findings()`；
- 参数：`findings_dir`（默认 `logs/findings`）、`persist_findings`（默认 true）；
- 报告事件：`findings_saved`（path/task_id/count）；
- 报告 `done`：`findings`、`findings_path`、`findings_task_id`；
- Markdown：`## 发现物` 表 + `发现物记录：<path>`；
- Action result：`findings_json`。

## 汇总 CLI

```bash
ros2 run mission task_findings --limit 20
ros2 run mission task_findings --json
ros2 run mission task_findings --md logs/findings/summary.md
```

输出：

```text
# 发现物汇总
- 任务数：1（有发现 1）
- 发现物总数：1
- 按类别：{'target': 1}
- 按状态：{'inspected': 1}
| 任务 | 时间 | 类别 | 位置 ENU | 状态 | 置信度 |
...
```

## 已验证

- 真机 Action 扫描 45 s：`finding #1 target [31.22, 29.69, 4.76]` → `inspected`；
- 落盘 `logs/findings/findings_action_fstore_20261018-171431.json` + `index.jsonl`；
- `task_findings` 汇总：任务 1、发现物 1、状态 `inspected`；
- Markdown 报告含「发现物」表与记录路径。

## 限制

- 跨任务不做空间去重：同一目标在不同任务会各记一条（当前按 task_id 去重）；
- 没有数据库/索引服务，只有 JSONL + 文件；
- `task_id` 冲突时 index 保留最后一条，文件覆盖（正常情况下每任务只写一次）。

---

## Finding 1.0（2026-10-20）

新增字段：`schema_version/task_id/epoch_id/frame_id/frame_seq/frame_size/entry_status/detector_hits/
first_seen_ros_ns/last_seen_ros_ns/clock_source/pose_at_first_seen/
revisit_of/discarded/discard_reason/history`。

- 时间：`first_seen/last_seen` 为任务相对 monotonic 秒；ROS 时间另存；
- 合并：`label + 半径（按 label 可配）+ 30 s 窗口`；窗口外同位置默认新建并记 `revisit_of`；
  `merge_revisits=true` 时窗口外同位置更新原 finding（history.revisit=true），`fly_nl` 默认启用；
- 状态只升级；`discarded` 是独立终态标记，不改变 status；
- `entry_status=confirmed`、`detector_hits` 来自 detector best_meta（hits≥3 才发 best）；
- `frame_seq/frame_size` 来自 detector 的 `/perception/best_meta` 与
  `/airsim_camera/camera_info`；
- `task_id/epoch_id` 取报告文件名（去 `report_`）。

真机扫描 45 s 验证：finding `target [17.45,34.36,4.65]`，`frame_seq=10`、
`frame_size=[1280,720]`、`first_seen_ros_ns`、`pose_at_first_seen`、`history` 均落盘。
