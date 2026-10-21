# SemanticIntent 1.0：语义意图 → TaskSpec

## 分层

```text
SemanticIntent 1.0（服务器/未来 JSON 解析器，手写也可）
  → WSL 确定性校验/编译（task_spec.validate + 本地事实）
  → TaskSpec 1.0
  → /mission/execute_task Action
```

模型不直接产出 TaskSpec；WSL 不接受未校验的意图。

## 文件

- `mission/semantic_intent.py`：`SemanticIntent`、`RejectedReason`、`compile()`
- `examples/semantic_intent_area_cover.json`：手写示例（6×6 m、REL 2 m）
- CLI：`ros2 run mission semantic_intent --file ... [--json]`

## 契约字段

| 字段 | 约束 |
|---|---|
| `contract_version` | 固定 `"1.0"`，否则 `UNSUPPORTED_VERSION` |
| `task_type` | `area_cover / go_to / inspect / patrol / return`，否则 `UNKNOWN_TASK_TYPE` |
| `target` | `label`、`world_enu [e,n]`、`frame_id=map` |
| `constraints.altitude_band_m` | `[low, high]`，0.5–30 m，v1.0 用 `low` 作为任务高度 |
| `constraints.altitude_frame` | 固定 `REL`；LCL → `UNSUPPORTED_ALTITUDE_FRAME` |
| `constraints.speed_cap_mps` | 0.1–1.0 |
| `constraints.keepout` | map 2D 多边形列表；目标在内 → `TARGET_IN_KEEPOUT` |
| `constraints.area` | 1.1 正式：`frame_id=map`、`polygon`、`line_spacing_m`/`overlap`；兼容旧 `shape/center/width/height/spacing_m` |
| `confidence` | `source=vlm` 且 <0.5 → `LOW_CONFIDENCE` |

稳定错误码：`SCHEMA_INVALID / UNSUPPORTED_VERSION / UNKNOWN_TASK_TYPE /
UNSUPPORTED_ALTITUDE_FRAME / TARGET_UNREACHABLE / TARGET_IN_KEEPOUT /
ALTITUDE_OUT_OF_BAND / CLEARANCE_TOO_LOW / LOW_CONFIDENCE / FACTS_CONFLICT /
LOCAL_FACTS_STALE / OPERATOR_REQUIRED`，每条 `{code, field, detail}`。

> `--remote-intent` 优先；旧 IntentFrame 仍支持，但必须配自然语言，二者不混用。
>
> `constraints.area` 是 WSL 扩展：服务器 1.0 的 `area_cover` 只给了目标点，
> 没有区域几何；没有它无法生成搜索航迹。等服务器补正式字段后再对齐。

## 编译规则

| task_type | TaskSpec |
|---|---|
| `area_cover` / `patrol` | takeoff → search(area) → land |
| `go_to` | takeoff → goto(target) → land |
| `inspect` | takeoff → goto → hover 3 s → land |
| `return` | takeoff → return_home |

- `area.polygon`：map 系绝对坐标，编译为 `coordinate_frame='map'`，执行器不再加起飞原点；
  旧 `shape/center` 仍是相对起飞点；
- `line_spacing_m`/`overlap`：二者取一；同时给以 spacing 为准并写 notes；都不给默认 overlap 0.25；
- `source=vlm` 的 `area_cover` → `OPERATOR_REQUIRED`（模型不产可靠多边形）；
- `target.label` 映射：person/victim→`person`；marker/color→`marker`；其他→`any`；
- 目标距起飞点 >40 m 或 TaskSpec 校验失败 → `TARGET_UNREACHABLE`；
- 区域超出 40 m 围栏 / keepout 由 TaskSpec/本模块拒绝，不"尽力执行"。

## 真实验证（2026-10-21，--remote-intent）

```text
scripts/fly_task.py --remote-intent examples/semantic_intent_area_cover_rel.json --execute
→ area_cover 4×4 m、REL 2 m、operator
→ search_waypoint 8/8，area 16 m²，coverage 64/64（100%）
→ heatmap logs/coverage/coverage_20261021-191109.png
→ pose_age 0.001 s，eph/epv 0.078/0.077 m
→ report logs/tasks/report_20261021-191101-802.jsonl
```

### 更早（2026-10-20，--semantic-intent 别名）

```text
SemanticIntent（手写 operator，area_cover，6×6 m、REL 2 m）
 → TaskSpec（takeoff/search/land）
 → ExecuteTask Action（goal=TaskSpec JSON）
 → 真机：search_waypoint 8/8，area 36 m²，path 30 m，
        下视足印 4 m/重叠 50%，coverage 144/144（100%），漏扫 0
 → result success=True；report_semantic_action_20261020-192310.jsonl
```

## 用法

```bash
# 服务器/模型入口：--remote-intent 自动识别 SemanticIntent 1.0
python3 scripts/fly_task.py --remote-intent examples/semantic_intent_area_cover.json --json

# 兼容别名；只校验/编译，打印 TaskSpec
python3 scripts/fly_task.py --semantic-intent examples/semantic_intent_area_cover.json --json

# 直接执行（fly_task 起 executor）
FLY_TASK_CONFIRM=1 scripts/fly_task.py \
  --semantic-intent examples/semantic_intent_area_cover.json --execute

# 走 Action：先编译，再让 idle executor + action server 执行
python3 scripts/fly_task.py --semantic-intent examples/semantic_intent_area_cover.json \
  --json > /tmp/task.json
ros2 run mission task_action_client --task-file /tmp/task.json --timeout 200
```
