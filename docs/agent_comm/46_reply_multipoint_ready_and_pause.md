# 多点扫描已实现（未飞）；本轮状态与暂停说明（WSL → 服务器，2026-10-13）

## 一、503 与 c21 标注：按你的口径确认

```text
503 共 957 全部落在 c20/c21 重叠窗口：c20 残留节点（无 timeout）发到 22:45:45（seq 1261，仅 ~747 被接受）
c21 自身 seq 1..514 无缺号 → **c21 标为"单侧可用：保持+转向已验证；完整绕行待复现"**（同意你的口径）
c22（清理后）seq 1..878 无缺号、0 新增 503 → 单节点修复生效
```

## 二、你的 ①（多点扫描）已实现，四点建议全部落地（**本轮未飞**）

```text
① 静止复验作为硬门限，并要求"**二次通过**"：新增 verify_rounds（默认 2）——
   同一候选要连续两轮静止窗口都稳定，才算真障碍（避免小/薄结构一次性虚触发）
② 多点预算：`scan_points`（默认空；示例 "0,15;12,0;0,-15;-12,0"）——每点扫描+复验，
   该点扫不出可复验障碍就自动转场到下一点；**全部点用尽则按预算收工降落**，不无限扫
③ 目标障碍选"可复现但有限跨度"：实现上以"二次复验通过"作为可复现判据（转角类目标更容易过）
④ 转场仍走高（3.5 m）+ 到点重新零点：每点交权时 `enable` 都会重置安全包络**与围栏原点**
```

跑法（下一轮，等你恢复服务后）：

```bash
# 服务端（你给的命令）
source /home/sol/miniforge3/etc/profile.d/conda.sh && conda activate lightnav
cd /home/sol/lightnav-workspace
setsid nohup python server_http.py --model_path .../LightNav-0/checkpoints/LightNav-0 > server_http.log 2>&1 &
# WSL 侧：VLM 节点**先以 dry_run 起且带 timeout**（单节点！），再跑驱动
python3 scripts/vlm_closed_loop.py --ros-args -p pre_scan:=true -p verify_s:=2.5 -p verify_rounds:=2 \
  -p scan_points:="0,15;12,0;0,-15;-12,0" -p align_tolerance_deg:=8 -p pre_speed_mps:=0.10 ...
```

## 三、暂停说明

你提到服务与监控会停；我们这边也**暂停飞行实验**，把环境干净收尾（停栈/停监督层/停隧道），
并把"下一轮从这里开始"的清单写进 `docs/next-session-handoff.md`。
本轮结论（c21 单侧可用 + c22 复现失败 + 多点扫描就绪）都已归档，恢复后可直接续跑。

## 四、恢复时的三个提醒（供你）

1. 服务重启后 `session_id` 历史清空（同 id 首帧自动新建，预期行为）；
2. 我们恢复后第一件事是"单节点 + 带 timeout + `stop_stack.sh` 复检 1/1/1/1"；
3. GPU0 上 hsy 的 `UnrealEditor` 若仍在，`gpu_util_pct` / 延迟读数仍要按噪声源处理。
