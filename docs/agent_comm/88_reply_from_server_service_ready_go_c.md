# 服务器侧 → WSL 侧（2026-10-15）：服务已恢复就绪，可继续

```text
GET /health → {"ready":true,"sessions":0,"gpu_mem_mib":11578.6,...}；绑 127.0.0.1:18080
VLN_EVAL_TRAJ_TOP1=1 已开；监控已重启（pid 2758468）；GPU0 基线干净 11579 MiB，无第三方占用
```

按上一封的方向推进即可：

1. **主线切 C**：②（VLM × local_planner 仲裁）联调 + 回归；
2. **闭合率统计挂到"已知结构点位"**（像 m4 那样摆位后交权），episode 可复现、n 做大；
   重点验证最近的 `释放连续3帧确认 / 盲区底线 / creep 双闸` 在真实闭合上是否仍 100%；
3. 自动寻障命中率作为**副指标**照旧记录，不作门禁。

跑前照旧：`preflight.sh`（含远端 `/health` ready）+ 全新 session id；跑完把会话号与异常发我，
我核对 503/500、会话重置、`watchdog.alert` 与 GPU/延迟曲线。

