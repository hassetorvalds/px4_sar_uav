# 观测覆盖：真实点云结果（2026-10-18）

## 它测什么

`coverage_monitor` 把 `/airsim_camera/points`（机体 FLU）转到 ENU，只取近地点
（`|z| ≤ 0.6 m`），落到 1 m 地面网格；网格只保留多边形内的格子。
`coverage_ratio = 已观测格 / 多边形内总格`，并记录 `ground_points` 与
`points_in_polygon` 作为诊断。

不是用相机 FOV 估的条带，是**实际点云**。

## 真机结果（矩形 6×4 m，搜索 6 航点）

```text
任务：起飞 2 m → 搜索 E5/N-6、宽 6 m、长 4 m（24 m²）→ 降落
点数：1,062,311
地面点（|z|≤0.6）：63,984
落在 6×4 m 多边形内：0
覆盖：0/24 格（0.0%），漏扫 24 格
报告：logs/tasks/report_20261018-173323-406.jsonl
```

落地点附近单独抓一帧（无人机在 E7.75/N-4.00，朝北）：

```text
地面点 2980 个，z≈0
east 3.68–10.97，north -3.60–3.54，median (7.70, -3.18)
多边形 north -8..-4 → 地面点整体在多边形北侧（前方）
```

## 结论

- 网格/坐标/点数统计都正常；**不是算法 bug**；
- 前视相机在 2 m 高度看到的是**前方数米到十几米**的地面，不是脚下/航线两侧；
- 6×4 m 的小区域，相机足印基本落在区域外，所以覆盖为 0；
- **继续加多边形航迹不会改善这个数**，因为瓶颈是传感器足印，不是航线形状。

## 下一步选项

| 选项 | 说明 |
|---|---|
| **下视相机/云台（推荐）** | 给搜索单独一个朝下的相机，覆盖模型才成立；检测仍用前视 |
| 航点扫描 | 每个航点 yaw ±90°/360° 驻留，用前视相机扫周围；会增加搜索时间 |
| 放大区域 | 区域至少覆盖相机前方足印；但"看过区域"的定义会变松，不推荐用来美化指标 |
| 多边形航迹 | 等覆盖有意义后再做，解决的是区域形状，不是传感器 |

## 复现

```bash
# 栈里默认启动 coverage_monitor（with_coverage=true）
FLY_TASK_CONFIRM=1 scripts/fly_task.py \
  "起飞到 2 米，搜索东侧 5 米、南侧 6 米、宽 6 米、长 4 米的矩形区域，降落" \
  --execute
# 报告里看 search_coverage / done.coverage
ros2 topic echo /coverage/status --once
```

---

## 下视相机修复（2026-10-18）

### 配置

Windows 侧 `C:\Users\steve\Documents\AirSim\settings.json` 的 `Vehicles.Drone1.Cameras`
增加：

```json
"Down": {
  "CaptureSettings": [
    {"ImageType": 0, "Width": 640, "Height": 480},
    {"ImageType": 1, "Width": 640, "Height": 480}
  ],
  "X": 0.0, "Y": 0.0, "Z": 0.1,
  "Pitch": -90.0, "Roll": 0.0, "Yaw": 0.0,
  "FOV_Degrees": 90.0
}
```

备份：`settings.json.bak-20261018-173936`。改完必须在 UE 里 **Stop→Play** 重载。

### 代码

- `airsim_camera` 新增 `camera_pitch_deg`；≤ -45° 时按**下视**做光学系→机体 FLU：
  `[-y_cam, -x_cam, -z_cam]`（前视仍是 `[z_cam, -x_cam, -y_cam]`）；
- 启动第二个节点 `airsim_camera_down`（`camera_name=Down`，`camera_pitch_deg=-90`）；
- `coverage_monitor` 默认改订 `/airsim_camera_down/points`；
- 前视相机/检测器不变。

### 实测（同一矩形 6×4 m、6 航点、2 m）

```text
前视：0/24（0.0%）   地面点 63,984，区域内 0
下视：24/24（100.0%） 地面点 2,745,600，区域内 1,170,872
报告：logs/tasks/report_20261018-174221-192.jsonl
```

### 结论

- 瓶颈确实是相机足印；换下视后覆盖从 0 变 100%；
- 1 m 网格下 6×4 m 区域全覆盖，漏扫 0；
- 下一步才值得做多边形航迹（形状问题），以及覆盖率随高度的敏感性测试。
