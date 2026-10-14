#!/usr/bin/env python3
"""Step C 闭环驱动（C1 只偏航 / C2 加前进 共用）：
验证 `waypoint yaw`→偏航、`waypoint forward`→前向速度这条链路的方向、幅值与收敛性。

做四件事：
  1) 起飞到 2 m 并悬停（位置模式）；
  2) 交给 `vlm_navigator`（非 dry_run）用速度设定点驱动：本脚本**不再发目标点**，
     让 offboard_bridge 切到速度模式，避免两个节点互相打架；
  3) 全程记录航向（解卷绕）、高度、**水平位移**、VLM 的 `apos_state`/`waypoint`/我方指令；
  4) 结束时给出**航向变化与状态动作的对应关系**——这是判符号的硬证据：
     PX4 航向是 NED（顺时针为正），所以"左转"= 航向**减小**。

用法（前置：Agent + PX4 + bringup 已启动；vlm_navigator 以
     `dry_run:=false forward_scale:=0.0 lateral_scale:=0.0 max_yaw_rate:=0.5
      run_duration_s:=30 geofence_max_distance_m:=5` 在跑）：

    python3 scripts/vlm_closed_loop.py --ros-args \
        -p start_altitude_m:=2.0 -p run_s:=40.0 \
        -p output_csv:=logs/2026-10-12/c2_closed_loop.csv
"""

import csv
import json
import math
import os
import re
import sys
import time

import rclpy
from flight_check import FlightCheckBase
from geometry_msgs.msg import Twist
import numpy as np
from px4_interface.frames import ned_to_enu, ned_yaw_to_enu_yaw
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vlm.safety import parse_clearance_status

# local_map 的状态串里"前向锥内最近点"的 ENU 相对坐标（微转复验用）
_NEAREST_RE = re.compile(r'nearest_fwd=\(([-0-9.]+),([-0-9.]+),([-0-9.]+)\)@([0-9.]+)m')
_NEAREST_WIDE_RE = re.compile(
    r'nearest_wide=\(([-0-9.]+),([-0-9.]+),([-0-9.]+)\)@([0-9.]+)m')
# 锥内连通簇质心（服务器侧"限定 ②"）：`(dx,dy,dz)@d@size`，优先用于世界系复验
_NEAREST_CENTROID_RE = re.compile(
    r'nearest_centroid=\(([-0-9.]+),([-0-9.]+),([-0-9.]+)\)@([0-9.]+)m@([0-9]+)')

# 日志行缓冲：外部脚本常靠 grep 日志来判断阶段（例如"等进入 RUN 再开始注入"），
# 块缓冲会让判断滞后甚至永远不命中（2026-10-12 踩到两次）。
sys.stdout.reconfigure(line_buffering=True)


class ClosedLoopBase(FlightCheckBase):
    """CLIMB → RUN（交出控制权给 VLM）→ 汇总 → 降落。"""

    def __init__(self) -> None:
        super().__init__('vlm_closed_loop')
        self.declare_parameter('start_altitude_m', 2.0)
        self.declare_parameter('run_s', 40.0)
        self.declare_parameter('decision_topic', '/vlm_navigator/decision')
        self.declare_parameter('output_csv', 'logs/c1_closed_loop.csv')
        self.declare_parameter('land_at_end', True)
        # 全局飞行时限（兜底）：到点无论处于哪个阶段都收工降落
        self.declare_parameter('max_flight_s', 600.0)
        # 摆位阶段：起飞后先朝当前机头方向慢速前进，直到"前向净空"降到阈值（把飞机送到障碍前）
        self.declare_parameter('pre_position', False)
        self.declare_parameter('pre_speed_mps', 0.2)
        # 交权前的前向净空阈值（服务器侧 2026-10-14 选 a）：贴近保持带 0.6 m，
        # 让 VLM 交权后再走 ~0.2 m（约 1.5 s）就能进保持——不再"在 1.0 m 处靠几十秒挤"。
        # 注意：**保持门限 0.6 m 是安全参数，不动**。
        self.declare_parameter('pre_stop_clearance_m', 0.8)
        self.declare_parameter('pre_timeout_s', 30.0)
        # 本地图偶发单帧近读数（实测见过 1.91 m 一闪而过），摆位判定必须要求**连续稳定**
        self.declare_parameter('pre_stable_samples', 3)
        # 扫描变体（服务器侧 2026-10-13 确认）：慢速偏航 360° 找障碍 → 对准 → 逼近 → 交权
        self.declare_parameter('pre_scan', False)
        self.declare_parameter('scan_yaw_rate_deg_s', 15.0)
        self.declare_parameter('scan_found_m', 3.0)
        self.declare_parameter('scan_side_open_m', 1.5)
        self.declare_parameter('scan_max_s', 32.0)
        self.declare_parameter('verify_s', 2.5)
        # 对准（服务器侧建议）：用扫描时确认的方位作为目标航向，转到位再逼近
        self.declare_parameter('align_tolerance_deg', 8.0)
        self.declare_parameter('align_timeout_s', 10.0)
        self.declare_parameter('align_yaw_rate', 0.4)
        self.declare_parameter('realign_max', 3)
        # 多点扫描（服务器侧 2026-10-13 建议 ①）：形如 "0,15;12,0;0,-15;-12,0"（东,北）
        self.declare_parameter('scan_points', '')
        self.declare_parameter('verify_rounds', 2)
        # 可闭合门限（服务器侧 2026-10-13 补充）：对准后、逼近前要求至少一侧 ≥ 该余量，
        # 否则即使障碍可复现也只会"保持不闭合"（大片墙面），应换下一个 scan_point
        self.declare_parameter('closable_side_m', 2.5)
        # 自动寻障（服务器侧 2026-10-14 建议 ①）：本趟飞行先深度扫测定方位，再自动转场
        self.declare_parameter('seek_enabled', False)
        self.declare_parameter('seek_yaw_rate_deg_s', 15.0)
        self.declare_parameter('seek_s', 26.0)
        self.declare_parameter('seek_margin_m', 3.0)
        self.declare_parameter('seek_min_m', 8.0)
        self.declare_parameter('seek_max_m', 30.0)
        self.declare_parameter('depth_topic', '/airsim_camera/depth')
        # 几何去地面（服务器侧 2026-10-14 建议）：把深度像素反投影成机体 3D 点，
        # **只保留机体上方**的点（地面被精确剔除，且随俯仰自动适应），再取最近点的方位/距离。
        self.declare_parameter('seek_hfov_deg', 90.0)
        self.declare_parameter('seek_ground_margin_m', 0.5)
        self.declare_parameter('seek_pixel_step', 4)
        self.declare_parameter('seek_max_range_m', 60.0)
        # 稳健化门限（服务器侧 2026-10-14 补充）：区分"大结构"与"小物件/地面棱"
        self.declare_parameter('seek_min_points', 40)       # 扇区有效点数下限
        self.declare_parameter('seek_min_span_deg', 6.0)    # 垂直角跨度下限（墙/建筑跨多行）
        self.declare_parameter('seek_max_mad_m', 1.0)       # 深度 MAD 上限（大平面离散小）
        # 结构质量门限 + 中位距离下限（服务器侧 2026-10-14 ①）：
        # 先按"点数/垂跨度/MAD"筛出**真结构**，再取中位距离 ≥ 下限里**最近的**，
        # 避免锁到近处杂波（如 3.8 m 小物），同时保证留出绕行空间（围栏 12 m、停止 0.6 m）。
        self.declare_parameter('seek_min_median_m', 6.0)
        self.declare_parameter('seek_max_candidates', 3)    # 最多产出几个候选点（按中位距离排序）
        # 转场（服务器侧 2026-10-13 建议 ①）：高一点飞过去，再降到扫描高度
        self.declare_parameter('transit_enabled', False)
        self.declare_parameter('transit_east_m', 0.0)
        self.declare_parameter('transit_north_m', 0.0)
        self.declare_parameter('transit_altitude_m', 3.5)
        self.declare_parameter('transit_timeout_s', 45.0)
        # 交权瞬间通知 VLM 节点解除 dry_run（节点提前以 dry_run 跑，消除 ~30 s 启动时间差）
        self.declare_parameter('vlm_enable_topic', '/vlm_navigator/enable')
        self.declare_parameter('map_status_topic', '/local_map/status')
        # 交权前的 ±微转复验（服务器侧 2026-10-14 选 B）：
        # 真结构在世界系固定——纯偏航左右各转一点，它的**世界坐标与距离都不变**；
        # 自运动拖影会随姿态改变/消失。比"原地静止再看一次"更本质，且顺带把
        # "转场到位后结构不在前向锥里"的情形扫进锥内。
        self.declare_parameter('rot_verify', True)
        self.declare_parameter('rot_deg', 15.0)          # 左右各转多少度
        self.declare_parameter('rot_rate_deg_s', 12.0)   # 微转速度（慢，避免位姿滞后）
        self.declare_parameter('rot_settle_s', 2.5)      # 每个姿态静置多久再采样（等地图刷新）
        self.declare_parameter('rot_timeout_s', 15.0)    # 单次转向的时间上限
        # 容差 1.5 m（服务器侧 2026-10-14 选 a）：对 20 m 外大面是 7.5% 相对误差，仍很紧；
        # 而"假读数"实测漂 3.49 m，照样被拒。次数保持"单次通过即算"（基元已换成簇质心）。
        self.declare_parameter('rot_tol_m', 1.5)         # 世界系位置一致性容差
        self.declare_parameter('rot_dist_tol_m', 1.5)    # 距离一致性容差
        self.declare_parameter('rot_pose_age_max_s', 0.5)  # 位姿质量门：位置太旧就不判定

        self.altitude = float(self.get_parameter('start_altitude_m').value)
        self.run_s = float(self.get_parameter('run_s').value)
        self.output_csv = str(self.get_parameter('output_csv').value)
        self.land_at_end = bool(self.get_parameter('land_at_end').value)
        self.max_flight_s = float(self.get_parameter('max_flight_s').value)
        self._flight_started = None
        self.pre_position = bool(self.get_parameter('pre_position').value)
        self.pre_speed = float(self.get_parameter('pre_speed_mps').value)
        self.pre_stop = float(self.get_parameter('pre_stop_clearance_m').value)
        self.pre_timeout = float(self.get_parameter('pre_timeout_s').value)
        self.pre_stable = max(int(self.get_parameter('pre_stable_samples').value), 1)
        self._clearance_history = []
        self.pre_scan = bool(self.get_parameter('pre_scan').value)
        self.scan_yaw_rate = math.radians(
            float(self.get_parameter('scan_yaw_rate_deg_s').value))
        self.scan_found = float(self.get_parameter('scan_found_m').value)
        self.scan_side_open = float(self.get_parameter('scan_side_open_m').value)
        self.scan_max_s = float(self.get_parameter('scan_max_s').value)
        self.verify_s = float(self.get_parameter('verify_s').value)
        self.transit_enabled = bool(self.get_parameter('transit_enabled').value)
        self.transit_east = float(self.get_parameter('transit_east_m').value)
        self.transit_north = float(self.get_parameter('transit_north_m').value)
        self.transit_altitude = float(self.get_parameter('transit_altitude_m').value)
        self.transit_timeout = float(self.get_parameter('transit_timeout_s').value)
        self.enable_pub = self.create_publisher(
            String, str(self.get_parameter('vlm_enable_topic').value), 10)
        self._scan_hits = 0
        self._scan_best = None
        self._verify_history = []
        self._align_heading = None       # 目标航向（ENU）
        self._realign_count = 0
        self._lost_counter = 0
        self._points = self._parse_points(str(self.get_parameter('scan_points').value))
        self._point_index = 0
        self._verify_rounds = max(int(self.get_parameter('verify_rounds').value), 1)
        self._verify_passed = 0
        self.rot_verify = bool(self.get_parameter('rot_verify').value)
        self._rot_deg = float(self.get_parameter('rot_deg').value)
        self._rot_rate = math.radians(float(self.get_parameter('rot_rate_deg_s').value))
        self._rot_settle = float(self.get_parameter('rot_settle_s').value)
        self._rot_timeout = float(self.get_parameter('rot_timeout_s').value)
        self._rot_tol = float(self.get_parameter('rot_tol_m').value)
        self._rot_dist_tol = float(self.get_parameter('rot_dist_tol_m').value)
        self._rot_pose_age_max = float(self.get_parameter('rot_pose_age_max_s').value)
        self._rot_step = 0            # 0 静置采 A → 1 转 +deg → 2 静置采 B → 3 转 -deg → 4 静置采 C → 5 判定
        self._rot_anchor_deg = 0.0
        self._rot_samples = []        # [{'heading_deg','world','dist','pose_age_s','fwd_m'}]
        self._rot_step_started = 0.0
        self._position_at = 0.0       # 位置最近一次更新的时刻（位姿质量门/落盘）
        self.nearest_delta = None     # 本机地图"前向锥内最近点"的 ENU 相对坐标
        self.nearest_dist = float('inf')
        self.nearest_at = 0.0
        self.nearest_wide_delta = None   # 宽锥（诊断 A）最近点的 ENU 相对坐标
        self.nearest_wide_dist = float('inf')
        self.nearest_centroid_delta = None   # 锥内连通簇质心（"限定 ②"）
        self.nearest_centroid_dist = float('inf')
        self.nearest_centroid_size = 0
        self._closable_side = float(self.get_parameter('closable_side_m').value)
        self._approach_confirmed = False     # 是否真的逼近到停止阈值（否则不交权）
        self.seek_enabled = bool(self.get_parameter('seek_enabled').value)
        self.seek_yaw_rate = math.radians(
            float(self.get_parameter('seek_yaw_rate_deg_s').value))
        self.seek_s = float(self.get_parameter('seek_s').value)
        self.seek_margin = float(self.get_parameter('seek_margin_m').value)
        self.seek_min = float(self.get_parameter('seek_min_m').value)
        self.seek_max = float(self.get_parameter('seek_max_m').value)
        self._seek_samples = {}
        self._seek_hfov = float(self.get_parameter('seek_hfov_deg').value)
        self._seek_ground_margin = float(
            self.get_parameter('seek_ground_margin_m').value)
        self._seek_step = max(int(self.get_parameter('seek_pixel_step').value), 1)
        self._seek_max_range = float(self.get_parameter('seek_max_range_m').value)
        self._seek_min_points = int(self.get_parameter('seek_min_points').value)
        self._seek_min_span = float(self.get_parameter('seek_min_span_deg').value)
        self._seek_max_mad = float(self.get_parameter('seek_max_mad_m').value)
        self._seek_min_median = float(self.get_parameter('seek_min_median_m').value)
        self._seek_max_candidates = max(
            int(self.get_parameter('seek_max_candidates').value), 1)
        self.clearance = None

        self.rows = []
        self.latest_decision = None
        self._last_seq = None
        self._heading_prev = None
        self._heading_unwrapped_deg = 0.0
        self.heading_rad = float('nan')
        self.px4_x = float('nan')
        self.px4_y = float('nan')
        self._origin_xy = None
        self.create_subscription(
            String, str(self.get_parameter('decision_topic').value),
            self._on_decision, 10)
        self.create_subscription(
            String, str(self.get_parameter('map_status_topic').value),
            self._on_map_status, 10)
        self.create_subscription(
            Image, str(self.get_parameter('depth_topic').value), self._on_depth_image, 2)
        # 心跳：独立监督进程（flight_supervisor.py）据此判断"驱动是否还活着"
        self.declare_parameter('heartbeat_topic', '/flight_heartbeat')
        self.heartbeat_pub = self.create_publisher(
            String, str(self.get_parameter('heartbeat_topic').value), 10)
        self.create_timer(0.5, self._beat)

    # ------------------------------------------------------------- 回调

    def _on_decision(self, msg: String) -> None:
        try:
            self.latest_decision = json.loads(msg.data)
        except ValueError:
            self.latest_decision = None

    def _on_map_status(self, msg: String) -> None:
        parsed = parse_clearance_status(msg.data)
        if parsed is not None:
            self.clearance = parsed
        # 前向锥内最近点的**ENU 相对坐标**（local_map 的 nearest_fwd=(dx,dy,dz)@d）：
        # 微转复验靠它算"世界系里的障碍点"是否稳定（服务器侧 2026-10-14 B）。
        match = _NEAREST_RE.search(msg.data or '')
        if match:
            self.nearest_delta = (float(match.group(1)), float(match.group(2)),
                                  float(match.group(3)))
            self.nearest_dist = float(match.group(4))
            self.nearest_at = time.monotonic()
        elif 'nearest_fwd=inf' in (msg.data or ''):
            self.nearest_delta = None
            self.nearest_dist = float('inf')
            self.nearest_at = time.monotonic()
        # 宽锥最近点（诊断 A，服务器侧 2026-10-14）：区分"真实但偏轴"与"本就是假的"
        wide = _NEAREST_WIDE_RE.search(msg.data or '')
        if wide:
            self.nearest_wide_delta = (float(wide.group(1)), float(wide.group(2)),
                                       float(wide.group(3)))
            self.nearest_wide_dist = float(wide.group(4))
        elif 'nearest_wide=inf' in (msg.data or ''):
            self.nearest_wide_delta = None
            self.nearest_wide_dist = float('inf')
        # 锥内连通簇质心（优先用于世界系复验）
        centroid = _NEAREST_CENTROID_RE.search(msg.data or '')
        if centroid:
            self.nearest_centroid_delta = (float(centroid.group(1)), float(centroid.group(2)),
                                           float(centroid.group(3)))
            self.nearest_centroid_dist = float(centroid.group(4))
            self.nearest_centroid_size = int(centroid.group(5))
        elif 'nearest_centroid=inf' in (msg.data or ''):
            self.nearest_centroid_delta = None
            self.nearest_centroid_dist = float('inf')
            self.nearest_centroid_size = 0

    def _on_depth_image(self, msg: Image) -> None:
        """SEEK 阶段用：**几何去地面**后取最近结构点的方位与距离。

        做法（服务器侧 2026-10-14 建议）：
        1. 深度像素按相机内参（HFOV + 宽高比推 VFOV）反投影成机体 FLU 点；
        2. 只保留 ``up > -(高度 - ground_margin)`` 的点 —— 地面被精确剔除，且不依赖行带比例；
        3. 取水平最近的保留点，记录"世界方位扇区 ← 该点距离（取各帧最小值）"。
        """
        if self.phase != 'SEEK' or msg.encoding != '32FC1':
            return
        depth = np.frombuffer(msg.data, dtype=np.float32)
        if depth.size < int(msg.width) * int(msg.height):
            return
        height, width = int(msg.height), int(msg.width)
        image = depth.reshape(height, width)[::self._seek_step, ::self._seek_step]
        rows, cols = image.shape
        valid = np.isfinite(image) & (image > 0.1) & (image < 500.0)
        if not np.any(valid):
            return
        ys, xs = np.mgrid[0:height:self._seek_step, 0:width:self._seek_step]
        x_norm = xs[valid] / max(width - 1, 1)
        y_norm = ys[valid] / max(height - 1, 1)
        depth_m = image[valid]
        tan_h = math.tan(math.radians(self._seek_hfov) / 2.0)
        aspect = width / max(height, 1)
        tan_v = tan_h / aspect
        forward = depth_m
        left = -(x_norm - 0.5) * 2.0 * depth_m * tan_h
        up = (0.5 - y_norm) * 2.0 * depth_m * tan_v
        if self.px4_z is None:
            return
        altitude = abs(float(self.px4_z))
        keep = (up > -(altitude - self._seek_ground_margin)) & (forward < self._seek_max_range)
        if not np.any(keep):
            return
        forward, left, up = forward[keep], left[keep], up[keep]
        horizontal = np.hypot(forward, left)
        body_bearings = np.arctan2(left, forward)
        vertical = np.degrees(np.arctan2(up, np.maximum(horizontal, 1e-6)))
        world_bearings = np.degrees(ned_yaw_to_enu_yaw(self.heading_rad)) + np.degrees(
            body_bearings)
        sectors = (world_bearings // 30).astype(int) * 30
        for sector in np.unique(sectors):
            mask = sectors == sector
            samples = self._seek_samples.setdefault(int(sector), [])
            if len(samples) >= 600:          # 限幅，避免长时间扫描吃内存
                continue
            samples.extend(zip(horizontal[mask].tolist(), vertical[mask].tolist()))

    def _beat(self) -> None:
        msg = String()
        msg.data = f'{self.phase} t={self.elapsed():.1f}'
        self.heartbeat_pub.publish(msg)

    def _on_position(self, msg) -> None:
        super()._on_position(msg)
        if getattr(msg, 'xy_valid', True):
            self.px4_x = float(msg.x)
            self.px4_y = float(msg.y)
            self._position_at = time.monotonic()
        heading = getattr(msg, 'heading', None)
        if heading is None or math.isnan(heading):
            return
        self.heading_rad = float(heading)
        # 解卷绕：NED 航向是 [-π, π]，跨 ±π 处要按最短方向累加，否则算不出总转角
        if self._heading_prev is not None:
            delta = math.remainder(self.heading_rad - self._heading_prev, 2 * math.pi)
            self._heading_unwrapped_deg += math.degrees(delta)
        self._heading_prev = self.heading_rad

    # ------------------------------------------------------------- 记录

    def record(self, phase: str) -> None:
        decision = self.latest_decision or {}
        if decision.get('frame_seq') == self._last_seq:
            return
        self._last_seq = decision.get('frame_seq')
        if phase == 'RUN' and self._origin_xy is None and not math.isnan(self.px4_x):
            self._origin_xy = (self.px4_x, self.px4_y)
        drift = (math.hypot(self.px4_x - self._origin_xy[0], self.px4_y - self._origin_xy[1])
                 if self._origin_xy and not math.isnan(self.px4_x) else None)
        waypoints = decision.get('waypoints') or []
        first = waypoints[0] if waypoints else {}
        velocity = decision.get('body_velocity') or {}
        self.rows.append({
            't': round(self.elapsed(), 2),
            'phase': phase,
            'heading_unwrapped_deg': round(self._heading_unwrapped_deg, 2),
            'heading_deg': (round(math.degrees(self.heading_rad), 2)
                            if not math.isnan(self.heading_rad) else None),
            'px4_z': (round(self.px4_z, 3) if self.px4_z is not None else None),
            'px4_x': None if math.isnan(self.px4_x) else round(self.px4_x, 3),
            'px4_y': None if math.isnan(self.px4_y) else round(self.px4_y, 3),
            'drift_m': None if drift is None else round(drift, 3),
            'frame_seq': decision.get('frame_seq'),
            'apos_state': decision.get('apos_state'),
            'forward_clear_m': (self.clearance[0] if self.clearance else None),
            'wp_yaw_deg': (math.degrees(first['yaw_rad'])
                           if isinstance(first.get('yaw_rad'), (int, float)) else None),
            'command_source': decision.get('command_source'),
            'guard': decision.get('guard'),
            'cmd_yaw_rate': velocity.get('yaw_rate'),
            'cmd_forward': velocity.get('forward'),
        })

    def record_approach(self) -> None:
        """摆位阶段没有 VLM 决策，单独记一行（保留同一套列名，便于统一分析）。"""
        now = round(self.elapsed(), 2)
        if self.rows and self.rows[-1]['phase'] == 'APPROACH' \
                and now - self.rows[-1]['t'] < 0.4:
            return
        row = {
            'frame_seq': None, 'apos_state': None, 'wp_yaw_deg': None, 'guard': None,
            'drift_m': None,
        }
        row.update({
            't': now,
            'phase': 'APPROACH',
            'heading_unwrapped_deg': round(self._heading_unwrapped_deg, 2),
            'heading_deg': (round(math.degrees(self.heading_rad), 2)
                            if not math.isnan(self.heading_rad) else None),
            'px4_z': (round(self.px4_z, 3) if self.px4_z is not None else None),
            'px4_x': None if math.isnan(self.px4_x) else round(self.px4_x, 3),
            'px4_y': None if math.isnan(self.px4_y) else round(self.px4_y, 3),
            'forward_clear_m': (self.clearance[0] if self.clearance else None),
            'command_source': 'approach',
            'cmd_forward': self.pre_speed,
            'cmd_yaw_rate': 0.0,
        })
        self.rows.append(row)

    def publish_enu_velocity(self, forward_mps: float) -> None:
        """按当前航向把"机头方向的前进速度"转成 ENU 速度发给桥接节点。

        摆位阶段我们用最简单的开环前进（不依赖 VLM），把飞机送到障碍前再交权。
        """
        if math.isnan(self.heading_rad):
            return
        heading_enu = ned_yaw_to_enu_yaw(self.heading_rad)
        east = forward_mps * math.cos(heading_enu)
        north = forward_mps * math.sin(heading_enu)
        msg = Twist()
        msg.linear.x = east
        msg.linear.y = north
        msg.linear.z = 0.0
        msg.angular.z = 0.0
        self.velocity_pub.publish(msg)

    def publish_yaw_rate(self, rate: float) -> None:
        """只给偏航角速度（速度模式），用于扫描阶段边转边看净空。"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.z = float(rate)
        self.velocity_pub.publish(msg)

    def publish_target_enu(self, east: float, north: float, up: float) -> None:
        """位置模式目标（ENU，绝对坐标）：转场与降高用。

        基类的 ``publish_target`` 只写 z（x,y 默认 0），会把飞机拉回原点——转场必须给全 x,y。
        """
        from geometry_msgs.msg import PoseStamped

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(east)
        msg.pose.position.y = float(north)
        msg.pose.position.z = float(up)
        msg.pose.orientation.w = 1.0
        self.target_pub.publish(msg)

    def position_enu(self):
        """当前 ENU 位置（拿不到返回 None）。"""
        if self.px4_z is None or math.isnan(self.px4_x) or math.isnan(self.px4_y):
            return None
        return ned_to_enu(self.px4_x, self.px4_y, self.px4_z)

    # ------------------------------------------------ 微转复验（服务器侧 B）

    @property
    def heading_enu_unwrapped_deg(self) -> float:
        """ENU 约定下的解卷绕航向（度）。

        ``publish_yaw_rate`` 用的是 ENU 角速度（正 = 逆时针/左），而
        ``_heading_unwrapped_deg`` 累积的是 **NED** 航向；ENU = 偏移 − NED，
        故取负即可（常数偏移在"两次采样之差"里抵消）。
        """
        return -self._heading_unwrapped_deg

    def _rot_sample(self, label: str) -> dict:
        """采一帧"世界系障碍点 + 距离 + 位姿质量"（微转复验用）。"""
        position = self.position_enu()
        # 世界点优先级（服务器侧 2026-10-14 的"限定 ②"）：**连通簇质心** > 宽锥最近点 > 窄锥最近点。
        # 质心代表"一个物体"，小姿态变化下比"锥内最近点"稳定得多（后者会在平面上滑移）。
        if self.nearest_centroid_delta is not None:
            delta, dist = self.nearest_centroid_delta, self.nearest_centroid_dist
        elif self.nearest_wide_delta is not None:
            delta, dist = self.nearest_wide_delta, self.nearest_wide_dist
        else:
            delta, dist = self.nearest_delta, self.nearest_dist
        world = None
        if position is not None and delta is not None:
            world = (position[0] + delta[0], position[1] + delta[1], position[2] + delta[2])
        pose_age = (time.monotonic() - self._position_at) if self._position_at else None
        return {
            'label': label,
            'heading_deg': self.heading_enu_unwrapped_deg,
            'world': world,
            'dist': dist,
            'dist_narrow': self.nearest_dist,
            'dist_wide': self.nearest_wide_dist,
            'centroid_size': self.nearest_centroid_size,
            'pose_age_s': None if pose_age is None else round(pose_age, 2),
            'fwd_m': None if not self.clearance else round(self.clearance[0], 2),
        }

    @staticmethod
    def _fmt_world(world) -> str:
        return '无' if world is None else f'({world[0]:+.1f},{world[1]:+.1f},{world[2]:+.1f})'

    # ------------------------------------------------------------- 阶段机

    def first_phase(self) -> str:
        return 'CLIMB'

    @staticmethod
    def _parse_points(text: str):
        """解析多点字符串 → [(east, north), ...]。

        分隔符支持 `;`、`|` 或空白（ROS 参数里 `;` 可能被 shell/CLI 吃掉，
        2026-10-14 实测 `"0,15;12,0"` 传进来变成空 → 这里多支持几种写法，
        文档/命令统一用 `|`）。
        """
        points = []
        normalized = (text or '').replace(';', '|').replace(',,', ',')
        for chunk in normalized.replace('\n', '|').split('|'):
            chunk = chunk.strip()
            if not chunk:
                continue
            if ',' not in chunk and ' ' in chunk:
                chunk = ','.join(chunk.split())
            try:
                east, north = (float(v) for v in chunk.split(','))
            except ValueError:
                continue
            points.append((east, north))
        return points

    def _current_point(self):
        if self._points:
            return self._points[min(self._point_index, len(self._points) - 1)]
        return (self.transit_east, self.transit_north)

    def _next_point(self) -> bool:
        """切到下一个候选点；用尽返回 False（调用方应降落收工）。"""
        self._point_index += 1
        if self._point_index >= len(self._points):
            return False
        self._scan_hits = 0
        self._verify_passed = 0
        self._realign_count = 0
        self._rot_step = 0
        self._rot_samples = []
        return True

    def enter(self, phase: str) -> None:
        """进入 RUN（交权）时通知提前跑着的 VLM 节点解除 dry_run。"""
        super().enter(phase)
        if phase == 'ARM' and self._flight_started is None:
            self._flight_started = time.monotonic()
        if phase == 'RUN':
            msg = String()
            msg.data = 'enable'
            self.enable_pub.publish(msg)
            self.get_logger().info('已向 VLM 节点发送 enable（解除 dry_run）')

    def on_phase(self) -> None:
        # 全局时限兜底：任何阶段卡住/超预算都收工降落（阶段级超时之外的第二道闸）
        if (self._flight_started is not None
                and time.monotonic() - self._flight_started > self.max_flight_s):
            self.get_logger().warn(
                f'全局飞行时限 {self.max_flight_s:.0f} s 到点（当前阶段 {self.phase}）→ 收工降落')
            self.send('land_fast')
            self.finish()
            return
        if self.phase == 'CLIMB':
            if self.climb_to(self.altitude):
                if self.pre_position:
                    if self.seek_enabled:
                        self.get_logger().info(
                            f'到达 {self.altitude:.1f} m，先做**深度自动寻障**：'
                            f'{math.degrees(self.seek_yaw_rate):.1f}°/s × {self.seek_s:.0f} s')
                        self.enter('SEEK')
                    elif self.transit_enabled or self._points:
                        self.get_logger().info(
                            f'多点扫描配置：{len(self._points)} 个候选点 {self._points}'
                            if self._points else '转场配置：无候选点列表（用 transit_east/north）')
                        self.get_logger().info(
                            f'到达 {self.altitude:.1f} m，先转场到 ENU（东 {self.transit_east:+.1f}, '
                            f'北 {self.transit_north:+.1f}），转场高度 {self.transit_altitude:.1f} m')
                        self.enter('TRANSIT')
                    else:
                        self.get_logger().info(
                            f'到达 {self.altitude:.1f} m，开始摆位：以 {self.pre_speed:.2f} m/s '
                            f'前进直到前向净空 ≤ {self.pre_stop:.1f} m')
                        self.enter('SCAN' if self.pre_scan else 'APPROACH')
                else:
                    self.get_logger().info(
                        f'到达 {self.altitude:.1f} m，交出控制权给 VLM（本脚本不再发目标点）')
                    self.enter('RUN')
            elif self.elapsed() > self.phase_timeout:
                self.fail('爬升超时')
            return

        if self.phase == 'SEEK':
            # 深度自动寻障：慢转一圈，边转边按扇区累计中位深度
            self.publish_yaw_rate(self.seek_yaw_rate)
            self.record_approach()
            if self.elapsed() <= self.seek_s:
                return
            self.publish_yaw_rate(0.0)
            if not self._seek_samples:
                self.get_logger().warn('深度寻障：整圈都没有有效回波 → 收工降落')
                self.send('land_fast')
                self.finish()
                return
            candidates = []
            for sector, samples in self._seek_samples.items():
                if not samples:
                    continue
                distances = np.array([s[0] for s in samples])
                verticals = np.array([s[1] for s in samples])
                median = float(np.median(distances))
                mad = float(np.median(np.abs(distances - median)))
                span = float(verticals.max() - verticals.min())
                count = int(distances.size)
                # 质量门限（点数/垂跨度/MAD）+ 中位距离下限：只有"够大、够平、够远"的
                # 扇区才算合格候选（服务器侧 2026-10-14 ①：最近的**合格大结构**）。
                qualifies = (count >= self._seek_min_points
                             and span >= self._seek_min_span
                             and mad <= self._seek_max_mad
                             and median >= self._seek_min_median)
                fails = []
                if count < self._seek_min_points:
                    fails.append(f'点<{self._seek_min_points}')
                if span < self._seek_min_span:
                    fails.append(f'跨度<{self._seek_min_span}°')
                if mad > self._seek_max_mad:
                    fails.append(f'MAD>{self._seek_max_mad}')
                if median < self._seek_min_median:
                    fails.append(f'中位<{self._seek_min_median}m')
                candidates.append({'sector': int(sector), 'median': median, 'mad': mad,
                                   'span': span, 'count': count, 'ok': qualifies,
                                   'fail': '/'.join(fails) or '—'})
            good = sorted((c for c in candidates if c['ok']), key=lambda c: c['median'])
            # 打印**全部扇区**（不只是前几个）：便于判断"是门限太严，还是场景真没有结构"
            # （服务器侧 2026-10-14 第五节建议）。
            summary = [(c['sector'], round(c['median'], 1), c['count'],
                        round(c['span'], 1), round(c['mad'], 2),
                        'OK' if c['ok'] else c['fail']) for c in
                       sorted(candidates, key=lambda c: c['median'])]
            self.get_logger().info(
                f'深度寻障统计（扇区, 中位 m, 点数, 垂跨度°, MAD m, 判定）'
                f'[门限：点≥{self._seek_min_points}/跨度≥{self._seek_min_span}°/'
                f'MAD≤{self._seek_max_mad}/中位≥{self._seek_min_median}m]：' + str(summary))
            if not good:
                self.get_logger().warn(
                    f'没有任何扇区同时满足"点数≥{self._seek_min_points}/垂跨度≥'
                    f'{self._seek_min_span}°/MAD≤{self._seek_max_mad}/中位≥'
                    f'{self._seek_min_median} m"门限 → 收工降落（先看上面的统计定位是哪一项）')
                self.send('land_fast')
                self.finish()
                return
            self._points = []
            for candidate in good[:self._seek_max_candidates]:
                bearing = math.radians(candidate['sector'] + 15.0)
                distance = min(max(candidate['median'] - self.seek_margin, self.seek_min),
                               self.seek_max)
                self._points.append((distance * math.cos(bearing), distance * math.sin(bearing)))
            self._point_index = 0
            self.get_logger().info(
                f'深度寻障结果：合格扇区 {len(good)} 个，最优 扇区 {good[0]["sector"]}° '
                f'中位 {good[0]["median"]:.1f} m（点 {good[0]["count"]}、跨度 '
                f'{good[0]["span"]:.1f}°、MAD {good[0]["mad"]:.2f} m）→ 生成 '
                f'{len(self._points)} 个候选转场点：'
                + str([(round(p[0], 1), round(p[1], 1)) for p in self._points]))
            self.enter('TRANSIT')
            return

        if self.phase in ('TRANSIT', 'DESCEND'):
            altitude = self.transit_altitude if self.phase == 'TRANSIT' else self.altitude
            target_east, target_north = self._current_point()
            self.publish_target_enu(target_east, target_north, altitude)
            self.record_approach()
            current = self.position_enu()
            close = (current is not None
                     and abs(current[0] - target_east) < 0.5
                     and abs(current[1] - target_north) < 0.5
                     and abs(current[2] - altitude) < 0.4)
            if close:
                if self.phase == 'TRANSIT':
                    self.get_logger().info(
                        f'转场到位（ENU 东 {current[0]:+.1f} 北 {current[1]:+.1f} '
                        f'上 {current[2]:.1f}），降到扫描高度 {self.altitude:.1f} m')
                    self.enter('DESCEND')
                else:
                    self.get_logger().info(
                        f'已降到 {self.altitude:.1f} m，开始扫描/摆位')
                    self.enter('SCAN' if self.pre_scan else 'APPROACH')
            elif self.elapsed() > self.transit_timeout:
                self.get_logger().warn(
                    f'{self.phase} 超时（{self.transit_timeout:.0f} s，'
                    f'当前 {current}）→ 就地继续')
                self.enter('SCAN' if self.pre_scan else 'APPROACH')
            return

        if self.phase == 'ALIGN':
            # 转到位再逼近（服务器侧建议）：用扫描确认方位作为目标航向
            if self._align_heading is None or math.isnan(self.heading_rad):
                self.enter('APPROACH')
                return
            current = ned_yaw_to_enu_yaw(self.heading_rad)
            error = math.remainder(self._align_heading - current, 2.0 * math.pi)
            tolerance = math.radians(float(self.get_parameter('align_tolerance_deg').value))
            if abs(error) <= tolerance:
                self.publish_yaw_rate(0.0)
                side_max = (max(self.clearance[1], self.clearance[2])
                            if self.clearance else None)
                if side_max is not None and side_max < self._closable_side:
                    self.get_logger().warn(
                        f'对准完成（误差 {math.degrees(error):+.1f}°）但两侧都紧'
                        f'（左 {self.clearance[1]:.2f} / 右 {self.clearance[2]:.2f}，'
                        f'< {self._closable_side:.1f} m）→ 绕行无法闭合，换下一个候选点')
                    if self._next_point():
                        target_east, target_north = self._current_point()
                        self.get_logger().info(
                            f'转场到下一个候选点（东 {target_east:+.0f}, 北 {target_north:+.0f}）')
                        self.enter('TRANSIT')
                    else:
                        self.get_logger().warn('候选点用尽 → 收工降落')
                        self.send('land_fast')
                        self.finish()
                    return
                # 对准完成且（若已知）至少一侧开阔 → 进入逼近
                # 注意：以下三行必须与 `if side_max ...` 同级——曾经因缩进被塞进内层
                # `return` 之后变成死代码，导致"对准成功也永远卡在 ALIGN"（2026-10-14 回归抓到）
                self.get_logger().info(
                    f'对准完成（误差 {math.degrees(error):+.1f}°，宽侧净空 '
                    f'{side_max if side_max is None else round(side_max, 2)} m）→ 开始以 '
                    f'{self.pre_speed:.2f} m/s 逼近')
                self._approach_confirmed = False
                self.enter('APPROACH')
                return
            if self.elapsed() > float(self.get_parameter('align_timeout_s').value):
                self.publish_yaw_rate(0.0)
                self.get_logger().warn(
                    f'对准超时（残留误差 {math.degrees(error):+.1f}°）→ 仍转入逼近')
                self.enter('APPROACH')
                return
            limit = float(self.get_parameter('align_yaw_rate').value)
            self.publish_yaw_rate(max(-limit, min(limit, 1.2 * error)))
            return

        if self.phase == 'APPROACH':
            forward_clear = self.clearance[0] if self.clearance else None
            # 逼近中**持续复验**：净空回升/消失（障碍脱离视野）→ 停下重新对准
            if forward_clear is not None and forward_clear >= self.scan_found:
                self._lost_counter += 1
            else:
                self._lost_counter = 0
            if (self._lost_counter >= self.pre_stable
                    and self._realign_count < int(self.get_parameter('realign_max').value)):
                self.publish_enu_velocity(0.0)
                self._realign_count += 1
                self._lost_counter = 0
                self._clearance_history = []
                self.get_logger().warn(
                    f'逼近中净空回升到 {forward_clear:.2f} m（障碍脱离视野）→ 停下重新对准'
                    f'（第 {self._realign_count} 次）')
                self._align_heading = (ned_yaw_to_enu_yaw(self.heading_rad)
                                       if self._align_heading is None else self._align_heading)
                self.enter('ALIGN')
                return
            if (self._lost_counter >= self.pre_stable
                    and self._realign_count >= int(self.get_parameter('realign_max').value)):
                # 服务器侧建议：re-align 到上限仍未稳定 → **该候选点作废**，换下一个合格扇区，
                # 而不是带着不确定目标交权（那样成功率不可解释）
                self.publish_enu_velocity(0.0)
                self.get_logger().warn(
                    f'净空回升且重新对准已达上限（{self._realign_count} 次）→ 该候选点作废')
                if self._next_point():
                    target_east, target_north = self._current_point()
                    self.get_logger().info(
                        f'转场到下一个候选点（东 {target_east:+.0f}, 北 {target_north:+.0f}）')
                    self.enter('TRANSIT')
                else:
                    self.get_logger().warn('候选点用尽（含 SEEK 产出的候选）→ 收工降落')
                    self.send('land_fast')
                    self.finish()
                return
            if forward_clear is not None:
                self._clearance_history.append(forward_clear)
                self._clearance_history = self._clearance_history[-self.pre_stable:]
            stable = (len(self._clearance_history) >= self.pre_stable
                      and all(value <= self.pre_stop for value in self._clearance_history))
            if stable:
                self.publish_enu_velocity(0.0)
                self._approach_confirmed = True
                self.get_logger().info(
                    f'摆位完成：前向净空连续 {self.pre_stable} 次 ≤ {self.pre_stop:.1f} m'
                    f'（最近 {[round(v, 2) for v in self._clearance_history]}），'
                    '停下并交出控制权给 VLM')
                self.enter('RUN')
                return
            if self.elapsed() > self.pre_timeout:
                self.publish_enu_velocity(0.0)
                # 服务器侧 2026-10-14：**没到过停止阈值 = 目标不确定 → 不交权**，
                # 作废该候选点、回 SEEK 选下一个（否则 VLM 对着"空"去绕行，双重失败）
                self.get_logger().warn(
                    f'摆位超时（{self.pre_timeout:.0f} s，前向净空 {forward_clear}）'
                    '且未到达停止阈值 → **不交权**，作废该候选点')
                if self._next_point():
                    target_east, target_north = self._current_point()
                    self.get_logger().info(
                        f'转场到下一个候选点（东 {target_east:+.0f}, 北 {target_north:+.0f}）')
                    self.enter('TRANSIT')
                else:
                    self.get_logger().warn('候选点用尽 → 收工降落（不交权）')
                    self.send('land_fast')
                    self.finish()
                return
            self.publish_enu_velocity(self.pre_speed)
            self.record_approach()
            return

        if self.phase == 'SCAN':
            # 慢速偏航扫一圈，边转边看本机地图净空（服务器侧 2026-10-13 确认的做法）
            self.publish_yaw_rate(self.scan_yaw_rate)
            self.record_approach()          # 与摆位共用记录（同一套列）
            forward_clear = self.clearance[0] if self.clearance else None
            sides_open = (self.clearance is not None
                          and max(self.clearance[1], self.clearance[2]) >= self.scan_side_open)
            if forward_clear is not None and forward_clear <= self.scan_found:
                self._scan_hits += 1
                self._scan_best = forward_clear
                if sides_open and self._scan_hits >= self.pre_stable:
                    self.publish_yaw_rate(0.0)
                    if self.rot_verify:
                        self.get_logger().info(
                            f'扫描发现候选障碍（前向 {forward_clear:.2f} m，'
                            f'左 {self.clearance[1]:.2f} / 右 {self.clearance[2]:.2f}）'
                            f'→ 停转，做**交权前 ±{self._rot_deg:.0f}° 微转复验**'
                            f'（真结构在世界系固定；拖影会随姿态改变，服务器侧 2026-10-14 选 B）')
                        self._rot_step = 0
                        self._rot_samples = []
                        self._rot_step_started = time.monotonic()
                        self._align_heading = (None if math.isnan(self.heading_rad)
                                               else ned_yaw_to_enu_yaw(self.heading_rad))
                        self.enter('ROTVERIFY')
                        return
                    self.get_logger().info(
                        f'扫描发现可用障碍：前向净空连续 {self._scan_hits} 次 ≤ '
                        f'{self.scan_found:.1f} m（最近 {forward_clear:.2f} m），'
                        f'且一侧开阔（左 {self.clearance[1]:.2f} / 右 {self.clearance[2]:.2f}）'
                        '→ 停转，先**原地静止复验**（排除自运动拖影）')
                    self._verify_history = []
                    self.enter('VERIFY')
                    return
                if not sides_open:
                    self.get_logger().info(
                        f'该方位净空 {forward_clear:.2f} m 但两侧都紧'
                        f'（左 {self.clearance[1]:.2f} / 右 {self.clearance[2]:.2f}）'
                        '→ 不适合绕行，继续扫', throttle_duration_sec=2.0)
                    self._scan_hits = 0
            else:
                self._scan_hits = 0
            if self.elapsed() > self.scan_max_s:
                self.publish_yaw_rate(0.0)
                if self._next_point():
                    target_east, target_north = self._current_point()
                    self.get_logger().warn(
                        f'本点扫描 {self.scan_max_s:.0f} s 未找到可复验障碍 → 转场到下一个候选点 '
                        f'(东 {target_east:+.0f}, 北 {target_north:+.0f})')
                    self.enter('TRANSIT')
                else:
                    self.get_logger().warn(
                        f'全部候选点扫描完毕仍未找到可复验障碍（{self.scan_max_s:.0f} s/点）'
                        '→ 按多点预算收工降落')
                    self.send('land_fast')
                    self.finish()
            return

        if self.phase == 'ROTVERIFY':
            # 交权前 ±rot_deg 微转复验（服务器侧 2026-10-14 选 B）：
            # 真结构在世界系固定 → 左右两个姿态下"世界坐标 + 距离"都近似不变；
            # 自运动拖影会随姿态改变或消失。它同时把"转场到位后结构不在前向锥里"
            # 的情形扫进锥内（顺带修 Form A）。
            self.record_approach()
            step_elapsed = time.monotonic() - self._rot_step_started

            if self._rot_step in (0, 2, 4):
                # 静置：等地图刷新、姿态稳定后再采样（也是"位姿质量门"的等待期）
                self.publish_yaw_rate(0.0)
                if step_elapsed < self._rot_settle:
                    return
                label = {0: 'A', 2: 'B', 4: 'C'}[self._rot_step]
                pose_age = (time.monotonic() - self._position_at
                            if self._position_at else None)
                if pose_age is not None and pose_age > self._rot_pose_age_max:
                    # 位姿不新鲜：**暂缓判定**（既不当真障碍也不判拖影），等它恢复
                    if step_elapsed > self._rot_timeout:
                        self.get_logger().warn(
                            f'微转复验：位姿 {pose_age:.2f} s 未更新（> '
                            f'{self._rot_pose_age_max:.2f} s）→ 本轮不判定，回到扫描')
                        self.enter('SCAN')
                    else:
                        self.get_logger().warn(
                            f'微转复验：位姿陈旧（{pose_age:.2f} s）→ 暂缓判定，等待位姿恢复',
                            throttle_duration_sec=2.0)
                    return
                sample = self._rot_sample(label)
                self._rot_samples = [s for s in self._rot_samples if s['label'] != label]
                self._rot_samples.append(sample)
                if label == 'A':
                    # 以"候选姿态"为 ±微转的锚点（此时仍是扫描确认障碍时的航向）
                    self._rot_anchor_deg = self.heading_enu_unwrapped_deg
                self.get_logger().info(
                    f'  微转复验采样 {label}：航向 {sample["heading_deg"]:+.1f}°，'
                    f'世界点 {self._fmt_world(sample["world"])}，距离 '
                    f'{sample["dist"]:.2f} m [宽锥 {sample["dist_wide"]:.2f} / '
                    f'窄锥 {sample["dist_narrow"]:.2f} m，簇 {sample["centroid_size"]} 体素]，'
                    f'位姿年龄 {sample["pose_age_s"]} s')
                self._rot_step += 1
                self._rot_step_started = time.monotonic()
                return

            if self._rot_step in (1, 3):
                # 转向目标：先 +rot_deg（步骤 1），再回到 -rot_deg（步骤 3）
                target = self._rot_anchor_deg + (
                    self._rot_deg if self._rot_step == 1 else -self._rot_deg)
                error = target - self.heading_enu_unwrapped_deg
                if abs(error) > 2.0:
                    if step_elapsed > self._rot_timeout:
                        self.get_logger().warn(
                            f'微转复验：转向超时（误差 {error:+.1f}°）→ 本轮不判定，回到扫描')
                        self.enter('SCAN')
                        return
                    self.publish_yaw_rate(self._rot_rate if error > 0 else -self._rot_rate)
                    return
                self.publish_yaw_rate(0.0)
                self._rot_step += 1
                self._rot_step_started = time.monotonic()
                return

            # 步骤 5：判定
            samples = {s['label']: s for s in self._rot_samples}
            b, c = samples.get('B'), samples.get('C')
            if b is None or c is None or b['world'] is None or c['world'] is None:
                self.get_logger().info(
                    '微转复验不通过：±两侧至少一侧看不到前向锥内障碍 → 继续扫描')
                self._scan_hits = 0
                self.enter('SCAN')
                return
            world_gap = math.dist(b['world'], c['world'])
            dist_gap = abs(b['dist'] - c['dist'])
            if world_gap <= self._rot_tol and dist_gap <= self._rot_dist_tol:
                self.get_logger().info(
                    f'微转复验通过：±{self._rot_deg:.0f}° 两姿态下世界点偏差 '
                    f'{world_gap:.2f} m、距离差 {dist_gap:.2f} m '
                    f'（≤ {self._rot_tol:.1f} / {self._rot_dist_tol:.1f} m）→ 真结构，先对准再逼近')
                self._clearance_history = []
                self.enter('ALIGN')
                return
            self.get_logger().info(
                f'微转复验不通过：世界点偏差 {world_gap:.2f} m、距离差 {dist_gap:.2f} m '
                f'（超 {self._rot_tol:.1f} / {self._rot_dist_tol:.1f} m，疑似自运动拖影）'
                '→ 继续扫描')
            self._scan_hits = 0
            self.enter('SCAN')
            return

        if self.phase == 'VERIFY':
            # 原地静止复验：自运动拖影在停下 2–3 s 后会消失，真障碍会稳定留在净空里
            self.publish_yaw_rate(0.0)
            self.record_approach()
            if self.clearance is not None:
                self._verify_history.append(self.clearance[0])
                self._verify_history = self._verify_history[-self.pre_stable:]
            if self.elapsed() > self.verify_s:
                stable = (len(self._verify_history) >= self.pre_stable
                          and all(v <= self.scan_found for v in self._verify_history))
                values = [round(v, 2) for v in self._verify_history]
                if stable:
                    self._verify_passed += 1
                    if self._verify_passed >= self._verify_rounds:
                        self.get_logger().info(
                            f'静止复验通过 {self._verify_passed}/{self._verify_rounds} 轮：'
                            f'净空 {values} 仍 ≤ {self.scan_found:.1f} m → 真障碍，先对准再逼近')
                        self._clearance_history = []
                        self._align_heading = ned_yaw_to_enu_yaw(self.heading_rad)
                        self.enter('ALIGN')
                    else:
                        self.get_logger().info(
                            f'静止复验第 {self._verify_passed}/{self._verify_rounds} 轮通过'
                            f'（净空 {values}）→ 再做一轮确认（服务器侧建议的"二次通过"）')
                        self._verify_history = []
                        self.enter('VERIFY')
                else:
                    self.get_logger().info(
                        f'静止复验未通过：净空 {values}（疑似自运动拖影）→ 继续扫描')
                    self._scan_hits = 0
                    self._verify_passed = 0
                    self.enter('SCAN')
            return

        if self.phase == 'RUN':
            self.record('RUN')
            if self.elapsed() > self.run_s:
                self.finish()
            return

    # ------------------------------------------------------------- 汇总

    def finish(self) -> None:
        rows = [r for r in self.rows if r['phase'] == 'RUN']
        states = {}
        for row in rows:
            states[row['apos_state'] or '-'] = states.get(row['apos_state'] or '-', 0) + 1
        yaw_cmds = [r['cmd_yaw_rate'] for r in rows if isinstance(r['cmd_yaw_rate'], (int, float))]
        positives = sum(1 for v in yaw_cmds if v > 0.05)
        negatives = sum(1 for v in yaw_cmds if v < -0.05)
        delta = self._heading_unwrapped_deg

        if self.output_csv:
            directory = os.path.dirname(self.output_csv)
            if directory:
                os.makedirs(directory, exist_ok=True)
            if self.rows:
                with open(self.output_csv, 'w', newline='', encoding='utf-8') as handle:
                    writer = csv.DictWriter(handle, fieldnames=list(self.rows[0].keys()))
                    writer.writeheader()
                    writer.writerows(self.rows)

        print('\n=========== 闭环小结（C1 只偏航 / C2 加前进）===========')
        print(f'运行 {len(rows)} 帧；apos_state 分布 {states}')
        print(f'我方偏航指令：正（=左）{positives} 帧 / 负（=右）{negatives} 帧')
        print(f'航向总变化：{delta:+.1f}°（PX4 NED：**负 = 左转**）')
        drifts = [r['drift_m'] for r in rows if isinstance(r['drift_m'], (int, float))]
        if drifts:
            print(f'水平位移：末值 {drifts[-1]:.2f} m / 最大 {max(drifts):.2f} m（围栏 5 m 判据）')
        speeds = [r['cmd_forward'] for r in rows if isinstance(r['cmd_forward'], (int, float))]
        if speeds:
            print(f'前进指令：最大 {max(abs(v) for v in speeds):.2f} m/s')
        print('\n判据：若"正（左）指令占多数"且航向变化为**负**，则方向的符号链正确；'
              '若航向变化为正，说明反号，必须停机改映射。')
        if self.land_at_end:
            self.send('land_fast')
            print('已下发 land_fast。')
        print(f'CSV：{self.output_csv}')
        raise SystemExit(0)


def main() -> int:
    rclpy.init()
    node = ClosedLoopBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.send('land_fast')
    except Exception as exc:  # noqa: BLE001
        # 顶层兜底：脚本自己崩了也必须先落地（今天踩过——汇总崩溃时保护一起没了）。
        # 注意只拦 Exception：正常收尾走 SystemExit(0)，不该被记成异常。
        node.get_logger().error(f'驱动脚本异常退出（{type(exc).__name__}: {exc}），先下发 land_fast')
        node.send('land_fast')
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
