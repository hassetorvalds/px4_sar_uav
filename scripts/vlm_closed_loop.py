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
# local_map 的净空"地板值"（fwd_floor_m=0.45）：读数贴着它 = 触底/回波可疑
_FLOOR_RE = re.compile(r'fwd_floor_m=([0-9.]+)')


def is_ego_echo(delta_a, delta_b, tol: float = 0.08) -> bool:
    """偏航前后"机体坐标系里的最近点偏移"几乎不变 → 判定为**自机体回波**。

    真障碍：偏航后它的**方位**在机体系里会变（世界位置不动）；
    自机体回波：在机体系里**固定**（随飞机一起转），所以偏航前后偏移几乎相同。
    任一为 None 时返回 False（不敢断言，按"真障碍"保守处理）。
    """
    if delta_a is None or delta_b is None:
        return False
    return sum((float(a) - float(b)) ** 2 for a, b in zip(delta_a, delta_b)) ** 0.5 <= tol

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
        # **高度门**（服务器侧 2026-10-17）：2 m 飞行时 0.6 m 的低矮物可直接飞越，不是障碍。
        # 只有点簇**顶部高度 ≥ 飞行高度 + 该余量**的结构才算"需要绕行的障碍"。
        # 0.0 = 顶部达到飞行高度即可；调大可要求"明显高于航路"。
        self.declare_parameter('seek_top_margin_m', 0.0)
        self.declare_parameter('seek_max_candidates', 3)    # 最多产出几个候选点（按中位距离排序）
        # 转场（服务器侧 2026-10-13 建议 ①）：高一点飞过去，再降到扫描高度
        self.declare_parameter('transit_enabled', False)
        self.declare_parameter('transit_east_m', 0.0)
        self.declare_parameter('transit_north_m', 0.0)
        self.declare_parameter('transit_altitude_m', 3.5)
        self.declare_parameter('transit_timeout_s', 45.0)
        # ⚠️ 转场限速 + 前方净空硬停（2026-10-15，用户报"高速撞障碍"后加）：
        # 位置目标点式转场会被 PX4 以巡航速度全速执行；改成"每周期只推进一点点"的追踪点，
        # 并把速度压到人控级别，前方净空过低时就地停住。
        self.declare_parameter('transit_speed_mps', 1.0)
        self.declare_parameter('transit_stop_clearance_m', 1.0)
        self.declare_parameter('transit_yaw_rate', 0.4)   # 转场时朝目标对齐的偏航角速度上限
        # 固定起始世界位姿（服务器侧 2026-10-15 要求 ②）：
        # 不要调 AirSim reset()（会断 lockstep）；改为**每轮起飞后用位置控制飞到固定世界位姿**
        # 再开始 SEEK，使各轮起点一致、命中率可横向比较。坐标用 AirSim 世界系（NED，米）。
        self.declare_parameter('start_pose_enabled', False)
        self.declare_parameter('start_world_north_m', 0.0)
        self.declare_parameter('start_world_east_m', 0.0)
        # 已知结构点位（走 C 的闭合回归用）：`scan_points` 里的坐标按 **AirSim 世界系**解释。
        # 世界→局部换算：先由 start_pose 把飞机摆到世界原点，记下"世界原点对应的局部 ENU"，
        # 之后 局部 = 世界 + 该偏移。这样不受 PX4 原点漂移影响（2026-10-16）。
        self.declare_parameter('scan_points_world', False)
        # 转场"贴地板读数"的主动判别（2026-10-15）：净空贴着 min_range 地板（≈0.45）时，
        # 可能是自机体回波（相机看到自家机架），也可能是真障碍。
        # 判据：**后退一小段**再看机体偏移——真障碍的距离会随之变大（机体偏移变），
        #       自机体回波跟着飞机走（机体偏移不变）。
        # 注：不能用偏航做判别——转场期间 offboard_bridge 处于**位置模式**
        #     （trajectory_setpoint 的 yaw = NaN），角速度指令不生效（2026-10-15 实测偏航超时）。
        self.declare_parameter('transit_echo_back_m', 0.25)
        self.declare_parameter('transit_echo_settle_s', 1.5)
        self.declare_parameter('transit_echo_delta_tol_m', 0.08)
        self.declare_parameter('transit_echo_ignore_s', 5.0)
        self.declare_parameter('transit_floor_band_m', 0.08)         # 贴着地板多近算"可疑"
        # 回波判别要有界（服务器侧 2026-10-15）：单次转场最多判别 N 次 / 累计后退 ≤ 上限，
        # 避免"贴着面反复后退"
        self.declare_parameter('transit_echo_max_probes', 2)
        self.declare_parameter('transit_echo_max_back_total_m', 0.5)
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
        self.transit_speed = float(self.get_parameter('transit_speed_mps').value)
        self.transit_stop_clearance = float(
            self.get_parameter('transit_stop_clearance_m').value)
        self.transit_yaw_rate = float(self.get_parameter('transit_yaw_rate').value)
        self._transit_step_at = 0.0
        self.start_pose_enabled = bool(self.get_parameter('start_pose_enabled').value)
        self._start_world = (
            float(self.get_parameter('start_world_north_m').value),
            float(self.get_parameter('start_world_east_m').value))
        self._start_pose_done = False
        self._after_start_pose = False
        self._origin_local = None        # 世界原点对应的局部 ENU（由 start_pose 到位后记下）
        self._points_world = bool(self.get_parameter('scan_points_world').value)
        self._ever_armed = False         # 曾经解锁过？（用于"被外力降落→收工退出"）
        self._scan_started_at = 0.0      # 本点扫描起点（**不因复验失败重置**，避免死循环）
        self._scan_min_forward = None    # 本点扫描期间的诊断量（最小前向净空/航向/侧向）
        self._scan_min_heading = None
        self._scan_min_sides = None
        self._map_floor_m = None
        self._echo_probe = None          # 判别状态机（None / dict）
        self._echo_ignore_until = 0.0    # 判定为回波后，短期内忽略"可疑带"读数
        self._echo_back_m = float(self.get_parameter('transit_echo_back_m').value)
        self._echo_settle_s = float(self.get_parameter('transit_echo_settle_s').value)
        self._echo_tol = float(self.get_parameter('transit_echo_delta_tol_m').value)
        self._echo_ignore_s = float(self.get_parameter('transit_echo_ignore_s').value)
        self._echo_band = float(self.get_parameter('transit_floor_band_m').value)
        self._echo_max_probes = max(
            int(self.get_parameter('transit_echo_max_probes').value), 1)
        self._echo_max_back_total = float(
            self.get_parameter('transit_echo_max_back_total_m').value)
        self._echo_probes_used = 0
        self._echo_back_total = 0.0
        self.enable_pub = self.create_publisher(
            String, str(self.get_parameter('vlm_enable_topic').value), 10)
        self._scan_hits = 0
        self._scan_best = None
        self._verify_history = []
        self._align_heading = None       # 目标航向（ENU）
        self._realign_count = 0
        self._lost_counter = 0
        self._points = self._parse_points(str(self.get_parameter('scan_points').value))
        # 已知结构点位也可用**环境变量**直传（KNOWN_POINTS="东,北;东,北;…"，AirSim 世界系）：
        # 绕开 `-p name:="a,b;c,d"` 这类"含分号字符串"在 shell/ROS 两层的解析坑（2026-10-16 踩到）。
        _env_points = os.environ.get('KNOWN_POINTS', '').strip()
        if _env_points:
            self._points = self._parse_points(_env_points)
            self._points_world = True
            self.get_logger().info(f'已知结构点位（来自环境变量 KNOWN_POINTS）：{self._points}')
        # 另存一份：起始位姿那段会临时把 `self._points` 当成"单点目标"用，到位后要还原
        self._known_points = list(self._points)
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
        self._seek_top_margin = float(self.get_parameter('seek_top_margin_m').value)
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
        floor = _FLOOR_RE.search(msg.data or '')
        if floor:
            self._map_floor_m = float(floor.group(1))

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

    def publish_velocity_enu(self, vx: float, vy: float, vz: float, yaw_rate: float) -> None:
        """ENU 世界速度 + 偏航角速度（速度模式）。

        转场必须用**速度**而不是位置目标点：位置目标点交给 PX4 会被它按自己的巡航速度执行
        （2026-10-15 高速撞障碍的根因）；而且"每周期推一点点位置"实测只跑出 ~0.11 m/s、
        98 s 都到不了 20 m 外的点（被监督层 TRANSIT 预算判超时）。速度控制能**精确限幅**。
        """
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = float(vz)
        msg.angular.z = float(yaw_rate)
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

    def _current_body_delta(self):
        """当前"最近点"在机体系下的 ENU 偏移（宽锥优先，其次窄锥）。"""
        return self.nearest_wide_delta if self.nearest_wide_delta is not None \
            else self.nearest_delta

    def _give_up_candidate(self, why: str) -> None:
        """标记当前候选点不可达 → 转下一个；用尽则收工降落。

        服务器侧 2026-10-15：转场受阻**不做复杂绕行**（那是 local_planner + 仲裁器的职责），
        驱动层只负责"这个点到不了就换下一个"，与"无确认不交权"同一哲学。
        """
        self.get_logger().warn(f'{why} → 该候选点标记为**不可达**，转下一个候选')
        self._echo_probes_used = 0
        self._echo_back_total = 0.0
        if self._next_point():
            target_east, target_north = self._current_point()
            self.get_logger().info(
                f'转场到下一个候选点（东 {target_east:+.0f}, 北 {target_north:+.0f}）')
            self.enter('TRANSIT')
            return
        self.get_logger().warn('候选点用尽 → 收工降落')
        self.send('land_fast')
        self.finish()

    def _resolve_world_points(self) -> None:
        """把"世界坐标的已知点位"换算成本机局部 ENU（一次性）。

        前提：调用时飞机已被 start_pose 摆到世界原点，且 `_origin_local` 已记下
        "世界原点对应的局部 ENU"。之后 局部 = 世界 + 偏移。
        """
        if not self._points_world:
            return
        if self._origin_local is None:
            self.get_logger().warn(
                '已知点位是世界坐标，但还没摆到世界原点（需 start_pose_enabled:=true）→ 按局部坐标处理')
            return
        off_e, off_n = self._origin_local
        self._points = [(e + off_e, n + off_n) for (e, n) in self._points]
        self.get_logger().info(
            '已知点位（世界→局部）换算完成：'
            + str([(round(a, 1), round(b, 1)) for a, b in self._points]))

    def _echo_probe_tick(self, now: float, current, altitude: float) -> None:
        """自机体回波判别状态机：停住 → **后退一小段** → 静止采样 → 比较机体偏移。

        判据：真障碍在后退后距离变大（机体偏移的前向分量变化 ≈ 后退距离）；
        自机体回波跟着飞机走（机体偏移几乎不变）。
        （不能用偏航——转场期间是位置模式，角速度指令不生效。）
        """
        probe = self._echo_probe
        if probe is None:
            return
        if probe['phase'] == 'back':
            self.publish_target_enu(probe['target'][0], probe['target'][1], altitude)
            moved = math.hypot(current[0] - probe['start'][0],
                               current[1] - probe['start'][1])
            if moved < self._echo_back_m - 0.06 and now - probe['t0'] < 5.0:
                return                      # 还在后退途中
            probe['phase'] = 'settle'
            probe['t0'] = now
            return
        # settle：静止 settle_s 后采样比较
        self.publish_target_enu(probe['target'][0], probe['target'][1], altitude)
        if now - probe['t0'] < self._echo_settle_s:
            return
        delta1 = self._current_body_delta()
        echo = is_ego_echo(probe['delta0'], delta1, self._echo_tol)
        if echo:
            self._echo_ignore_until = now + self._echo_ignore_s
            self.get_logger().warn(
                f'回波判别：后退 {self._echo_back_m:.2f} m 后机体偏移几乎不变 '
                f'（{probe["delta0"]} → {delta1}）→ **判定为自机体回波**，'
                f'忽略该读数 {self._echo_ignore_s:.0f} s 继续转场（仍限速 {self.transit_speed:.1f} m/s）')
        else:
            self.get_logger().warn(
                f'回波判别：后退 {self._echo_back_m:.2f} m 后机体偏移明显变化'
                f'（{probe["delta0"]} → {delta1}）'
                f'→ **判定为真障碍**，转场保持停住')
        self._echo_probe = None

    def _local_target_for_world_pose(self):
        """把"AirSim 世界系目标位姿"换成**本机局部 ENU 目标**（用于固定起始位姿）。

        局部目标 = 当前局部 ENU + (世界目标 − 世界当前)。这样既不动 lockstep、也不调 reset()，
        每轮都能把飞机摆到同一个世界坐标（服务器侧 2026-10-15 要求 ②）。
        拿不到 AirSim 位姿时返回 ``None``（调用方跳过并记 WARN）。
        """
        current_local = self.position_enu()
        if current_local is None:
            return None
        client = getattr(self, 'client', None)
        if client is None:
            return None
        try:
            world = client.simGetVehiclePose().position      # AirSim 世界系（NED）
        except Exception:                                    # noqa: BLE001 - 拿不到就跳过
            return None
        # AirSim 世界 NED → 我们的 ENU 差量：东 = +y, 北 = +x
        d_east = self._start_world[1] - float(world.y_val)
        d_north = self._start_world[0] - float(world.x_val)
        return (current_local[0] + d_east, current_local[1] + d_north)

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
        self._scan_started_at = time.monotonic()   # 新点重新计时
        return True

    def enter(self, phase: str) -> None:
        """进入 RUN（交权）时通知提前跑着的 VLM 节点解除 dry_run。"""
        previous = self.phase
        super().enter(phase)
        # 扫描计时只在"真正开始扫描"时重置；从 ROTVERIFY/VERIFY 失败返回 SCAN 时**不重置**，
        # 否则"发现候选→复验失败→回 SCAN"会无限循环、scan_max_s 永远不到期（2026-10-15 实测）
        if phase == 'SCAN' and previous not in ('ROTVERIFY', 'VERIFY'):
            self._scan_started_at = time.monotonic()
            self._scan_min_forward = None
            self._scan_min_heading = None
            self._scan_min_sides = None
        if phase == 'TRANSIT':
            # 每段转场的回波判别预算独立（否则跨段累计会把后面的点也判成"用尽"）
            self._echo_probes_used = 0
            self._echo_back_total = 0.0
        if phase == 'ARM' and self._flight_started is None:
            self._flight_started = time.monotonic()
        if phase == 'RUN':
            msg = String()
            msg.data = 'enable'
            self.enable_pub.publish(msg)
            self.get_logger().info('已向 VLM 节点发送 enable（解除 dry_run）')

    def on_phase(self) -> None:
        if self.armed:
            self._ever_armed = True
        # 安全闸：**被外力降落/上锁就该收工退出**（2026-10-15 实测的坑）。
        # 监督层按阶段预算下发 land_fast 后，驱动原来自顾自继续跑阶段机（还会 SCAN→ROTVERIFY 死循环），
        # 于是"飞机早落地了、终端还一直 running"。这里一旦发现从"已解锁"变成"上锁"就收尾。
        if getattr(self, '_ever_armed', False) and not self.armed and self.phase != 'DONE':
            self.get_logger().error(
                f'检测到飞机已上锁（当前阶段 {self.phase}）——可能被监督层/外力 land_fast → '
                f'驱动收工退出（不再发指令）')
            self.finish()
            return
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
                # 固定起始世界位姿（服务器侧 ②）：起飞后先用位置控制飞到目标世界位姿，
                # 再进入常规流程。局部目标 = 当前局部坐标 + (世界目标 − 世界当前)。
                if self.start_pose_enabled and not self._start_pose_done:
                    self._start_pose_done = True
                    target = self._local_target_for_world_pose()
                    if target is not None:
                        self.get_logger().info(
                            f'固定起始位姿：先飞到世界 (北 {self._start_world[0]:+.1f}, '
                            f'东 {self._start_world[1]:+.1f}) → 局部 ENU '
                            f'({target[0]:+.1f}, {target[1]:+.1f})')
                        self._points = [target]
                        self._point_index = 0
                        self._after_start_pose = True
                        self.enter('TRANSIT')
                        return
                    self.get_logger().warn('固定起始位姿：拿不到 AirSim 世界位姿 → 跳过')
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
                # **高度门**（服务器侧 2026-10-17）：换算到世界 up 的顶部/底部高度。
                # `verticals` 是机体 FLU 的 up（相对相机/机体），加当前 ENU 高度即世界 z。
                _pos = self.position_enu()
                flight_up = _pos[2] if _pos is not None else self.altitude
                top_z = float(flight_up + verticals.max())
                bottom_z = float(flight_up + verticals.min())
                tall_enough = top_z >= flight_up + self._seek_top_margin
                # 质量门限（点数/垂跨度/MAD）+ 中位距离下限：只有"够大、够平、够远"的
                # 扇区才算合格候选（服务器侧 2026-10-14 ①：最近的**合格大结构**）。
                qualifies = (count >= self._seek_min_points
                             and span >= self._seek_min_span
                             and mad <= self._seek_max_mad
                             and median >= self._seek_min_median
                             and tall_enough)
                fails = []
                if count < self._seek_min_points:
                    fails.append(f'点<{self._seek_min_points}')
                if span < self._seek_min_span:
                    fails.append(f'跨度<{self._seek_min_span}°')
                if mad > self._seek_max_mad:
                    fails.append(f'MAD>{self._seek_max_mad}')
                if median < self._seek_min_median:
                    fails.append(f'中位<{self._seek_min_median}m')
                if not tall_enough:
                    fails.append(f'低矮可飞越(top{top_z:.1f}<{flight_up + self._seek_top_margin:.1f})')
                candidates.append({'sector': int(sector), 'median': median, 'mad': mad,
                                   'span': span, 'count': count, 'ok': qualifies,
                                   'top_z': top_z, 'bottom_z': bottom_z,
                                   'fail': '/'.join(fails) or '—'})
            good = sorted((c for c in candidates if c['ok']), key=lambda c: c['median'])
            # 打印**全部扇区**（不只是前几个）：便于判断"是门限太严，还是场景真没有结构"
            # （服务器侧 2026-10-14 第五节建议）。
            summary = [(c['sector'], round(c['median'], 1), c['count'],
                        round(c['span'], 1), round(c['mad'], 2),
                        round(c['top_z'], 1), round(c['bottom_z'], 1),
                        'OK' if c['ok'] else c['fail']) for c in
                       sorted(candidates, key=lambda c: c['median'])]
            self.get_logger().info(
                f'深度寻障统计（扇区, 中位 m, 点数, 垂跨度°, MAD m, 顶部z, 底部z, 判定）'
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

        if self.phase == 'TRANSIT':
            # ⚠️ 转场用**速度控制**（2026-10-15 改）：位置目标点会被 PX4 按自身巡航速度执行
            # （"高速撞障碍"的根因）；而"每周期推一点点位置"实测只有 ~0.11 m/s（98 s 都到不了
            # 20 m 外的点，被监督层 TRANSIT 预算判超时）。速度控制能**精确限幅**。
            altitude = self.transit_altitude
            target_east, target_north = self._current_point()
            current = self.position_enu()
            if current is None:
                self.record_approach()
                return
            dx, dy = target_east - current[0], target_north - current[1]
            remaining = math.hypot(dx, dy)
            bearing = math.atan2(dy, dx) if remaining > 1e-3 else 0.0
            heading = (ned_yaw_to_enu_yaw(self.heading_rad)
                       if not math.isnan(self.heading_rad) else bearing)
            err = math.remainder(bearing - heading, 2.0 * math.pi)
            now = time.monotonic()
            forward_clear = self.clearance[0] if self.clearance else None
            suspicious = (forward_clear is not None and self._map_floor_m is not None
                          and forward_clear <= self._map_floor_m + self._echo_band)
            ignored = suspicious and now < self._echo_ignore_until
            blocked = (forward_clear is not None
                       and forward_clear <= self.transit_stop_clearance
                       and remaining > 0.5 and not ignored)
            if self._echo_probe is not None:
                self._echo_probe_tick(now, current, altitude)
                self.record_approach()
                return
            if blocked and suspicious:
                # 有界：单次转场最多判别 N 次、累计后退有上限（服务器侧 2026-10-15 要求）
                if (self._echo_probes_used >= self._echo_max_probes
                        or self._echo_back_total + self._echo_back_m > self._echo_max_back_total + 1e-6):
                    self.get_logger().warn(
                        f'回波判别已用尽（{self._echo_probes_used} 次 / 累计后退 '
                        f'{self._echo_back_total:.2f} m）→ 按真障碍处理，不再后退',
                        throttle_duration_sec=3.0)
                    self.publish_velocity_enu(0.0, 0.0, 0.0, 0.0)
                    self.record_approach()
                    if self.elapsed() > self.transit_timeout:
                        self._give_up_candidate('转场受阻且回波判别用尽')
                    return
                # 疑似自机体回波（读数贴着地板）→ 后退一小段判别
                back = (current[0] - self._echo_back_m * math.cos(heading),
                        current[1] - self._echo_back_m * math.sin(heading))
                self._echo_probe = {'phase': 'back', 't0': now,
                                    'start': (current[0], current[1]),
                                    'target': back, 'delta0': self._current_body_delta()}
                self._echo_probes_used += 1
                self._echo_back_total += self._echo_back_m
                self.publish_velocity_enu(0.0, 0.0, 0.0, 0.0)
                self.get_logger().warn(
                    f'转场净空 {forward_clear:.2f} m 贴着地板 {self._map_floor_m:.2f} m'
                    f'（距目标 {remaining:.1f} m）→ 疑似自机体回波：后退 '
                    f'{self._echo_back_m:.2f} m 判别（真障碍距离会变大、回波不变）')
                self.record_approach()
                return
            forward = 0.0 if blocked else self.transit_speed * max(math.cos(err), 0.0)
            yaw_rate = max(-self.transit_yaw_rate,
                           min(self.transit_yaw_rate, 1.2 * err))
            if blocked:
                self.get_logger().warn(
                    f'转场前方净空 {forward_clear:.2f} m ≤ {self.transit_stop_clearance:.1f} m '
                    f'（距目标还有 {remaining:.1f} m）→ **就地停住**，不再盲飞',
                    throttle_duration_sec=3.0)
            vz = max(-0.6, min(0.6, 0.6 * (altitude - current[2])))
            self.publish_velocity_enu(forward * math.cos(bearing),
                                      forward * math.sin(bearing), vz, yaw_rate)
            self.record_approach()
            if remaining <= 0.5 and abs(current[2] - altitude) < 0.4:
                self.get_logger().info(
                    f'转场到位（ENU 东 {current[0]:+.1f} 北 {current[1]:+.1f} '
                    f'上 {current[2]:.1f}），降到扫描高度 {self.altitude:.1f} m')
                self.enter('DESCEND')
                return
            if self.elapsed() > self.transit_timeout:
                # 服务器侧 2026-10-15：**转场受阻不做复杂绕行**，直接"标记该候选不可达 → 转下一个"
                self._give_up_candidate(
                    f'TRANSIT 超时 {self.transit_timeout:.0f} s（受阻，当前 {current}）')
            return

        if self.phase == 'DESCEND':
            # 纯降高 + 小幅水平修正（也用速度模式，避免与转场的模式来回切换）
            altitude = self.altitude
            target_east, target_north = self._current_point()
            current = self.position_enu()
            if current is None:
                self.record_approach()
                return
            dx, dy = target_east - current[0], target_north - current[1]
            vz = max(-0.6, min(0.6, 0.6 * (altitude - current[2])))
            hx = max(-0.3, min(0.3, 0.4 * dx))
            hy = max(-0.3, min(0.3, 0.4 * dy))
            self.publish_velocity_enu(hx, hy, vz, 0.0)
            self.record_approach()
            close = (math.hypot(dx, dy) < 0.5 and abs(current[2] - altitude) < 0.4)
            if close:
                # 固定起始位姿那一程结束后回到**深度自动寻障**（否则会跳过 SEEK → d14 空跑一轮）
                if self._after_start_pose:
                    self._after_start_pose = False
                    # 此刻飞机在世��原点（start_pose 的目标就是世界 0,0）→ 记下世界原点对应的局部 ENU，
                    # 供"已知结构点位（世界坐标）"换算（局部 = 世界 + 该偏移）
                    self._origin_local = (current[0], current[1])
                    if self.seek_enabled:
                        self._seek_samples = {}
                        self.get_logger().info(
                            f'固定起始位姿到位（ENU 东 {current[0]:+.1f} 北 '
                            f'{current[1]:+.1f} 上 {current[2]:.1f}）→ 开始深度自动寻障')
                        self.enter('SEEK')
                        return
                    # 还原"已知点位"（起始位姿那段临时把 self._points 当单点目标用了）
                    self._points = list(self._known_points)
                    if self._points:
                        self._resolve_world_points()
                        target_east, target_north = self._current_point()
                        self.get_logger().info(
                            f'已知结构点位模式：转到第 1 个点（东 {target_east:+.1f} 北 {target_north:+.1f}）')
                        self.enter('TRANSIT')
                        return
                self.get_logger().info(f'已降到 {self.altitude:.1f} m，开始扫描/摆位')
                self.enter('SCAN' if self.pre_scan else 'APPROACH')
            elif self.elapsed() > self.transit_timeout:
                self.get_logger().warn(
                    f'DESCEND 超时（{self.transit_timeout:.0f} s，当前 {current}）→ 就地继续')
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
                # 交权点落盘（服务器侧 2026-10-16：有/无 episode 的轮次都要记，便于归因命中率）
                _enu = self.position_enu()
                _heading = (math.degrees(ned_yaw_to_enu_yaw(self.heading_rad))
                            if not math.isnan(self.heading_rad) else None)
                self.get_logger().info(
                    f'交权点记录：ENU 东 {_enu[0]:+.1f} 北 {_enu[1]:+.1f} 上 {_enu[2]:+.1f}，'
                    f'航向(ENU) {_heading if _heading is None else round(_heading, 1)}°，'
                    f'前向净空 {round(self._clearance_history[-1], 2)} m，'
                    f'左 {None if not self.clearance else round(self.clearance[1], 2)} '
                    f'右 {None if not self.clearance else round(self.clearance[2], 2)}')
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
            # 诊断（2026-10-17）：记录本次扫描的**最小前向净空 + 对应航向**与最小侧向净空——
            # 用于回答"到了点位却找不到候选"是距离太远、还是朝向/侧向门限的问题。
            if forward_clear is not None and forward_clear < 500.0:
                heading_now = (math.degrees(ned_yaw_to_enu_yaw(self.heading_rad))
                               if not math.isnan(self.heading_rad) else None)
                if self._scan_min_forward is None or forward_clear < self._scan_min_forward:
                    self._scan_min_forward = forward_clear
                    self._scan_min_heading = heading_now
                    self._scan_min_sides = (self.clearance[1], self.clearance[2])
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
            # 用"本点扫描起点"而不是 phase 开始时刻：复验失败返回 SCAN 时不重置（防死循环）
            if time.monotonic() - self._scan_started_at > self.scan_max_s:
                self.publish_yaw_rate(0.0)
                self.get_logger().info(
                    f'扫描诊断：本点最小前向净空 '
                    f'{None if self._scan_min_forward is None else round(self._scan_min_forward, 2)} m'
                    f'（航向(ENU) {None if self._scan_min_heading is None else round(self._scan_min_heading, 1)}°，'
                    f'左/右 {None if not self._scan_min_sides else tuple(round(v, 2) for v in self._scan_min_sides)}）；'
                    f'门限 scan_found={self.scan_found:.1f} m、侧向开阔门限={self.scan_side_open:.1f} m')
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
