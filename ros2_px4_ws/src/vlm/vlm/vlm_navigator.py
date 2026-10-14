"""VLM 导航节点：图像 → 指向点 → 机体速度 → （经接口层）PX4。

安全边界：

- 本节点**不直接**接触 ``/fmu/*``，只向 offboard_bridge 发布 ENU 速度指令；
- 只有 PX4 已解锁、处于 OFFBOARD、且未进入 failsafe 时才发布指令；
- VLM 调用失败或超时即停止发布，桥接节点会在速度超时后回到位置保持；
- 所有指令都经过 ``pointing_to_body_velocity`` 的速度/偏航限幅。
"""

import json
import math
import os
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from px4_interface.frames import body_flu_to_enu, ned_to_enu, ned_yaw_to_enu_yaw
from px4_interface.state import Px4StateMonitor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .pointing import (
    BodyVelocity,
    PointingGeometry,
    PointingParseError,
    parse_pointing,
    parse_waypoints,
    pointing_to_body_velocity,
    waypoint_to_body_velocity,
)
from .safety import (
    MapSafetyLimits,
    ObstacleTrack,
    apply_map_safety,
    parse_clearance_status,
)
from .vlm_client import VlmError, create_client, image_file_to_jpeg, image_message_to_jpeg

# 提示词为本项目自写（接口约定与本地参考实现 SeePointFly 一致，但文字不复制其原文；
# SeePointFly 为 Proprietary 许可，见 docs/reference-see-point-fly.md）。
PROMPT_TEMPLATE = """你是一架无人机的视觉导航模块，需要根据机载相机图像决定下一个前往位置。

任务描述：{instruction}

请完成两件事：
1. 在图中找出所有符合该任务描述的目标物体；
2. 从中选出最值得前往的一个，并在该物体上标出一个点（尽量落在物体中心）。

只输出 JSON 数组，不要输出任何解释文字：
[{{"point": [y, x], "depth": depth_value, "label": "简短说明"}}]

取值约定：
- x：横向位置，0-1000，500 为画面中心，大于 500 表示偏右；
- y：纵向位置，0-1000，数值越小表示越靠画面上方；
- depth：目测距离等级 1-10，1 表示非常近（物体占画面很大），10 表示很远（物体很小）；
- label：一句话说明你指向的物体。

如果图中没有符合描述的目标，输出空数组 []。"""


class VlmNavigator(Node):
    def __init__(self) -> None:
        super().__init__('vlm_navigator')

        self.declare_parameter('instruction', 'fly toward the center of the open area')
        self.declare_parameter('provider', 'mock')
        self.declare_parameter('model', '')
        self.declare_parameter('api_key_env', '')
        self.declare_parameter('image_topic', '/sensor_bridge/camera/image')
        self.declare_parameter('image_file', '')
        self.declare_parameter('velocity_topic', '/offboard_bridge/velocity_setpoint')
        self.declare_parameter('command_rate_hz', 5.0)
        self.declare_parameter('vlm_rate_hz', 0.5)
        self.declare_parameter('hfov_deg', 90.0)
        self.declare_parameter('base_velocity', 1.0)
        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('max_yaw_rate', 0.6)
        self.declare_parameter('yaw_gain', 1.0)
        self.declare_parameter('yaw_deadband_deg', 10.0)
        # 视觉层的兜底阈值（宽松）：真正的“接近终止”策略在 mission 层决定
        self.declare_parameter('stop_distance_m', 0.8)
        self.declare_parameter('decision_topic', '~/decision')
        self.declare_parameter('min_command_duration_s', 1.0)
        self.declare_parameter('command_timeout_s', 3.0)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('output_dir', 'vlm_decisions')
        self.declare_parameter('record_decisions', True)
        self.declare_parameter('depth_topic', '/airsim_camera/depth')
        self.declare_parameter('use_local_depth', True)
        self.declare_parameter('depth_window_px', 3)
        self.declare_parameter('record_frames', True)
        self.declare_parameter('remote_url', 'http://127.0.0.1:18080/navigate')
        self.declare_parameter('session_id', 'drone1')
        self.declare_parameter('remote_timeout_s', 3.0)
        # 竖直分量默认不交给视觉：apos 是"可通行落点"，反投影会算出下降速度（见 pointing 文档）
        self.declare_parameter('vertical_mode', 'hold')
        self.declare_parameter('max_vertical_mps', 0.4)
        self.declare_parameter('use_waypoints', True)
        self.declare_parameter('waypoint_forward_norm_m', 0.375)
        self.declare_parameter('waypoint_lateral_norm_m', 0.25)
        self.declare_parameter('waypoint_yaw_norm_deg', 9.0)
        self.declare_parameter('max_lateral_speed', 1.0)
        # 冷启动前若干帧远端只会给 rot_left（时序策略需要历史），此期间只记录不下发运动。
        # 服务器侧建议 5–8 帧（我们探针实测第 4 帧才转 point），故默认 6。
        self.declare_parameter('warmup_frames', 6)
        self.declare_parameter('instruction_topic', '~/instruction')
        # 运行中"解除 dry_run"：让节点提前启动（只记录），交权瞬间再实飞。
        # 背景（2026-10-13）：摆位脚本要先找到障碍再交权，但节点启动要 ~30 s，
        # 这段时间障碍读数已随地图衰减消失；提前启动可消除这个时间差。
        self.declare_parameter('enable_topic', '~/enable')
        # 安全包络（C1/C2 小步闭环用）：分量缩放 + 运行时长上限 + 距起点围栏
        self.declare_parameter('forward_scale', 1.0)
        self.declare_parameter('lateral_scale', 1.0)
        self.declare_parameter('run_duration_s', 0.0)          # 0 = 不限制
        self.declare_parameter('geofence_max_distance_m', 0.0)  # 0 = 不限制
        # 机载看门狗（服务器侧 2026-10-12 明确：实时安全只能由机载负责，服务端不可当急停）
        self.declare_parameter('max_consecutive_errors', 3)     # 连续失败到这数就停止下发运动
        self.declare_parameter('hold_on_slow_response_ms', 800.0)  # 0 = 不判慢
        # C3 地图安全层（详见 vlm/safety.py 的说明）
        self.declare_parameter('map_status_topic', '/local_map/status')
        self.declare_parameter('avoidance_enabled', True)
        # 地图约 2.25 Hz（周期 0.44 s）：阈值取 1.5 s（≈连丢 3 帧）才不会被正常抖动误判陈旧
        self.declare_parameter('map_stale_s', 1.5)
        self.declare_parameter('stale_forward_limit_mps', 0.1)
        self.declare_parameter('obstacle_slow_start_m', 1.2)
        self.declare_parameter('obstacle_stop_m', 0.6)
        self.declare_parameter('obstacle_release_m', 0.9)
        self.declare_parameter('obstacle_avoid_yaw_rate', 0.15)
        # 保持超时后"后撤退开"（服务器侧"可闭合"原则的延伸）：
        # 若持续处于 obstacle 保持超过 hold_max_s 仍未释放，说明这是个"绕不开"的点
        # （常见于两侧都紧的大墙面），此时慢速后退 + 朝宽侧偏航，让障碍重新进入视野/脱困，
        # 而不是一直贴到窗口结束（2026-10-14 t8 实测保持 208 帧、约 90 s）。
        self.declare_parameter('hold_max_s', 20.0)
        self.declare_parameter('escape_back_mps', 0.10)
        # 脱困也要有界（服务器侧 2026-10-14 第 ④ 条）：避免从"保持卡死"变成"后退卡死"；
        # 到达上限后结束脱困、恢复常规判断；同一段障碍里最多再重试 escape_max_retries 次，
        # 之后放弃脱困，交由驱动阶段超时 / max_flight_s / 监督层兜底落地。
        # 上限采用**预判式**（服务器侧 2026-10-14 选 B）：判断"本周期再加一段后退是否会越界"，
        # 会越界就本帧不再发后退——使边界与速度、控制周期无关（改 escape_back_mps 也不会越界）。
        self.declare_parameter('escape_max_s', 10.0)
        self.declare_parameter('escape_max_m', 1.0)
        self.declare_parameter('escape_max_retries', 1)
        self.declare_parameter('obstacle_side_release_m', 1.2)
        self.declare_parameter('obstacle_max_turn_deg', 60.0)
        self.declare_parameter('obstacle_creep_mps', 0.1)
        # 位姿质量门（服务器侧 2026-10-14 第三节）：世界系一致性判据只有在位姿可靠时才有意义。
        # 位置陈旧 / 角速度过大 → **暂缓判定**（既不算真障碍，也不判拖影），等姿态稳定再判；
        # 否则一次"快速偏航下的位姿滞后"会被误判成拖影（t14 的形态 B 很可能就是这种）。
        self.declare_parameter('pose_quality_gate', True)
        self.declare_parameter('pose_age_max_s', 0.5)
        self.declare_parameter('pose_max_yaw_rate', 0.6)   # rad/s
        self.declare_parameter('obstacle_creep_open_side_m', 1.5)
        self.declare_parameter('obstacle_stop_confirm_frames', 3)
        self.declare_parameter('obstacle_world_tolerance_m', 0.4)
        self.declare_parameter('obstacle_candidate_max_m', 4.0)
        self.declare_parameter('obstacle_track_size', 3)
        # 交权后锁偏航若干秒（外部锁优先于安全层避让偏航，用于逼出"保持"状态做验证）
        self.declare_parameter('yaw_lock_s', 0.0)

        self._instruction = str(self.get_parameter('instruction').value)
        self._image_file = str(self.get_parameter('image_file').value)
        self._dry_run = bool(self.get_parameter('dry_run').value)
        self._max_speed = float(self.get_parameter('max_speed').value)
        self._max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self._base_velocity = float(self.get_parameter('base_velocity').value)
        self._yaw_gain = float(self.get_parameter('yaw_gain').value)
        self._yaw_deadband_rad = math.radians(
            float(self.get_parameter('yaw_deadband_deg').value))
        self._stop_distance = float(self.get_parameter('stop_distance_m').value)
        self._min_duration = float(self.get_parameter('min_command_duration_s').value)
        self._command_timeout = float(self.get_parameter('command_timeout_s').value)
        self._record = bool(self.get_parameter('record_decisions').value)
        self._use_local_depth = bool(self.get_parameter('use_local_depth').value)
        self._depth_window = int(self.get_parameter('depth_window_px').value)
        self._record_frames = bool(self.get_parameter('record_frames').value)
        self._vertical_mode = str(self.get_parameter('vertical_mode').value)
        self._max_vertical_mps = float(self.get_parameter('max_vertical_mps').value)
        self._use_waypoints = bool(self.get_parameter('use_waypoints').value)
        self._waypoint_forward_norm_m = float(
            self.get_parameter('waypoint_forward_norm_m').value)
        self._waypoint_lateral_norm_m = float(
            self.get_parameter('waypoint_lateral_norm_m').value)
        self._waypoint_yaw_norm_rad = math.radians(
            float(self.get_parameter('waypoint_yaw_norm_deg').value))
        self._max_lateral_speed = float(self.get_parameter('max_lateral_speed').value)
        self._warmup_frames = int(self.get_parameter('warmup_frames').value)
        self._forward_scale = float(self.get_parameter('forward_scale').value)
        self._lateral_scale = float(self.get_parameter('lateral_scale').value)
        self._run_duration = float(self.get_parameter('run_duration_s').value)
        self._geofence = float(self.get_parameter('geofence_max_distance_m').value)
        self._max_errors = int(self.get_parameter('max_consecutive_errors').value)
        self._slow_ms = float(self.get_parameter('hold_on_slow_response_ms').value)
        self._map_limits = MapSafetyLimits(
            enabled=bool(self.get_parameter('avoidance_enabled').value),
            stale_s=float(self.get_parameter('map_stale_s').value),
            stale_forward_limit_mps=float(
                self.get_parameter('stale_forward_limit_mps').value),
            slow_start_m=float(self.get_parameter('obstacle_slow_start_m').value),
            stop_m=float(self.get_parameter('obstacle_stop_m').value),
            release_m=float(self.get_parameter('obstacle_release_m').value),
            avoid_yaw_rate=float(self.get_parameter('obstacle_avoid_yaw_rate').value),
            side_release_m=float(self.get_parameter('obstacle_side_release_m').value),
            max_turn_deg=float(self.get_parameter('obstacle_max_turn_deg').value),
            creep_mps=float(self.get_parameter('obstacle_creep_mps').value),
            creep_open_side_m=float(
                self.get_parameter('obstacle_creep_open_side_m').value))
        self._stop_confirm = int(
            self.get_parameter('obstacle_stop_confirm_frames').value)
        self._world_tolerance = float(
            self.get_parameter('obstacle_world_tolerance_m').value)
        self._candidate_max = float(
            self.get_parameter('obstacle_candidate_max_m').value)
        self._track = ObstacleTrack(
            size=int(self.get_parameter('obstacle_track_size').value),
            tolerance_m=self._world_tolerance)
        self._yaw_lock_s = float(self.get_parameter('yaw_lock_s').value)
        self._hold_max_s = float(self.get_parameter('hold_max_s').value)
        self._escape_back_mps = float(self.get_parameter('escape_back_mps').value)
        self._escape_max_s = float(self.get_parameter('escape_max_s').value)
        self._escape_max_m = float(self.get_parameter('escape_max_m').value)
        self._escape_max_retries = max(
            int(self.get_parameter('escape_max_retries').value), 0)
        self._pose_gate = bool(self.get_parameter('pose_quality_gate').value)
        self._pose_age_max = float(self.get_parameter('pose_age_max_s').value)
        self._pose_max_yaw = float(self.get_parameter('pose_max_yaw_rate').value)
        self._output_dir = str(self.get_parameter('output_dir').value)
        vlm_rate = max(float(self.get_parameter('vlm_rate_hz').value), 0.05)
        command_rate = max(float(self.get_parameter('command_rate_hz').value), 1.0)

        self._geometry = PointingGeometry(1, 1, float(self.get_parameter('hfov_deg').value))
        self._client = create_client(
            str(self.get_parameter('provider').value),
            str(self.get_parameter('model').value),
            str(self.get_parameter('api_key_env').value),
            timeout_s=float(self.get_parameter('remote_timeout_s').value),
            instruction=self._instruction,
            session_id=str(self.get_parameter('session_id').value))
        if hasattr(self._client, 'url'):
            self._client.url = str(self.get_parameter('remote_url').value)

        self._velocity_pub = self.create_publisher(
            Twist, str(self.get_parameter('velocity_topic').value), 10)
        self._decision_pub = self.create_publisher(
            String, str(self.get_parameter('decision_topic').value), 10)
        self._monitor = Px4StateMonitor(self)

        self._lock = threading.Lock()
        self._latest_jpeg = None
        self._latest_image_size = None
        self._latest_depth = None       # (depth 数组, 宽, 高)
        self._pending_command = None      # (BodyVelocity, Pointing, timestamp)
        self._vlm_busy = False
        self._last_error = ''
        self._frame_seq = 0               # 已发给远端的帧序号（服务端用它检测重启）
        self._last_command_source = ''    # 本周期指令来自哪一路（指向/路点/状态动作）
        self._last_waypoints = []
        self._guard_started_at = None
        self._guard_origin = None
        self._guard_expired = False
        self._last_guard = 'idle'
        self._error_streak = 0
        self._slow_warned = False
        self._clearance = None          # (前, 左, 右) 米
        self._clearance_raw = ''        # 地图状态原文（含最近点相对坐标）
        self._clearance_stamp = None    # 收到时刻（monotonic）
        self._map_holding = False       # 障碍保持的迟滞状态
        self._obstacle_entry_heading = None   # 进入避障时的航向（用于累积转角）
        self._obstacle_turn_deg = 0.0
        self._obstacle_world = None      # 世界系(ENU)候选障碍点（用于拖影判别）
        self._hold_started_at = None     # 连续处于 obstacle 保持的起点（用于保持超时后撤）
        self._last_heading = None        # 上一帧航向（用于估计角速度，喂位姿质量门）
        self._last_heading_at = 0.0
        self._pose_defer_state = {'pose_age_s': None, 'yaw_rate': None}
        self._escape_started_at = None   # 本次脱困的起点（用于脱困时长/距离上限）
        self._escape_count = 0           # 本段障碍内已脱困次数（用于有限重试）
        self._escape_gave_up = False     # 脱困重试用尽后置位，不再后退
        self._world_consistent = True    # 必须是实例属性：未解锁/慢响应路径也会读它（今天踩过）
        self._obstacle_warned = False
        self._last_guard_detail = {}
        self._pre_filter_command = None
        self._stop_streak = 0            # 连续低于 stop_m 的帧数（进入保持的时间维确认）
        self._running = True

        if self._record:
            os.makedirs(self._output_dir, exist_ok=True)

        if self._image_file:
            self._load_image_file()
        else:
            topic = str(self.get_parameter('image_topic').value)
            self.create_subscription(Image, topic, self._on_image, 1)
            self.get_logger().info(f'等待图像话题：{topic}')
            if self._use_local_depth:
                depth_topic = str(self.get_parameter('depth_topic').value)
                self.create_subscription(Image, depth_topic, self._on_depth, 1)
                self.get_logger().info(
                    f'使用本机深度（{depth_topic}）在指向像素处采样，替代远端占位深度')

        # 运行中换指令：**不能重启节点**，否则 frame_seq 归零会被服务端判为新 episode
        # 并清空历史（服务器侧 2026-10-12 明确的硬性要求）。
        self.create_subscription(
            String, str(self.get_parameter('instruction_topic').value),
            self._on_instruction, 10)
        self.create_subscription(
            String, str(self.get_parameter('enable_topic').value),
            self._on_enable, 10)
        # C3：局部地图三向净空（fail-safe 安全层的输入）
        self.create_subscription(
            String, str(self.get_parameter('map_status_topic').value),
            self._on_map_status, 10)

        self._worker = threading.Thread(target=self._vlm_loop, daemon=True)
        self._worker.start()
        self.create_timer(1.0 / command_rate, self._publish_loop)

        self.get_logger().info(
            f'vlm_navigator 启动：provider={self._client.__class__.__name__}，'
            f'指令「{self._instruction}」，速度上限 {self._max_speed:.2f} m/s，'
            f'偏航上限 {self._max_yaw_rate:.2f} rad/s，dry_run={self._dry_run}')

    # ------------------------------------------------------------------
    # 图像来源
    # ------------------------------------------------------------------

    def _load_image_file(self) -> None:
        from PIL import Image as PilImage

        with PilImage.open(self._image_file) as image:
            width, height = image.size
        jpeg = image_file_to_jpeg(self._image_file)
        with self._lock:
            self._latest_jpeg = jpeg
            self._latest_image_size = (width, height)
        self._geometry = PointingGeometry(
            width, height, self._geometry.hfov_deg)
        self.get_logger().info(
            f'离线模式：使用静态图像 {self._image_file}（{width}x{height}）')

    def _on_image(self, msg: Image) -> None:
        try:
            jpeg = image_message_to_jpeg(msg)
        except (VlmError, ImportError) as exc:
            self.get_logger().warn(f'图像编码失败：{exc}', throttle_duration_sec=10.0)
            return
        with self._lock:
            self._latest_jpeg = jpeg
            self._latest_image_size = (int(msg.width), int(msg.height))

    def _on_depth(self, msg: Image) -> None:
        """缓存最新的 32FC1 深度图，用于在指向像素处采样本机深度。"""
        if msg.encoding != '32FC1':
            return
        import numpy as np

        depth = np.frombuffer(msg.data, dtype=np.float32)
        if depth.size < int(msg.width) * int(msg.height):
            return
        with self._lock:
            self._latest_depth = (depth.reshape(int(msg.height), int(msg.width)),
                                  int(msg.width), int(msg.height))

    def _on_instruction(self, msg: String) -> None:
        """运行中切换指令（同一会话、frame_seq 继续递增；服务端 instruction 可逐帧改）。"""
        text = str(msg.data).strip()
        if not text or text == self._instruction:
            return
        self._instruction = text
        if hasattr(self._client, 'instruction'):
            self._client.instruction = text
        self.get_logger().info(f'指令已切换为「{text}」（同一会话，frame_seq 继续递增）')

    def _on_enable(self, msg: String) -> None:
        """解除 dry_run：交权瞬间由摆位脚本触发，节点开始真正下发速度。"""
        token = str(msg.data).strip().lower()
        if token in ('1', 'true', 'enable', 'go', 'live'):
            if self._dry_run:
                self._dry_run = False
                # **原子**重置：以"enable 这一刻"的位姿作为围栏原点与包络起点，
                # 不留"下一帧再建"的窗口（否则首个 post-enable 帧会按旧原点判定——
                # 2026-10-14 trial4 的 geofence 就是这样出现的）
                now = time.monotonic()
                state = self._monitor.snapshot()
                origin = ((state.x, state.y) if self._monitor.position_fresh() else None)
                with self._lock:
                    self._guard_started_at = now
                    self._guard_origin = origin
                self._guard_expired = False
                origin_text = ('未知' if origin is None
                               else f'({origin[0]:.2f}, {origin[1]:.2f}) NED')
                self.get_logger().warn(
                    f'收到 enable：解除 dry_run；围栏原点**原子重置**为当前位姿 {origin_text}，'
                    '安全包络重新计时')
        elif token in ('0', 'false', 'disable', 'hold'):
            if not self._dry_run:
                self._dry_run = True
                self.get_logger().warn('收到 disable：回到 dry_run（只记录不执行）')

    def _on_map_status(self, msg: String) -> None:
        """缓存局部地图的三向净空（用于 C3 安全层；解析失败按"未知"处理）。"""
        parsed = parse_clearance_status(msg.data)
        with self._lock:
            self._clearance = parsed
            self._clearance_stamp = time.monotonic()
            self._clearance_raw = msg.data        # 原始状态串（含最近点坐标，便于归因）

    def _depth_at(self, x_norm: float, y_norm: float):
        """在归一化像素处取本机深度（小窗口中位数，抗噪）。"""
        import math as _math

        import numpy as np

        if _math.isnan(x_norm) or _math.isnan(y_norm):
            return None
        with self._lock:
            entry = self._latest_depth
        if entry is None:
            return None
        depth, width, height = entry
        px = int(round(x_norm * (width - 1)))
        py = int(round(y_norm * (height - 1)))
        window = max(self._depth_window, 0)
        patch = depth[max(py - window, 0):py + window + 1,
                      max(px - window, 0):px + window + 1]
        valid = patch[np.isfinite(patch) & (patch > 0.05)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    # ------------------------------------------------------------------
    # VLM 工作线程
    # ------------------------------------------------------------------

    def _vlm_loop(self) -> None:
        period = 1.0 / max(float(self.get_parameter('vlm_rate_hz').value), 0.05)
        while self._running:
            started = time.monotonic()
            self._run_single_decision()
            time.sleep(max(0.0, period - (time.monotonic() - started)))

    def _run_single_decision(self) -> None:
        with self._lock:
            jpeg = self._latest_jpeg
            size = self._latest_image_size
        if jpeg is None or size is None:
            return
        if size != (self._geometry.image_width, self._geometry.image_height):
            self._geometry = PointingGeometry(
                size[0], size[1], self._geometry.hfov_deg)

        prompt = PROMPT_TEMPLATE.format(instruction=self._instruction)
        self._frame_seq += 1
        meta = self._link_meta()
        try:
            response = self._client.complete(prompt, jpeg, meta=meta)
            self._error_streak = 0
            pointing = parse_pointing(response)
            waypoints = parse_waypoints(response) if self._use_waypoints else []
            self._last_waypoints = [
                {'forward_m': w.forward_m, 'lateral_m': w.lateral_m, 'yaw_rad': w.yaw_rad}
                for w in waypoints]
            # 指令来源优先级（2026-10-11 与服务器侧确认后定）：
            #   ① stop/not_visible → 保持（显式停止最优先）
            #   ② waypoints 非空 → 用路点：水平速度 + 偏航角速度，**竖直恒为 0**
            #   ③ rot_left/rot_right → 原地偏航
            #   ④ 其余 → 指向反投影（竖直是否交给视觉由 vertical_mode 决定）
            state_action = (pointing.apos_state or '').strip().lower()
            self._last_command_source = ''
            if self._frame_seq <= self._warmup_frames:
                # 时序策略需要历史：实测冷启动前 3–4 帧远端一律给 rot_left
                # （换会话复测同一张帧也会变），此期间只记录、不下发运动。
                command = BodyVelocity(0.0, 0.0, 0.0, 0.0, self._min_duration)
                self._last_command_source = 'warmup'
                self.get_logger().info(
                    f'冷启动预热 {self._frame_seq}/{self._warmup_frames}：只记录不下发运动',
                    throttle_duration_sec=1.0)
            elif state_action in ('stop', 'not_visible'):
                command = BodyVelocity(0.0, 0.0, 0.0, 0.0, self._min_duration)
                self._last_command_source = f'state:{state_action}'
                self.get_logger().info(f'远端状态动作 {state_action} → 保持位置')
            elif waypoints:
                target = waypoints[0]
                command = waypoint_to_body_velocity(
                    target,
                    base_velocity=self._base_velocity,
                    max_speed=self._max_speed,
                    max_yaw_rate=self._max_yaw_rate,
                    yaw_gain=self._yaw_gain,
                    yaw_deadband_rad=self._yaw_deadband_rad,
                    min_duration_s=self._min_duration,
                    forward_norm_m=self._waypoint_forward_norm_m,
                    lateral_norm_m=self._waypoint_lateral_norm_m,
                    yaw_norm_rad=self._waypoint_yaw_norm_rad,
                    max_lateral_speed=self._max_lateral_speed)
                self._last_command_source = f'waypoint[{len(waypoints)}]'
                self.get_logger().info(
                    f'远端路点（缩放前）前 {target.forward_m:+.2f} m / 横 {target.lateral_m:+.2f} m / '
                    f'偏航 {math.degrees(target.yaw_rad):+.1f}° → '
                    f'前 {command.forward:+.2f} 左 {command.left:+.2f} m/s、'
                    f'偏航 {command.yaw_rate:+.2f} rad/s（竖直 0，高度交飞控）',
                    throttle_duration_sec=1.0)
            elif state_action in ('rot_left', 'rot_right'):
                yaw = self._max_yaw_rate if state_action == 'rot_left' else -self._max_yaw_rate
                command = BodyVelocity(0.0, 0.0, 0.0, yaw, self._min_duration)
                self._last_command_source = f'state:{state_action}'
                self.get_logger().info(f'远端状态动作 {state_action} → 原地旋转 {yaw:+.2f} rad/s')
            else:
                # 用本机深度替换远端占位深度，再做几何反投影
                if self._use_local_depth and pointing.has_position:
                    measured = self._depth_at(pointing.x_norm, pointing.y_norm)
                    if measured is not None:
                        self.get_logger().info(
                            f'深度：远端占位 {pointing.depth_m:.2f} m → 本机采样 {measured:.2f} m',
                            throttle_duration_sec=2.0)
                        pointing.depth_m = measured
                self._last_command_source = 'pointing'
                command = pointing_to_body_velocity(
                    pointing, self._geometry,
                    base_velocity=self._base_velocity,
                    max_speed=self._max_speed,
                    max_yaw_rate=self._max_yaw_rate,
                    yaw_gain=self._yaw_gain,
                    yaw_deadband_rad=self._yaw_deadband_rad,
                    stop_distance_m=self._stop_distance,
                    min_duration_s=self._min_duration,
                    vertical_mode=self._vertical_mode,
                    max_vertical_mps=self._max_vertical_mps)
        except (VlmError, PointingParseError, ValueError) as exc:
            self._note_error()
            if str(exc) != self._last_error:
                self.get_logger().warn(f'本周期未产生有效指令：{exc}')
                self._last_error = str(exc)
            return
        except Exception as exc:  # noqa: BLE001
            # 兜底：任何未预期异常都只丢弃本周期，绝不让工作线程退出
            # （否则节点看起来还在跑，实际再也不会产生决策——2026-10-11 联调时踩过）。
            self.get_logger().error(
                f'本周期发生未预期异常（已跳过，线程继续）：{type(exc).__name__}: {exc}',
                throttle_duration_sec=5.0)
            self._note_error()
            return

        self._pre_filter_command = command
        command, self._last_guard, self._last_guard_detail = self._guard_command(command)
        # 日志里的"缩放前"命令与实际下发的命令可能不同（forward_scale/lateral_scale、
        # 时长/围栏/慢响应都会改），这里把**实际下发值**单独打一行，避免误读日志。
        self.get_logger().info(
            f'实际下发：source={self._last_command_source} guard={self._last_guard} '
            f'前 {command.forward:+.2f} 左 {command.left:+.2f} 上 {command.up:+.2f} m/s '
            f'偏航 {command.yaw_rate:+.2f} rad/s',
            throttle_duration_sec=1.0)

        # 下面这段（发布 + 日志 + 落盘）**必须**也在 try 里：
        # 状态动作（rot_left/rot_right/stop/not_visible）没有像素坐标，x_norm/y_norm 是 NaN，
        # 之前在这里做 pixel_from_norm 会抛 ValueError 并把工作线程整条打死
        # （2026-10-11 Step B 重跑时踩到：首帧必是 rot_left，于是永远 0 条记录）。
        try:
            self._last_error = ''
            with self._lock:
                self._pending_command = (command, pointing, time.monotonic(), response)
            # 把指向结果与生成的机体速度发布出去，供任务层做“接近终止”等策略决策
            decision = String()
            decision.data = json.dumps({
            'frame_seq': self._frame_seq,
            'command_source': self._last_command_source,
            'guard': self._last_guard,
            'guard_detail': self._last_guard_detail,
            'clearance': self._clearance,
            'pre_filter': (None if self._pre_filter_command is None else {
                'forward': self._pre_filter_command.forward,
                'left': self._pre_filter_command.left,
                'yaw_rate': self._pre_filter_command.yaw_rate}),
            'post_filter': {
                'forward': command.forward,
                'left': command.left,
                'yaw_rate': command.yaw_rate},
            'label': pointing.label,
            'point_x_norm': pointing.x_norm,
            'point_y_norm': pointing.y_norm,
            'depth_m': pointing.depth_m,
            'apos_state': pointing.apos_state,
            'opos_state': pointing.opos_state,
            'waypoints': self._last_waypoints,
            'body_velocity': {
                'forward': command.forward,
                'left': command.left,
                'up': command.up,
                'yaw_rate': command.yaw_rate,
            },
            'stamp': time.time(),
            }, ensure_ascii=False, allow_nan=True)
            self._decision_pub.publish(decision)
            if pointing.has_position:
                self.get_logger().info(
                    f'VLM 指向：像素 '
                    f'{self._geometry.pixel_from_norm(pointing.x_norm, pointing.y_norm)}'
                    f' 深度 {pointing.depth_m:.2f} m「{pointing.label}」→ '
                    f'机体速度 (前 {command.forward:.2f}, 左 {command.left:.2f}, '
                    f'上 {command.up:.2f}) m/s，偏航 {command.yaw_rate:.2f} rad/s，'
                    f'时长 {command.duration_s:.1f} s')
            else:
                self.get_logger().info(
                    f'VLM 状态动作：{pointing.apos_state or "-"}（无像素坐标）→ '
                    f'机体速度 (前 {command.forward:.2f}, 左 {command.left:.2f}, '
                    f'上 {command.up:.2f}) m/s，偏航 {command.yaw_rate:.2f} rad/s，'
                    f'时长 {command.duration_s:.1f} s')
            if self._record:
                self._record_decision(pointing, command, response, jpeg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f'发布/记录阶段异常（已跳过，线程继续）：{type(exc).__name__}: {exc}',
                throttle_duration_sec=5.0)

    def _link_meta(self) -> dict:
        """接口 v2 的 ``meta``：帧序号 + 高度 + 航向（拿不到就省略该字段）。"""
        meta = {'frame_seq': self._frame_seq}
        state = self._monitor.snapshot()
        if self._monitor.position_fresh() and math.isfinite(state.z):
            meta['altitude_m'] = round(-float(state.z), 2)
        if math.isfinite(state.heading):
            meta['heading_deg'] = round(math.degrees(float(state.heading)) % 360.0, 1)
        return meta

    def _pose_deferred(self, now: float, state) -> bool:
        """位姿质量门：位置陈旧或角速度过大时返回 True（本帧**暂缓判定**）。

        服务器侧 2026-10-14：真障碍在世界系固定，只有**位姿是准的**才成立；
        快速偏航时里程计滞后会让反投影点抖动，从而被误判成"拖影"。所以位姿不可靠时
        既不当真障碍、也不判拖影，等姿态稳定再判。
        """
        if not self._pose_gate:
            return False
        age = self._monitor.position_age_s()
        yaw_rate = 0.0
        if math.isfinite(state.heading):
            if self._last_heading is not None:
                dt = now - self._last_heading_at
                if dt > 1e-3:
                    yaw_rate = math.remainder(
                        float(state.heading) - self._last_heading, 2 * math.pi) / dt
            self._last_heading = float(state.heading)
            self._last_heading_at = now
        self._pose_defer_state = {
            'pose_age_s': None if not math.isfinite(age) else round(age, 2),
            'yaw_rate': round(yaw_rate, 2),
        }
        return age > self._pose_age_max or abs(yaw_rate) > self._pose_max_yaw

    def _nearest_world_point(self, state):
        """把地图状态里的最近点（机体系 ENU 偏移）换算成**世界系 ENU 点**。

        用于服务器侧建议的"世界系一致性"判据：自运动拖影逐帧跳，真障碍稳定。
        拿不到（状态里没有 `nearest_fwd` 或位置无效）时返回 ``None``。
        """
        text = self._clearance_raw or ''
        if 'nearest_fwd=(' not in text:
            return None
        if not self._monitor.position_fresh():
            return None
        try:
            body = text.split('nearest_fwd=(')[1].split(')')[0]
            dx, dy, dz = (float(value) for value in body.split(','))
        except (IndexError, ValueError):
            return None
        east, north, up = ned_to_enu(state.x, state.y, state.z)
        return (east + dx, north + dy, up + dz)

    def _note_error(self) -> None:
        """连续失败计数：到阈值就清空待发指令，让桥接节点回到位置保持。

        服务器侧 2026-10-12 明确：他们的监控**只写日志、不能实时介入**，
        所以"请求失败/超时 → 归零保持"必须由本机负责，这里就是那道闸。
        """
        self._error_streak += 1
        if self._max_errors > 0 and self._error_streak >= self._max_errors:
            with self._lock:
                self._pending_command = None
            self.get_logger().error(
                f'机载看门狗：连续 {self._error_streak} 次未取得有效指令，'
                '已停止下发运动（桥接节点保持位置）',
                throttle_duration_sec=5.0)

    def _guard_command(self, command: BodyVelocity):
        """安全包络：分量缩放 + 运行时长上限 + 距起点围栏（C1/C2 小步闭环用）。

    返回 ``(command, guard_state)``，``guard_state`` 会写进决策记录里便于事后归因：

    - ``idle``：尚未进入"已解锁 + OFFBOARD"，计时与围栏原点都不启动
      （节点可能在地面就跑了很久，不能把那段时间算进飞行时长）；
    - ``ok``：正常；
    - ``duration``：飞行时长达到 ``run_duration_s``，强制归零悬停；
    - ``geofence``：距起点超过 ``geofence_max_distance_m``，强制归零悬停。
        """
        if self._dry_run:
            # 未交权（dry_run）：**不执行**时长/围栏包络——此时由摆位脚本控制飞机，
            # 按"起降点旧原点"判 geofence 会污染记录（服务器侧 2026-10-14 指出）。
            self._last_guard_detail = {'reason': 'dry_run'}
            return command, 'dry_run', {'reason': 'dry_run'}

        command = BodyVelocity(
            command.forward * self._forward_scale,
            command.left * self._lateral_scale,
            command.up,
            command.yaw_rate,
            command.duration_s)

        def _hold():
            return BodyVelocity(0.0, 0.0, 0.0, 0.0, command.duration_s)

        # 慢响应降级：延迟超阈值时**不再前进/偏航**，改为原地保持
        # （服务器侧明确"不要指望服务端实时急停"，这一层必须在本机）。
        latency = getattr(self._client, 'last_latency_ms', None)
        if (self._slow_ms > 0.0 and isinstance(latency, (int, float))
                and latency > self._slow_ms):
            if not self._slow_warned:
                self.get_logger().warn(
                    f'延迟 {latency:.0f} ms 超过阈值 {self._slow_ms:.0f} ms，'
                    '暂停运动并保持（直到延迟恢复）')
                self._slow_warned = True
            return _hold(), 'slow', {'reason': 'slow_response',
                                     'latency_ms': round(float(latency), 1)}
        self._slow_warned = False

        state = self._monitor.snapshot()
        if not (state.armed and state.offboard):
            self._guard_started_at = None
            self._guard_expired = False
            self._last_guard_detail = {}
            return command, 'idle', {}

        now = time.monotonic()

        # ---- 地图安全层（C3）：fail-safe 陈旧处理 + 压速 + 障碍保持（含迟滞）----
        # 世界系一致性：自运动拖影在世界系里逐帧跳变，真障碍稳定（服务器侧 2026-10-13 建议）
        self._world_consistent = True
        world_point = self._nearest_world_point(state)
        close_to_obstacle = (self._clearance is not None
                             and self._clearance[0] < self._map_limits.stop_m)
        # 几何门限：只把"前向锥内且距离 ≤ obstacle_candidate_max_m"的点当候选
        # （服务器侧 2026-10-13：别把远处噪声当候选；容差 0.4 m 只适合近距）
        candidate_ok = (close_to_obstacle
                        and (self._clearance[0] <= self._candidate_max))
        if not candidate_ok:
            self._obstacle_world = None
            self._track.reset()
        elif world_point is not None:
            # 与"最近 N 帧质心"比较，而不是只和上一帧链式比较
            self._world_consistent = self._track.update(world_point)
            self._obstacle_world = world_point
        age = (now - self._clearance_stamp) if self._clearance_stamp else None
        # 累积转角：以"进入避障时的航向"为基准，到上限就停止继续转（防死角打转）
        pending_hold = bool(self._map_holding)
        if not pending_hold and self._clearance is not None:
            pending_hold = self._clearance[0] < self._map_limits.stop_m
        if pending_hold:
            if self._obstacle_entry_heading is None and math.isfinite(state.heading):
                self._obstacle_entry_heading = float(state.heading)
            if (self._obstacle_entry_heading is not None
                    and math.isfinite(state.heading)):
                self._obstacle_turn_deg = math.degrees(math.remainder(
                    float(state.heading) - self._obstacle_entry_heading, 2 * math.pi))
        else:
            self._obstacle_entry_heading = None
            self._obstacle_turn_deg = 0.0
        command, map_guard, self._map_holding, detail = apply_map_safety(
            command, self._clearance, age, self._map_holding, self._map_limits,
            turn_deg=self._obstacle_turn_deg, stop_streak=self._stop_streak,
            world_consistent=self._world_consistent, defer=self._pose_deferred(now, state))
        self._stop_streak = int(detail.get('stop_streak', 0))
        if self._pose_gate:
            # 把位姿质量落进记录，便于事后把"拖影判定"与"位姿滞后/高角速度"对齐分析
            detail = dict(detail)
            detail.update(self._pose_defer_state)
        # 保持超时 → 慢速后撤退开（"绕不开的点"不要贴到窗口结束）。
        # 脱困本身**有界**（时长/距离）且**有限重试**（服务器侧 2026-10-14 第 ④ 条）：
        # 避免从"保持卡死"变成"后退卡死"；重试用尽后交由阶段超时 / max_flight_s / 监督层兜底。
        if map_guard == 'obstacle':
            if self._hold_started_at is None:
                self._hold_started_at = now
            allowed_escapes = self._escape_max_retries + 1
            if (self._escape_started_at is None
                    and self._escape_count < allowed_escapes
                    and now - self._hold_started_at > self._hold_max_s):
                self._escape_started_at = now
                self._escape_count += 1
                self._hold_started_at = now      # 重新计时：脱困结束后才会再次触发
                self.get_logger().warn(
                    f'保持已持续 > {self._hold_max_s:.0f} s 仍未释放 → '
                    f'第 {self._escape_count}/{allowed_escapes} 次脱困'
                    f'（上限 {self._escape_max_s:.0f} s / {self._escape_max_m:.2f} m）',
                    throttle_duration_sec=5.0)
            elif (self._escape_started_at is None
                  and self._escape_count >= allowed_escapes
                  and not self._escape_gave_up):
                self._escape_gave_up = True
                self.get_logger().error(
                    f'脱困重试已达上限（{allowed_escapes} 次）仍无法释放 → 不再后退，'
                    f'交由阶段超时 / max_flight_s / 监督层兜底')
            if self._escape_started_at is not None:
                elapsed = now - self._escape_started_at
                travelled = abs(self._escape_back_mps) * elapsed
                # 预判：把本周期（到下一次指令）的后退也算进去，越界就本帧不再后退
                step = max(float(getattr(command, 'duration_s', 0.0)), 0.0)
                projected_t = elapsed + step
                projected_m = abs(self._escape_back_mps) * projected_t
                if projected_t >= self._escape_max_s or projected_m >= self._escape_max_m:
                    # 已达（或再发一帧就会越过）时长/距离上限 → 结束本轮脱困
                    self._escape_started_at = None
                    self._hold_started_at = now
                    self.get_logger().warn(
                        f'脱困已达上限（已后退 {travelled:.2f} m / {elapsed:.1f} s；'
                        f'上限 {self._escape_max_m:.2f} m / {self._escape_max_s:.0f} s，'
                        f'预判本周期会越界）→ 结束后退，恢复常规保持判断')
                else:
                    side = detail.get('avoid_side') or 'left'
                    yaw = self._map_limits.avoid_yaw_rate * (
                        1.0 if side == 'left' else -1.0)
                    command = BodyVelocity(
                        -abs(self._escape_back_mps), 0.0, 0.0, yaw, command.duration_s)
                    detail = dict(detail)
                    detail['escape'] = True
                    detail['escape_index'] = self._escape_count
                    detail['avoid_side'] = side
                    detail['hold_s'] = round(now - self._hold_started_at, 1)
                    map_guard = 'obstacle_escape'
        else:
            # 障碍解除：复位保持/脱困状态，使下一次新的障碍可以重新脱困
            self._hold_started_at = None
            self._escape_started_at = None
            self._escape_count = 0
            self._escape_gave_up = False
        self._last_guard_detail = detail
        if map_guard == 'obstacle' and not self._obstacle_warned:
            self.get_logger().warn(
                f'地图安全层：前向净空 {detail.get("forward_m")} m 过近，'
                f'归零保持并朝更宽一侧（{detail.get("avoid_side")}）偏航')
            self._obstacle_warned = True
        elif map_guard != 'obstacle':
            self._obstacle_warned = False

        if self._guard_started_at is None:
            self._guard_started_at = now
            self._guard_origin = ((state.x, state.y)
                                  if self._monitor.position_fresh() else None)
            self.get_logger().info(
                f'安全包络起始：时长上限 {self._run_duration:.0f} s、'
                f'围栏 {self._geofence:.1f} m、前向缩放 {self._forward_scale:.2f}、'
                f'横向缩放 {self._lateral_scale:.2f}')

        if self._run_duration > 0.0 and now - self._guard_started_at > self._run_duration:
            if not self._guard_expired:
                self.get_logger().warn(
                    f'安全包络：已达到 {self._run_duration:.0f} s 上限，停止下发运动（保持悬停）')
                self._guard_expired = True
            return _hold(), 'duration', {'reason': 'duration_limit',
                                         'limit_s': self._run_duration}

        if (self._geofence > 0.0 and self._guard_origin is not None
                and self._monitor.position_fresh()):
            drift = math.hypot(state.x - self._guard_origin[0],
                               state.y - self._guard_origin[1])
            if drift > self._geofence:
                self.get_logger().warn(
                    f'安全包络：距起点 {drift:.1f} m 超过围栏 {self._geofence:.1f} m，'
                    '停止下发运动')
                return _hold(), 'geofence', {'reason': 'geofence',
                                             'drift_m': round(drift, 2)}
        # 外部锁偏航优先：用于"逼出保持状态"的验证（否则安全层的避让偏航会立刻把视野转开）
        if self._yaw_lock_s > 0.0 and (now - self._guard_started_at) < self._yaw_lock_s:
            if command.yaw_rate != 0.0:
                command = BodyVelocity(command.forward, command.left, command.up,
                                       0.0, command.duration_s)
            detail = dict(detail)
            detail['yaw_locked_s'] = round(self._yaw_lock_s
                                           - (now - self._guard_started_at), 1)
        return command, map_guard, detail

    # ------------------------------------------------------------------
    # 指令发布
    # ------------------------------------------------------------------

    def _publish_loop(self) -> None:
        with self._lock:
            pending = self._pending_command
        if pending is None:
            return
        command, pointing, stamp, _ = pending
        if time.monotonic() - stamp > self._command_timeout:
            self.get_logger().warn(
                'VLM 指令已过期，停止发布（桥接节点将回到位置保持）',
                throttle_duration_sec=5.0)
            return

        state = self._monitor.snapshot()
        if not (state.armed and state.offboard and not state.failsafe):
            if not self._monitor.status_fresh():
                self.get_logger().warn(
                    '未收到 PX4 状态，暂不发送速度指令', throttle_duration_sec=5.0)
            else:
                self.get_logger().info(
                    f'未处于可控制状态（armed={state.armed}, offboard={state.offboard}, '
                    f'failsafe={state.failsafe}），暂不发送速度指令',
                    throttle_duration_sec=5.0)
            return

        heading_enu = ned_yaw_to_enu_yaw(state.heading)
        east, north, up = body_flu_to_enu(
            command.forward, command.left, command.up, heading_enu)
        twist = Twist()
        twist.linear.x = east
        twist.linear.y = north
        twist.linear.z = up
        twist.angular.z = command.yaw_rate

        if self._dry_run:
            self.get_logger().info(
                f'[dry_run] ENU 速度 ({east:.2f}, {north:.2f}, {up:.2f}) m/s，'
                f'偏航 {command.yaw_rate:.2f} rad/s',
                throttle_duration_sec=1.0)
            return
        self._velocity_pub.publish(twist)

    # ------------------------------------------------------------------
    # 决策记录
    # ------------------------------------------------------------------

    def _record_decision(self, pointing, command, response: str, jpeg: bytes) -> None:
        stamp = time.strftime('%Y%m%d_%H%M%S') + f'_{int(time.time() * 1000) % 1000:03d}'
        state = self._monitor.snapshot()      # 速度/俯仰用于"伪点是否与运动相关"的分析
        record = {
            'timestamp': stamp,
            'frame_seq': self._frame_seq,
            'instruction': self._instruction,
            'provider': self._client.__class__.__name__,
            'latency_ms': getattr(self._client, 'last_latency_ms', None),
            'frame_file': None,
            'image_size': [self._geometry.image_width, self._geometry.image_height],
            'point_pixel': (list(self._geometry.pixel_from_norm(
                pointing.x_norm, pointing.y_norm)) if pointing.has_position else None),
            'point_norm': ([pointing.x_norm, pointing.y_norm]
                           if pointing.has_position else None),
            'depth_m': pointing.depth_m,
            'depth_score': pointing.depth_raw,
            'apos_state': pointing.apos_state,
            'opos_state': pointing.opos_state,
            'command_source': self._last_command_source,
            'guard': self._last_guard,
            'guard_detail': self._last_guard_detail,
            'clearance': self._clearance,
            'map_status': self._clearance_raw,
            'obstacle_world': self._obstacle_world,
            'world_consistent': self._world_consistent,
            'px4_velocity': {'vx': state.vx, 'vy': state.vy, 'vz': state.vz},
            'pitch_deg': (round(math.degrees(state.pitch), 2)
                          if math.isfinite(state.pitch) else None),
            'waypoints': self._last_waypoints,
            # 过滤前 vs 过滤后（服务器侧要求：便于事后归因，也能对时间窗）
            'pre_filter': (None if self._pre_filter_command is None else {
                'forward': self._pre_filter_command.forward,
                'left': self._pre_filter_command.left,
                'up': self._pre_filter_command.up,
                'yaw_rate': self._pre_filter_command.yaw_rate}),
            'post_filter': {
                'forward': command.forward,
                'left': command.left,
                'up': command.up,
                'yaw_rate': command.yaw_rate},
            'label': pointing.label,
            'body_velocity': {
                'forward': command.forward,
                'left': command.left,
                'up': command.up,
                'yaw_rate': command.yaw_rate,
                'duration_s': command.duration_s,
            },
            'raw_response': response,
        }
        path = os.path.join(
            self._output_dir, f'decision_{stamp}_seq{self._frame_seq:05d}.json')
        if self._record_frames:
            frames_dir = os.path.join(self._output_dir, 'frames')
            os.makedirs(frames_dir, exist_ok=True)
            frame_path = os.path.join(
                frames_dir, f'frame_{stamp}_seq{self._frame_seq:05d}.jpg')
            try:
                with open(frame_path, 'wb') as handle:
                    handle.write(jpeg)
                record['frame_file'] = os.path.relpath(frame_path, self._output_dir)
            except OSError as exc:
                self.get_logger().warn(f'帧图像写入失败：{exc}')
        try:
            with open(path, 'w', encoding='utf-8') as handle:
                json.dump(record, handle, ensure_ascii=False, indent=2)
        except OSError as exc:
            self.get_logger().warn(f'决策记录写入失败：{exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VlmNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
