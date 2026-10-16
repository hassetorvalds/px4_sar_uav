"""地图安全层（Step C3）：把本机局部地图的三向净空插进 VLM 命令通道。

设计要点（2026-10-12 与服务器侧确认后定稿）：

1. **fail-safe 而不是 fail-open**：地图数据陈旧（>``stale_s``）时视为"前向净空未知"，
   把前向速度压到 ``stale_forward_limit_mps``，而不是"没有数据就原样飞"；
2. **左右符号与 LightNav 一致**：`local_map` 的 ``left_m`` 是机体左方（航向 +90° 锥），
   与路点 ``lateral_m``（+ = 左）同一套；因此"左更宽 → 左转（yaw_rate > 0）"，
   这里用单元测试把方向钉死；
3. **迟滞**：进入保持用 ``stop_m``、退出门槛用更高的 ``release_m``，避免在阈值附近抖振；
4. **障碍保持时偏航覆盖 VLM**（限幅），前向与横向归零；
5. 压速作用在**最终下发速度**上（不是路点层）：从 ``slow_start_m`` 到 ``stop_m`` 线性压到 0。

模块只依赖 ``math``/``dataclasses``/``re``，便于离线单元测试。
"""

import math
import re
from dataclasses import dataclass

from .pointing import BodyVelocity

_CLEARANCE_RE = re.compile(
    r'nearest_forward_m=(-?[\d.]+|inf|nan)\s+left_m=(-?[\d.]+|inf|nan)'
    r'\s+right_m=(-?[\d.]+|inf|nan)')


@dataclass
class MapSafetyLimits:
    """安全层阈值（全部可在 ROS 参数里覆盖）。"""

    enabled: bool = True
    # 净空数据超过这个时间视为"未知"。**取值必须大于地图周期的 3 倍**：
    # 本机 local_map 跟随相机约 2.25 Hz（周期 0.44 s），若阈值取 0.5 s，
    # 两个自由运行的循环会每隔一帧就判一次"陈旧"，出现 ok/stale 逐帧闪烁
    # （2026-10-12 真实避障飞行实测到）。1.5 s ≈ 允许连丢 3 帧地图。
    stale_s: float = 1.5
    stale_forward_limit_mps: float = 0.1  # 未知时前向速度上限（fail-safe）
    slow_start_m: float = 1.2            # 从该净空开始压速
    stop_m: float = 0.6                  # 低于该净空进入"保持"
    # 进入保持需要**连续 N 次**都低于 stop_m（时间维滤除单帧伪点，而不是按距离硬切）
    stop_confirm_frames: int = 3
    release_m: float = 0.9               # 高于该净空才解除"保持"（迟滞）
    # 解除也要**连续 N 次**满足（服务器侧 2026-10-13 建议的"进出双门限"）。
    # 为什么需要（2026-10-15 d4 实测）：释放是瞬时的，地图偶尔一跳 ≥release_m 就放行，
    # 于是出现"保持↔释放"来回切换（净空 0.45–0.58 m、每段仅 14–15 帧），绕行永远做不完。
    # 解除延后只会让飞机**多保持一会儿**（更保守），不会削弱安全。
    release_confirm_frames: int = 3
    avoid_yaw_rate: float = 0.15         # 保持时朝较宽一侧的偏航角速度（覆盖 VLM）
    side_release_m: float = 1.2          # 所选侧也要恢复到该净空才解除（迟滞）
    max_turn_deg: float = 60.0           # 相对进入避障时的航向，累积转角上限（防死角打转）
    creep_mps: float = 0.1               # 较宽侧明显开阔时的小前向（绕行用，0 = 关闭）
    # "明显开阔"的判定净空（服务器侧 2026-10-15 要求 ③a：creep 只在所选侧 ≥2.0 m 时才给——
    # 安全前提是"侧向确实有空间可以蹭过去"，而不是"当前净空还没触底"）
    creep_open_side_m: float = 2.0
    # 压速带（stop_m ~ slow_start_m）里的**前向下限**（0 = 关闭）。
    # 为什么需要（2026-10-15 实测）：压速是按净空线性缩放，净空越接近 stop_m 前向越趋近 0，
    # 于是"越想靠近越不走"，飞机长期停在 0.6~0.9 m 之间，永远跨不进保持带 → 拿不到 episode。
    #
    # 服务器侧 2026-10-15 认可作为试验默认，并**写死两条约束**（本实现逐条对应）：
    #   1) 只在 `guard == obstacle_slow` 时生效——这个分支本身就排除了
    #      保持(obstacle)/脱困(obstacle_escape)/陈旧(stale)/未交权(dry_run/defer)；
    #      且**只放大本来就想前进的帧**（`command.forward > 0`）：
    #      forward ≤ 0（VLM 显式停止）或纯转向（forward=0 且 yaw≠0）时**一律不加下限**；
    #   2) 每帧把"是否施加了下限"落盘为 `guard_detail['creep_floor']`，便于事后区分
    #      "模型想走" 与 "被下限推着走"。
    # 另：量级与保持态已有的 creep_mps 一致，且**不改变 stop_m（0.6 m 安全阈值）**。
    # 服务器侧备注：本质是"给语义层加前向偏置"，等 ② 集成时值得回头评估是否该在语义层解决。
    slow_min_forward_mps: float = 0.0
    # **盲区底线**（0 = 关闭）：地图的 `min_range` 过滤（本机 0.45 m）会把更近的点当自机体回波丢掉，
    # 所以净空读数在 0.45 m 处**触底**——"看不见"不等于"没有障碍"。
    # 只要净空 ≤ 该值，就认为已经贴到传感器盲区：**一律归零前向**（保持态也不再 creep），
    # 但**保留偏航**（继续朝宽侧转，把视野打开）。2026-10-15 用户观察到贴近障碍后加。
    blind_zone_m: float = 0.5


class ObstacleTrack:
    """世界系"障碍候选点"的短时 track（服务器侧 2026-10-13 建议）。

    为什么不只和"上一帧候选点"链式比较：

    - 链式比较会**随帧缓慢漂移**（每帧只差一点点也一路通过）；
    - 中间丢一帧就清零重整，容易误判。

    这里维护最近 ``size`` 个世界坐标点的**质心**：新点必须落在质心的
    ``tolerance_m`` 内才算"同一个障碍"，随后并入 track；否则视为新目标（计数重新开始）。
    """

    def __init__(self, size: int = 3, tolerance_m: float = 0.4):
        self.size = max(int(size), 1)
        self.tolerance_m = float(tolerance_m)
        self._points = []

    @property
    def centroid(self):
        if not self._points:
            return None
        count = len(self._points)
        return tuple(sum(p[i] for p in self._points) / count for i in range(3))

    def update(self, point) -> bool:
        """喂入本帧世界坐标；返回"是否与既有 track 一致"。"""
        if point is None:
            return False
        center = self.centroid
        if center is None:
            self._points = [tuple(point)]
            return True
        distance = math.dist(tuple(point), center)
        consistent = distance <= self.tolerance_m
        if consistent:
            self._points.append(tuple(point))
            self._points = self._points[-self.size:]
        else:
            self._points = [tuple(point)]
        return consistent

    def reset(self) -> None:
        self._points = []


def parse_clearance_status(text: str):
    """解析 ``/local_map/status``：``... nearest_forward_m=0.42 left_m=3.00 right_m=2.10``。

    返回 ``(forward, left, right)``；解析不出来返回 ``None``（上层按"未知"处理）。
    """
    if not text:
        return None
    match = _CLEARANCE_RE.search(text.replace(',', ' '))
    if not match:
        return None
    values = []
    for group in match.groups():
        try:
            values.append(float(group))
        except ValueError:
            return None
    return tuple(values)


def apply_map_safety(command: BodyVelocity, clearance, age_s: float, holding: bool,
                     limits: MapSafetyLimits, turn_deg: float = 0.0, stop_streak: int = 0,
                     world_consistent: bool = True, defer: bool = False,
                     at_min_range: bool = False, creep_allowed: bool = True):
    """对命令施加地图安全层。

    返回 ``(新命令, guard, 新 holding, 细节dict)``：

    - ``guard`` ∈ ``map_off / ok / stale / obstacle_slow / obstacle``；
    - ``holding`` 需要在上层保存（迟滞状态），下次调用传回来；
    - ``turn_deg`` 是"相对进入避障时航向"的累积转角（上层按航向算好传进来），
      达到 ``max_turn_deg`` 后**停止继续转**，避免在死角里绕圈；
      较宽侧明显开阔（≥``creep_open_side_m``）时给 ``creep_mps`` 的小前向配合绕行，
      只有两侧都紧才完全归零保持。
    - ``world_consistent``：本帧近场点在**世界系**里是否与上一帧候选点接近。
      自运动拖影在世界系里逐帧跳变，真障碍稳定——用它把"接近但世界坐标在跳"的帧
      排除在保持确认之外（服务器侧 2026-10-13 的建议）。
    - ``defer``：**位姿质量不足**（位置陈旧 / 角速度过大）时置位——此时世界系判据不可靠，
      因此**暂缓判定**：既不当真障碍，也不判拖影（服务器侧 2026-10-14）。已在保持就维持保持，
      否则只压速、既不累计也不清零 ``stop_streak``，等位姿恢复后再判。
    - ``at_min_range``：地图上报的净空**触底**（等于 ``min_range`` 过滤值）时置位——
      它的含义是"≤该值，未知"，不是"有这么多余量"（服务器侧 2026-10-15）。与盲区同等待遇。
    - ``creep_allowed``：保持态"朝宽侧 creep"的**预算闸**（服务器侧 2026-10-15 要求 ③b）：
      由上层按"单次保持内累计 creep 距离/时间"判定，超预算就传 False（只转不推）。
    """
    if not limits.enabled:
        return command, 'map_off', holding, {}

    def _with(forward=None, left=None, up=None, yaw_rate=None):
        return BodyVelocity(
            command.forward if forward is None else forward,
            command.left if left is None else left,
            command.up if up is None else up,
            command.yaw_rate if yaw_rate is None else yaw_rate,
            command.duration_s)

    if clearance is None or (age_s is not None and age_s > limits.stale_s):
        limited = math.copysign(
            min(abs(command.forward), limits.stale_forward_limit_mps), command.forward)
        return (_with(forward=limited), 'stale', holding,
                {'reason': 'clearance_unknown', 'age_s': None if age_s is None else round(age_s, 2),
                 'forward_limit_mps': limits.stale_forward_limit_mps,
                 'stop_streak': 0})

    forward_c, left_c, right_c = clearance

    if defer:
        # 位姿质量不足 → 暂缓判定（不采信"真障碍"，也不采信"拖影"）
        if holding:
            yaw = limits.avoid_yaw_rate if left_c >= right_c else -limits.avoid_yaw_rate
            side = 'left' if left_c >= right_c else 'right'
            capped = abs(turn_deg) >= limits.max_turn_deg
            return (_with(forward=0.0, left=0.0, yaw_rate=0.0 if capped else yaw),
                    'obstacle', True,
                    {'reason': 'pose_defer_hold', 'pose_defer': True, 'avoid_side': side,
                     'forward_m': round(forward_c, 2), 'left_m': round(left_c, 2),
                     'right_m': round(right_c, 2), 'stop_streak': stop_streak})
        limited = math.copysign(
            min(abs(command.forward), limits.stale_forward_limit_mps), command.forward)
        return (_with(forward=limited), 'stale', holding,
                {'reason': 'pose_defer', 'pose_defer': True, 'forward_m': round(forward_c, 2),
                 'stop_streak': stop_streak})

    # 迟滞：进入用 stop_m（且需连续 confirm 帧），退出要求"前向恢复到 release_m 且所选侧 ≥ side_release_m"
    if holding:
        release_now = (forward_c >= limits.release_m
                       and max(left_c, right_c) >= limits.side_release_m)
        # 释放侧也用**同一个连续计数**做确认（"进出双门限"）：连续 N 帧满足才真正释放
        stop_streak = (stop_streak + 1) if release_now else 0
        holding = not (stop_streak >= max(int(limits.release_confirm_frames), 1))
        if not holding:
            stop_streak = 0
    else:
        if forward_c < limits.stop_m and world_consistent:
            stop_streak += 1
        elif forward_c < limits.stop_m:
            stop_streak = 1      # 近但在跳（疑拖影）：不累计，重新观察
        else:
            stop_streak = 0
        holding = stop_streak >= max(int(limits.stop_confirm_frames), 1)
    if holding:
        # 障碍保持：偏航**覆盖** VLM（朝更宽一侧）
        yaw = limits.avoid_yaw_rate if left_c >= right_c else -limits.avoid_yaw_rate
        side = 'left' if left_c >= right_c else 'right'
        side_clear = max(left_c, right_c)
        creep = limits.creep_mps if (limits.creep_mps > 0.0 and creep_allowed
                                     and side_clear >= limits.creep_open_side_m) else 0.0
        blind = ((limits.blind_zone_m > 0.0 and forward_c <= limits.blind_zone_m)
                 or at_min_range)
        if blind:
            creep = 0.0          # 已贴到盲区：不再前推，只保留偏航（继续朝宽侧转）
        capped = abs(turn_deg) >= limits.max_turn_deg
        if capped:
            # 到达转角上限：停止继续转（避免原地绕圈）。
            # 并且**同时把前向归零**——2026-10-16 k7-e2 实测的教训：
            # "封顶不转 + creep 继续推"会贴死在结构上（右侧本来是开的，却因为不转而没有绕开）。
            # 转不动就不推，交给脱困/超时/预算兜底。
            yaw = 0.0
            creep = 0.0
        return (_with(forward=creep, left=0.0, yaw_rate=yaw), 'obstacle', True,
                {'reason': 'obstacle_hold', 'forward_m': round(forward_c, 2),
                 'left_m': round(left_c, 2), 'right_m': round(right_c, 2),
                 'avoid_side': side, 'yaw_rate': round(yaw, 2),
                 'side_clear_m': round(side_clear, 2), 'creep_mps': round(creep, 2),
                 'turn_deg': round(turn_deg, 1), 'yaw_capped': capped,
                 'stop_streak': stop_streak, 'release_streak': stop_streak,
                 'blind_zone': blind})

    if forward_c < limits.slow_start_m:
        span = max(limits.slow_start_m - limits.stop_m, 1e-6)
        scale = min(max((forward_c - limits.stop_m) / span, 0.0), 1.0)
        scaled = command.forward * scale
        floored = False
        # 走到这里 ⇒ guard 必为 obstacle_slow（enabled 关→map_off；陈旧→stale；defer→stale/obstacle；
        # holding→obstacle 都已提前返回）。下限**只放大本来就想前进的帧**：
        # forward ≤ 0（VLM 显式停止）或纯转向时 command.forward 不 > 0 → 不加下限。
        if (limits.slow_min_forward_mps > 0.0 and command.forward > 0.0
                and scaled < limits.slow_min_forward_mps):
            scaled = min(limits.slow_min_forward_mps, command.forward)
            floored = True
        blind = ((limits.blind_zone_m > 0.0 and forward_c <= limits.blind_zone_m)
                 or at_min_range)
        if blind:
            scaled = 0.0         # 盲区内一律不前推（含刚被抬起的下限）
            floored = False
        return (_with(forward=scaled), 'obstacle_slow', False,
                {'reason': 'obstacle_slow', 'forward_m': round(forward_c, 2),
                 'scale': round(scale, 3), 'stop_streak': stop_streak,
                 'creep_floor': floored, 'blind_zone': blind})

    return command, 'ok', False, {'forward_m': round(forward_c, 2),
                                  'stop_streak': stop_streak}
