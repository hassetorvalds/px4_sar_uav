"""高度门 C 验证核心（可注入、无 ROS 依赖）。

C 口径：飞行高度与簇顶都取 **map/REL**，共同漂移在相减时抵消：
    overflyable = (flight_z_map - cluster_top_map) >= clearance_m

legacy 口径：一律判障碍、永不飞越（overflyable=False）。
"""

from dataclasses import dataclass, asdict

MODES = ('legacy', 'c_map_rel')


@dataclass
class GateDecision:
    mode: str
    frame_id: str
    phase: str
    anchor_id: str
    transform_id: str
    px4_z: float
    airsim_z: float
    map_z: float
    cluster_top_z: float
    clearance_m: float
    gap_m: float
    overflyable: bool
    reason: str

    def to_dict(self):
        return asdict(self)


def evaluate_height_gate(*, mode: str, phase: str, px4_z: float,
                         airsim_z: float, map_z: float, cluster_top_z: float,
                         clearance_m: float, frame_id: str = 'map',
                         anchor_id: str = 'local_map',
                         transform_id: str = 'static_arm') -> GateDecision:
    if mode not in MODES:
        raise ValueError(f'未知高度门模式 {mode!r}，可选 {MODES}')
    if mode == 'legacy':
        gap = float(px4_z) - float(cluster_top_z)
        return GateDecision(
            mode='legacy', frame_id=frame_id, phase=phase,
            anchor_id=anchor_id, transform_id=transform_id,
            px4_z=float(px4_z), airsim_z=float(airsim_z), map_z=float(map_z),
            cluster_top_z=float(cluster_top_z), clearance_m=float(clearance_m),
            gap_m=gap, overflyable=False,
            reason='legacy_no_overfly')
    gap = float(map_z) - float(cluster_top_z)
    overfly = gap >= float(clearance_m)
    return GateDecision(
        mode='c_map_rel', frame_id=frame_id, phase=phase,
        anchor_id=anchor_id, transform_id=transform_id,
        px4_z=float(px4_z), airsim_z=float(airsim_z), map_z=float(map_z),
        cluster_top_z=float(cluster_top_z), clearance_m=float(clearance_m),
        gap_m=gap, overflyable=overfly,
        reason='c_map_rel')
