"""接近终止策略（纯函数，便于单元测试与复用）。

分层约定：VLM 层只负责“看到什么、指向哪里”；**是否继续接近、何时停下**
属于任务层策略，放在这里决策，VLM 层的 stop_distance 只作为兜底。
"""

from dataclasses import dataclass


@dataclass
class ApproachDecision:
    action: str      # 'continue' | 'hold' | 'timeout'
    reason: str = ''


def evaluate_approach(depth_m, elapsed_s, standoff_m: float = 0.8,
                      timeout_s: float = 60.0, have_target: bool = True,
                      already_holding: bool = False) -> ApproachDecision:
    """根据当前目标深度与已用时间决定是否继续接近。

    - 没有目标：继续等待（不产生任何动作）
    - 深度 ≤ 站定距离：停止接近（hold）
    - 超过接近超时：停止接近并标记超时
    """
    if already_holding:
        return ApproachDecision('hold', '已在站定保持')
    if not have_target or depth_m is None:
        return ApproachDecision('continue', '等待目标')
    if depth_m <= standoff_m:
        return ApproachDecision('hold', f'已接近到 {depth_m:.2f} m（阈值 {standoff_m:.2f} m）')
    if elapsed_s >= timeout_s:
        return ApproachDecision('timeout', f'接近超时（{elapsed_s:.0f} s，当前 {depth_m:.2f} m）')
    return ApproachDecision('continue', '')
