"""接近终止策略的单元测试（纯函数，不需要 ROS 运行）。"""

from mission.approach_policy import evaluate_approach


def test_continue_when_far():
    decision = evaluate_approach(depth_m=2.5, elapsed_s=5.0, standoff_m=0.8)
    assert decision.action == 'continue'


def test_hold_when_within_standoff():
    decision = evaluate_approach(depth_m=0.75, elapsed_s=5.0, standoff_m=0.8)
    assert decision.action == 'hold'
    assert '0.75' in decision.reason


def test_timeout_when_exceeds_deadline():
    decision = evaluate_approach(depth_m=3.0, elapsed_s=61.0, timeout_s=60.0)
    assert decision.action == 'timeout'


def test_wait_without_target():
    decision = evaluate_approach(depth_m=None, elapsed_s=5.0, have_target=False)
    assert decision.action == 'continue'
    assert decision.reason == '等待目标'


def test_hold_is_sticky():
    decision = evaluate_approach(depth_m=5.0, elapsed_s=1.0, already_holding=True)
    assert decision.action == 'hold'
