"""候选速度净空复核测试。"""

import pytest

from planner.candidate_safety import gate_candidate


def test_gate_forward_blocks_east():
    out = gate_candidate(0.4, 0.0, heading_rad=0.0, forward_m=0.4,
                         left_m=2.0, right_m=2.0, stop_m=0.6)
    assert out.blocked is True
    assert out.east == 0.0
    assert out.north == 0.0


def test_gate_unknown_left_blocks_left():
    out = gate_candidate(0.0, 0.4, heading_rad=0.0, forward_m=2.0,
                         left_m=float('inf'), left_known=False,
                         right_m=2.0, stop_m=0.6)
    assert out.blocked is True
    assert out.north == 0.0


def test_gate_allows_clear_forward():
    out = gate_candidate(0.4, 0.0, heading_rad=0.0, forward_m=2.0,
                         left_m=2.0, right_m=2.0, stop_m=0.6)
    assert out.blocked is False
    assert out.east == pytest.approx(0.4)
