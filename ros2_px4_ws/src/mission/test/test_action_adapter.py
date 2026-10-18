"""ExecuteTask feedback 映射的单测（不依赖 ROS 运行）。"""

from mission.task_action_server import status_to_feedback


def test_feedback_maps_phase_step_and_altitude():
    status = {'phase': 'RUN', 'step': 1, 'step_total': 3, 'step_type': 'search',
              'altitude_lcl': {'value_m': 1.5},
              'task_altitude': {'value_m': 2.0}}
    feedback = status_to_feedback(status)
    assert feedback.phase == 'RUN'
    assert feedback.step_index == 1
    assert feedback.step_total == 3
    assert feedback.step_type == 'search'
    assert abs(feedback.progress - 1 / 3) < 1e-6
    assert feedback.altitude_lcl_m == 1.5
    assert feedback.target_altitude_m == 2.0


def test_feedback_missing_altitude_is_nan_and_progress_clamped():
    feedback = status_to_feedback({'phase': 'IDLE', 'step': 0, 'step_total': 0})
    assert feedback.progress == 0.0
    assert feedback.altitude_lcl_m != feedback.altitude_lcl_m  # NaN
