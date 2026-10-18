"""高度帧标签/比较守卫的单测。"""

import pytest

from mission.altitude import (AltitudeFrame, format_altitude, is_control_safe,
                              parse_frame, require_same_frame, to_dict)


def test_parse_frame_default_and_valid():
    assert parse_frame(None) == AltitudeFrame.REL
    assert parse_frame('lcl') == AltitudeFrame.LCL
    assert parse_frame(AltitudeFrame.TRUTH) == AltitudeFrame.TRUTH


def test_parse_frame_rejects_unknown():
    with pytest.raises(ValueError):
        parse_frame('AGL')


def test_format_altitude_labels_frame():
    assert format_altitude(2.0, 'REL') == '2.00 m REL'
    assert format_altitude(None, 'LCL') == 'Unknown'


def test_require_same_frame_guards_comparison():
    assert require_same_frame('REL', 'rel') == AltitudeFrame.REL
    with pytest.raises(ValueError):
        require_same_frame('LCL', 'TRUTH')


def test_to_dict_has_value_frame_source():
    assert to_dict(2.0, 'LCL', source='px4_local') == {
        'value_m': 2.0, 'frame': 'LCL', 'source': 'px4_local'}
    assert to_dict(None, 'REL')['value_m'] is None


def test_truth_is_not_control_safe():
    assert is_control_safe('REL')
    assert is_control_safe('LCL')
    assert not is_control_safe('TRUTH')


def test_nan_is_unknown():
    assert format_altitude(float('nan'), 'LCL') == 'Unknown'
    assert to_dict(float('nan'), 'LCL')['value_m'] is None
