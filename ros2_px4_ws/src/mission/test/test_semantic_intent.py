"""SemanticIntent 1.0 校验/编译单测。"""

from mission.semantic_intent import SemanticIntent


def _area_intent(**overrides):
    data = {
        'contract_version': '1.0', 'intent_id': 't1', 'task_type': 'area_cover',
        'target': {'label': 'target', 'world_enu': [5.0, -6.0], 'frame_id': 'map'},
        'constraints': {
            'altitude_band_m': [2.0, 2.0], 'altitude_frame': 'REL',
            'speed_cap_mps': 0.5, 'keepout': [], 'min_clearance_m': 0.6,
            'area': {'shape': 'rectangle', 'center_east_m': 5.0,
                     'center_north_m': -6.0, 'width_m': 6.0,
                     'height_m': 6.0, 'spacing_m': 2.0}},
        'priority': 'normal', 'confidence': 1.0, 'source': 'operator',
        'facts_used': ['map'], 'notes': 'test',
    }
    data.update(overrides)
    return SemanticIntent.from_dict(data)


def test_area_cover_compiles_to_search_task():
    spec, reasons = _area_intent().compile()
    assert reasons == []
    assert [step.type for step in spec.steps] == ['takeoff', 'search', 'land']
    assert spec.steps[1].params['shape'] == 'rectangle'
    assert spec.steps[1].params['width_m'] == 6.0


def test_go_to_and_return_compile():
    go = SemanticIntent(task_type='go_to',
                        target={'label': 'target', 'world_enu': [5.0, -6.0],
                                'frame_id': 'map'},
                        constraints={'altitude_band_m': [2.0, 2.0],
                                     'altitude_frame': 'REL'})
    spec, reasons = go.compile()
    assert reasons == []
    assert [step.type for step in spec.steps] == ['takeoff', 'goto', 'land']

    ret = SemanticIntent(task_type='return',
                         constraints={'altitude_band_m': [2.0, 2.0],
                                      'altitude_frame': 'REL'})
    spec, reasons = ret.compile()
    assert reasons == []
    assert [step.type for step in spec.steps] == ['takeoff', 'return_home']


def test_rejects_bad_contract_fields():
    assert _area_intent(contract_version='2.0').validate()[0].code == 'UNSUPPORTED_VERSION'
    assert _area_intent(task_type='track').validate()[0].code == 'UNKNOWN_TASK_TYPE'
    intent = _area_intent()
    intent.constraints['altitude_frame'] = 'LCL'
    assert intent.validate()[0].code == 'UNSUPPORTED_ALTITUDE_FRAME'


def test_rejects_low_confidence_vlm():
    reasons = _area_intent(source='vlm', confidence=0.4).validate()
    assert any(reason.code == 'LOW_CONFIDENCE' for reason in reasons)


def test_rejects_target_in_keepout():
    intent = _area_intent()
    intent.constraints['keepout'] = [[[0, -10], [10, -10], [10, 0], [0, 0]]]
    reasons = intent.validate()
    assert any(reason.code == 'TARGET_IN_KEEPOUT' for reason in reasons)


def test_area_cover_requires_area():
    intent = _area_intent()
    intent.constraints.pop('area')
    reasons = intent.validate()
    assert any(reason.code == 'SCHEMA_INVALID' for reason in reasons)
