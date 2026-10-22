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


def _polygon_intent(**overrides):
    data = _area_intent(**overrides).to_dict()
    data['constraints'] = dict(data['constraints'])
    data['constraints'].pop('area', None)
    data['constraints']['area'] = {
        'frame_id': 'map',
        'polygon': [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]],
        'overlap': 0.25,
    }
    return SemanticIntent.from_dict(data)


def test_polygon_area_compiles_map_frame():
    spec, reasons = _polygon_intent().compile()
    assert reasons == []
    assert spec.coordinate_frame == 'map'
    assert spec.steps[1].params['shape'] == 'polygon'
    assert abs(spec.steps[1].params['spacing_m'] - 3.0) < 1e-6  # footprint 4 * (1-0.25)
    assert spec.intent_id == 't1'


def test_self_intersecting_polygon_rejected():
    intent = _polygon_intent()
    intent.constraints['area']['polygon'] = [[0, 0], [4, 4], [4, 0], [0, 4]]
    assert any(reason.code == 'SCHEMA_INVALID' for reason in intent.validate())


def test_vlm_area_cover_requires_operator():
    reasons = _area_intent(source='vlm', confidence=0.9).validate()
    assert any(reason.code == 'OPERATOR_REQUIRED' for reason in reasons)


def test_line_spacing_wins_over_overlap():
    intent = _polygon_intent()
    intent.constraints['area']['line_spacing_m'] = 1.5
    spec, reasons = intent.compile()
    assert reasons == []
    assert spec.steps[1].params['spacing_m'] == 1.5


def _fenced_intent():
    intent = _polygon_intent()
    intent.constraints['map_boundary'] = [[-1.0, -1.0], [5.0, -1.0],
                                          [5.0, 5.0], [-1.0, 5.0]]
    intent.constraints['keepout_polygons'] = []
    return intent


def test_polygon_inside_map_boundary_passes():
    spec, reasons = _fenced_intent().compile()
    assert reasons == []
    assert spec.steps[1].params['polygon'][0] == [0.0, 0.0]


def test_polygon_outside_map_boundary_rejected():
    intent = _fenced_intent()
    intent.constraints['area']['polygon'][0] = [6.0, 0.0]
    reasons = intent.validate()
    assert any(reason.code == 'ERROR_TARGET_OUTSIDE_MAP' for reason in reasons)
    detail = next(r.detail for r in reasons if r.code == 'ERROR_TARGET_OUTSIDE_MAP')
    assert 'area.polygon[0]' in detail


def test_polygon_in_keepout_polygons_rejected():
    intent = _fenced_intent()
    intent.constraints['keepout_polygons'] = [
        [[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]]
    reasons = intent.validate()
    assert any(reason.code == 'ERROR_TARGET_IN_KEEPOUT' for reason in reasons)


def test_missing_map_boundary_preserves_old_behavior():
    intent = _polygon_intent()
    intent.constraints.pop('map_boundary', None)
    spec, reasons = intent.compile()
    assert reasons == []
    assert spec is not None


def test_any_outside_polygon_point_rejects_with_index():
    intent = _fenced_intent()
    intent.constraints['area']['polygon'] = [
        [0.0, 0.0], [6.0, 0.0], [4.0, 4.0], [0.0, 4.0]]
    reasons = intent.validate()
    detail = next(r.detail for r in reasons
                  if r.code == 'ERROR_TARGET_OUTSIDE_MAP')
    assert 'area.polygon[1]' in detail


def test_explicit_empty_keepout_polygons_disables_fallback():
    # 方案 1：显式 [] = 无 exclusion，不回退 keepout；
    # 即使 keepout 包含 area 顶点，也不应报 ERROR_TARGET_IN_KEEPOUT。
    intent = _fenced_intent()
    intent.constraints['keepout_polygons'] = []
    intent.constraints['keepout'] = [
        [[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]]
    spec, reasons = intent.compile()
    assert reasons == []
    assert spec is not None


def test_overlap_override_missing_keeps_default():
    intent = _polygon_intent()
    spec, reasons = intent.compile()
    assert reasons == []
    assert abs(spec.steps[1].params['spacing_m'] - 3.0) < 1e-6
    assert not any('overlap_override' in note for note in spec.notes)


def test_overlap_override_is_recorded_without_changing_overlap():
    intent = _polygon_intent()
    intent.constraints['area']['overlap_override'] = 0.5
    spec, reasons = intent.compile()
    assert reasons == []
    assert abs(spec.steps[1].params['spacing_m'] - 3.0) < 1e-6
    assert 'area.overlap_override=0.5' in spec.notes


def test_overlap_override_invalid_value_rejected():
    intent = _polygon_intent()
    intent.constraints['area']['overlap_override'] = 1.5
    reasons = intent.validate()
    assert any(reason.code == 'SCHEMA_INVALID'
               and reason.field == 'constraints.area.overlap_override'
               for reason in reasons)


def test_missing_keepout_polygons_falls_back_to_keepout():
    intent = _fenced_intent()
    intent.constraints.pop('keepout_polygons', None)
    intent.constraints['keepout'] = [
        [[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]]
    reasons = intent.validate()
    assert any(reason.code == 'ERROR_TARGET_IN_KEEPOUT' for reason in reasons)
