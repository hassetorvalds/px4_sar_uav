"""VLM TaskSpec → A* 目标（纯逻辑）。"""


def goal_from_task(task, position_enu):
    """取 TaskSpec 的第一个 goto，按 coordinate_frame 转为 A* 目标 (east,north)。"""
    steps = task.get('steps') or []
    goto = next((step for step in steps if step.get('type') == 'goto'), None)
    if goto is None:
        raise ValueError('task has no goto')
    params = goto.get('params') or {}
    if params.get('east_m') is None or params.get('north_m') is None:
        raise ValueError('goto missing east_m/north_m')
    east = float(params['east_m'])
    north = float(params['north_m'])
    if task.get('coordinate_frame', 'relative') == 'relative':
        east += float(position_enu[0])
        north += float(position_enu[1])
    return east, north
