"""多区域排序（纯逻辑）：按最近邻决定 search step 的执行顺序。"""

import math


def region_center(step):
    params = step.params or {}
    return (float(params.get('center_east_m', 0.0)),
            float(params.get('center_north_m', 0.0)))


def order_search_steps(spec, start=(0.0, 0.0)):
    """把 spec 里的 search step 按最近邻重排，其他 step 位置不变。

    只改变 search 之间的顺序；takeoff/land/scan 等原地不动。
    """
    indices = [index for index, step in enumerate(spec.steps)
               if step.type == 'search']
    if len(indices) < 2:
        return spec
    remaining = set(range(len(indices)))
    current = (float(start[0]), float(start[1]))
    order = []
    while remaining:
        chosen = min(
            remaining,
            key=lambda i: math.hypot(region_center(spec.steps[indices[i]])[0] - current[0],
                                     region_center(spec.steps[indices[i]])[1] - current[1]))
        remaining.remove(chosen)
        order.append(chosen)
        current = region_center(spec.steps[indices[chosen]])

    original = [spec.steps[index] for index in indices]
    for slot, chosen in zip(indices, order):
        spec.steps[slot] = original[chosen]
    spec.notes.append(
        '区域最近邻排序：' + ' → '.join(
            f'({region_center(original[i])[0]:.1f},{region_center(original[i])[1]:.1f})'
            for i in order))
    return spec
