"""碰撞显著性过滤：地面轻接触不算显著碰撞。"""


def significant_collision(*, has_collided: bool, object_name: str,
                          penetration_m: float,
                          min_penetration_m: float = 0.05) -> bool:
    if not has_collided:
        return False
    if penetration_m > min_penetration_m:
        return True
    name = (object_name or '').lower()
    return not (name.startswith('ground') or name in ('', 'none'))


def collision_label(object_name: str, penetration_m: float) -> str:
    name = object_name or 'unknown'
    if name.lower().startswith('ground'):
        return f'ground_contact penetration={penetration_m:.3f}'
    return f'collision object={name} penetration={penetration_m:.3f}'
