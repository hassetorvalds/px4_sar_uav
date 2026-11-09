"""碰撞显著性测试。"""

from sensor_bridge.collision_filter import collision_label, significant_collision


def test_ground_touch_is_not_significant():
    assert significant_collision(has_collided=True, object_name='Ground_5',
                                 penetration_m=0.0) is False
    assert 'ground' in collision_label('Ground_5', 0.0)


def test_wall_is_significant():
    assert significant_collision(has_collided=True, object_name='Wall_2',
                                 penetration_m=0.0) is True


def test_ground_penetration_is_significant():
    assert significant_collision(has_collided=True, object_name='Ground_5',
                                 penetration_m=0.2) is True
