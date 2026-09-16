"""启动 5 m 方形航点任务：offboard_bridge + waypoint_mission。"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    square_5m = [
        0.0, 0.0, 3.0,
        5.0, 0.0, 3.0,
        5.0, 5.0, 3.0,
        0.0, 5.0, 3.0,
        0.0, 0.0, 3.0,
    ]

    return LaunchDescription([
        Node(
            package='px4_interface',
            executable='offboard_bridge',
            name='offboard_bridge',
            output='screen',
        ),
        Node(
            package='mission',
            executable='waypoint_mission',
            name='waypoint_mission',
            output='screen',
            parameters=[{
                'waypoints': square_5m,
                'takeoff_altitude_m': 3.0,
                'arrival_radius_m': 0.5,
            }],
        ),
    ])
