"""A* 规划/跟随/桥的部署入口；默认全部关闭，不接管飞控。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('goal_task_file', default_value=''),
        DeclareLaunchArgument('planner_enabled', default_value='true'),
        DeclareLaunchArgument('follower_enabled', default_value='false'),
        DeclareLaunchArgument('bridge_enabled', default_value='false'),
        DeclareLaunchArgument('allow_flight', default_value='false'),
        DeclareLaunchArgument('plan_rate_hz', default_value='1.0'),
        DeclareLaunchArgument('max_speed_mps', default_value='0.3'),
        DeclareLaunchArgument('max_active_s', default_value='5.0'),
    ]
    return LaunchDescription(args + [
        Node(package='planner', executable='astar_planner',
             name='astar_planner', output='screen',
             parameters=[{
                 'goal_task_file': LaunchConfiguration('goal_task_file'),
                 'plan_rate_hz': LaunchConfiguration('plan_rate_hz'),
             }]),
        Node(package='planner', executable='astar_follower',
             name='astar_follower', output='screen',
             parameters=[{
                 'enabled': LaunchConfiguration('follower_enabled'),
                 'max_speed_mps': LaunchConfiguration('max_speed_mps'),
             }]),
        Node(package='planner', executable='astar_velocity_bridge',
             name='astar_velocity_bridge', output='screen',
             parameters=[{
                 'enabled': LaunchConfiguration('bridge_enabled'),
                 'allow_flight': LaunchConfiguration('allow_flight'),
                 'max_speed_mps': LaunchConfiguration('max_speed_mps'),
                 'max_active_s': LaunchConfiguration('max_active_s'),
             }]),
    ])
