"""统一启动仿真栈（AirSim + PX4 + ROS 2 各层节点）。

用法：
    ros2 launch bringup sim_stack.launch.py                       # 接口+传感器+地图+规划
    ros2 launch bringup sim_stack.launch.py with_mission:=true    # 再加航点任务
    ros2 launch bringup sim_stack.launch.py with_vlm:=true        # 再加 VLM 导航
    ros2 launch bringup sim_stack.launch.py with_explorer:=true   # 再加自主探索
    ros2 launch bringup sim_stack.launch.py airsim_host:=172.21.192.1

前置：Windows 侧 AirSim 已运行；scripts/run_agent.sh 与 scripts/run_px4_sitl.sh 已启动。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('airsim_host', default_value='172.21.192.1'),
        DeclareLaunchArgument('airsim_port', default_value='45000'),
        DeclareLaunchArgument('camera_rate_hz', default_value='2.5'),
        DeclareLaunchArgument('with_mission', default_value='false'),
        DeclareLaunchArgument('with_vlm', default_value='false'),
        DeclareLaunchArgument('with_explorer', default_value='false'),
    ]
    return LaunchDescription(args + [
        # 接口层：唯一直接接触 /fmu/* 的节点
        Node(package='px4_interface', executable='offboard_bridge',
             name='offboard_bridge', output='screen'),
        # 传感器层：AirSim 相机/深度/点云
        Node(package='sensor_bridge', executable='airsim_camera',
             name='airsim_camera', output='screen',
             parameters=[{
                 'host': LaunchConfiguration('airsim_host'),
                 'port': LaunchConfiguration('airsim_port'),
                 'rate_hz': LaunchConfiguration('camera_rate_hz'),
             }]),
        # 地图层：局部体素障碍地图 + 三向净空
        Node(package='mapping', executable='local_map',
             name='local_map', output='screen'),
        # 规划层：目标跟踪 + 避障
        Node(package='planner', executable='local_planner',
             name='local_planner', output='screen'),
        # 可选：任务层状态机
        Node(package='mission', executable='waypoint_mission',
             name='waypoint_mission', output='screen',
             condition=IfCondition(LaunchConfiguration('with_mission'))),
        # 可选：VLM 导航（需要 provider/API key 参数，按需覆盖）
        Node(package='vlm', executable='vlm_navigator',
             name='vlm_navigator', output='screen',
             condition=IfCondition(LaunchConfiguration('with_vlm'))),
        # 可选：自主探索（frontier）
        Node(package='planner', executable='frontier_explorer',
             name='frontier_explorer', output='screen',
             condition=IfCondition(LaunchConfiguration('with_explorer'))),
    ])
