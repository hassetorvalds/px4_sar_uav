"""统一启动仿真栈（AirSim + PX4 + ROS 2 各层节点）。

用法：
    ros2 launch bringup sim_stack.launch.py                       # 接口+传感器+地图+规划
    ros2 launch bringup sim_stack.launch.py with_mission:=true    # 再加航点任务
    ros2 launch bringup sim_stack.launch.py with_vlm:=true        # 再加 VLM 导航
    ros2 launch bringup sim_stack.launch.py with_explorer:=true   # 再加自主探索
    ros2 launch bringup sim_stack.launch.py airsim_host:=172.21.192.1
    ros2 launch bringup sim_stack.launch.py with_arbiter:=false   # 关掉速度仲裁（回到"直连"旧行为）

速度仲裁（默认开启）：local_planner 与 vlm_navigator 都把速度指令发给
`velocity_arbiter`，由它按优先级（几何避障 avoid > 语义导航 vlm）选一路转发给
`offboard_bridge`，避免两者同时运行在同一话题上互相覆盖。只有一路在发时行为等价于直连。

前置：Windows 侧 AirSim 已运行；scripts/run_agent.sh 与 scripts/run_px4_sitl.sh 已启动。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('airsim_host', default_value='172.21.192.1'),
        DeclareLaunchArgument('airsim_port', default_value='45000'),
        DeclareLaunchArgument('camera_rate_hz', default_value='2.5'),
        DeclareLaunchArgument('with_mission', default_value='false'),
        DeclareLaunchArgument('with_vlm', default_value='false'),
        DeclareLaunchArgument('with_explorer', default_value='false'),
        DeclareLaunchArgument('with_arbiter', default_value='true'),
    ]
    arbiter_on = LaunchConfiguration('with_arbiter')
    arbiter_if = IfCondition(arbiter_on)
    # 仲裁开启时，两个发布者把速度指令发给仲裁器；关闭时直连 bridge（旧行为）。
    avoid_topic = PythonExpression([
        "'/velocity_arbiter/avoid_cmd' if '", arbiter_on,
        "' == 'true' else '/offboard_bridge/velocity_setpoint'",
    ])
    vlm_topic = PythonExpression([
        "'/velocity_arbiter/vlm_cmd' if '", arbiter_on,
        "' == 'true' else '/offboard_bridge/velocity_setpoint'",
    ])
    return LaunchDescription(args + [
        # 接口层：唯一直接接触 /fmu/* 的节点
        Node(package='px4_interface', executable='offboard_bridge',
             name='offboard_bridge', output='screen'),
        # 速度仲裁：把多路速度指令按优先级合并成一路喂给 offboard_bridge
        Node(package='px4_interface', executable='velocity_arbiter',
             name='velocity_arbiter', output='screen',
             parameters=[{'priority': 'avoid,vlm'}],
             condition=arbiter_if),
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
        # 规划层：目标跟踪 + 避障（经仲裁器；关掉仲裁时直连，保持旧行为）
        Node(package='planner', executable='local_planner',
             name='local_planner', output='screen',
             parameters=[{'velocity_topic': avoid_topic}]),
        # 可选：任务层状态机
        Node(package='mission', executable='waypoint_mission',
             name='waypoint_mission', output='screen',
             condition=IfCondition(LaunchConfiguration('with_mission'))),
        # 可选：VLM 导航（需要 provider/API key 参数，按需覆盖）
        Node(package='vlm', executable='vlm_navigator',
             name='vlm_navigator', output='screen',
             parameters=[{'velocity_topic': vlm_topic}],
             condition=IfCondition(LaunchConfiguration('with_vlm'))),
        # 可选：自主探索（frontier）
        Node(package='planner', executable='frontier_explorer',
             name='frontier_explorer', output='screen',
             condition=IfCondition(LaunchConfiguration('with_explorer'))),
    ])
