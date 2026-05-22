"""
mission.launch.py — 외부에서 goal 토픽 발행으로 robot 미션 실행

구성: Gazebo + Nav2 (map_server + AMCL + planner/controller/bt) + goal_relay_node
RViz 없음 (필요한 결과는 launch 터미널에 logger 로 표시).

Usage:
  ros2 launch ur3_4040_urdf mission.launch.py

  # 다른 터미널에서 goal 발행:
  ros2 topic pub --once /simple_goal geometry_msgs/PoseStamped \\
    "{header: {frame_id: 'map'},
      pose: {position: {x: 5.0, y: 2.0, z: 0.0},
             orientation: {x: 0, y: 0, z: 0.707, w: 0.707}}}"

  # 결과는 launch 터미널에 출력 + /navigation_result 토픽으로도 publish
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ur3_4040_urdf')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_file = os.path.join(pkg_share, 'maps', 'my_capstone_map.yaml')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # 1. Gazebo + robot spawn + bridge
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
        )
    )

    # 2. Localization (map_server + AMCL) — Gazebo 안정화 후 시작
    localization_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
                ),
                launch_arguments={
                    'map': map_file,
                    'params_file': nav2_params_file,
                    'use_sim_time': 'true',
                    'autostart': 'true',
                }.items()
            )
        ]
    )

    # 3. Navigation (planner/controller/bt) — Localization activate 후
    navigation_launch = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'params_file': nav2_params_file,
                    'use_sim_time': 'true',
                    'autostart': 'true',
                }.items()
            )
        ]
    )

    # 4. Goal relay — nav2 fully active 직후 시작
    #    /simple_goal 토픽 받아서 navigate_to_pose action 으로 전달
    #    feedback (거리/ETA) + result 를 launch 터미널에 출력
    goal_relay_node = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='ur3_4040_urdf',
                executable='goal_relay_node.py',
                name='goal_relay',
                output='screen',
                emulate_tty=True,
                parameters=[{'use_sim_time': True}],
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        localization_launch,
        navigation_launch,
        goal_relay_node,
    ])
