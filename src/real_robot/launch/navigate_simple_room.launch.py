"""
navigate_simple_room.launch.py  (real_robot 패키지)

저장된 simple_room 맵을 로드해서 AMCL + Nav2 자율 주행.
- gazebo include: real_robot/launch/gazebo_simple_room.launch.py
- map: real_robot/maps/simple_room_map.yaml
  (다른 이름으로 저장했다면 map_file 경로만 수정)
- nav2_params: ur3_4040_urdf/config/nav2_params.yaml

매핑 launch 로 mapping 후 map_saver_cli 로 저장한 결과를 사용.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ur3 = get_package_share_directory('ur3_4040_urdf')   # nav2_params
    pkg_real = get_package_share_directory('real_robot')     # map + gazebo launch
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_file = os.path.join(pkg_real, 'maps', 'my_real_map.yaml')
    nav2_params_file = os.path.join(pkg_ur3, 'config', 'nav2_params.yaml')

    # 1. Gazebo (simple_room) + robot spawn + bridge
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_real, 'launch', 'gazebo_simple_room.launch.py')
        )
    )

    # 2. Localization (map_server + AMCL)
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

    # 3. Navigation
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

    # 4. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo_launch,
        localization_launch,
        navigation_launch,
        rviz_node,
    ])
