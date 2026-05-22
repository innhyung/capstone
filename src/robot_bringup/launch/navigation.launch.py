"""
navigation.launch.py  —  실기 로봇 네비게이션 단계 (PC에서 실행)

저장된 맵을 불러와 AMCL 로 위치추정 + Nav2 로 자율주행.

전제:
- 라즈베리파이에서 sensors.launch.py 가 떠서 /scan publish 중
- ESP32 모터컨트롤러가 PC에 USB 시리얼로 연결됨
- mapping.launch.py 로 매핑 후 map_saver_cli 로 저장한 맵이 있음

이 launch 가 PC에서 띄우는 것:
- robot_state_publisher : URDF 기반 TF
- serial_bridge         : ESP32 ↔ ROS2 (/cmd_vel 수신, /odom + TF 발행)
- nav2 localization     : map_server + AMCL (map→odom TF)
- nav2 navigation       : planner / controller / behavior / bt_navigator
- rviz2

사용:
  ros2 launch robot_bringup navigation.launch.py
  ros2 launch robot_bringup navigation.launch.py map:=/경로/다른맵.yaml
RViz 의 "2D Pose Estimate" 로 초기 위치를 잡은 뒤 "Nav2 Goal" 로 목표 지정.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_urdf = get_package_share_directory('ur3_4040_urdf')
    pkg_bringup = get_package_share_directory('robot_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    xacro_file = os.path.join(pkg_urdf, 'urdf', 'full_robot.urdf.xacro')
    nav2_params_file = os.path.join(pkg_urdf, 'config', 'nav2_params.yaml')
    default_map = os.path.join(pkg_bringup, 'maps', 'my_real_map.yaml')

    # ---- launch 인자 ----
    port = LaunchConfiguration('port')
    declare_port = DeclareLaunchArgument(
        'port', default_value='/dev/ttyUSB0',
        description='ESP32 모터컨트롤러 USB 시리얼 포트')

    map_file = LaunchConfiguration('map')
    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='네비게이션에 사용할 맵 yaml 경로')

    # ---- URDF → robot_description ----
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_sim:=false']),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': False}],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    serial_bridge = Node(
        package='ros2_pid_bridge',
        executable='serial_bridge',
        name='serial_bridge',
        output='screen',
        parameters=[{
            'port': port,
            'baud': 115200,
            'wheel_radius': 0.0625,
            'wheelbase': 0.542,       # URDF 바퀴간격 기준. 실측 후 보정 권장.
            'ticks_per_rev': 1200.0,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
        }],
    )

    # Localization (map_server + AMCL) — 노드들 안정화 후 시작
    localization = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
                ),
                launch_arguments={
                    'map': map_file,
                    'params_file': nav2_params_file,
                    'use_sim_time': 'false',
                    'autostart': 'true',
                }.items(),
            )
        ],
    )

    # Navigation (planner/controller/bt) — localization activate 후 시작
    navigation = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'params_file': nav2_params_file,
                    'use_sim_time': 'false',
                    'autostart': 'true',
                }.items(),
            )
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        declare_port,
        declare_map,
        robot_state_publisher,
        joint_state_publisher,
        serial_bridge,
        localization,
        navigation,
        rviz,
    ])
