"""
realmap_sim.launch.py — 실제 로봇 맵(real_real_final)을 Gazebo 월드로 재현해 검증.

map_to_sdf.py 로 만든 real_room.sdf(월드 좌표계=맵 좌표계) 에서:
  Gazebo + 로봇 spawn(맵 좌표 그대로) + 브릿지 + Nav2(AMCL+real_real_final 맵)
  + 초기위치 자동 고정(/initialpose) + RViz.

검증 흐름:
  ros2 launch ur3_4040_urdf realmap_sim.launch.py
  # 다른 터미널:
  python3 scripts/approach_point.py        # 0.7m 접근 노드
  # RViz Publish Point 로 목표 클릭 → 0.7m 앞에서 목표 바라보며 정지하는지 확인

핵심: 월드=맵 좌표계라, 실제에서 쓰던 좌표(1.3924,-4.0099 등)가 시뮬에서도 동일 위치.
"""
import os
import math

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 시작 포즈 (맵 좌표 = 월드 좌표). 방위 +y = yaw 90°.
START_X = 1.3924
START_Y = -4.0099
START_Z = 0.15
START_YAW = math.pi / 2.0          # +y 방향
WORLD = '/home/min/capstone/src/ur3_4040_urdf/worlds/real_room.sdf'
MAP = '/home/min/capstone/src/robot_bringup/maps/real_real_final_0604.yaml'
RVIZ = '/home/min/capstone/nav.rviz'


def generate_launch_description():
    pkg_share = get_package_share_directory('ur3_4040_urdf')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    xacro_file = os.path.join(pkg_share, 'urdf', 'full_robot.urdf.xacro')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    robot_desc = xacro.process_file(xacro_file).toxml()

    qz = math.sin(START_YAW / 2.0)
    qw = math.cos(START_YAW / 2.0)

    # 1) Gazebo (우리 실제맵 월드)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {WORLD}'}.items(),
    )

    # 2) robot_state_publisher (base_footprint→base_link→센서 TF)
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
    )

    # 3) 로봇 spawn — 맵 좌표 그대로, yaw 90°
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', 'L_AND_F', '-string', robot_desc,
                   '-x', str(START_X), '-y', str(START_Y), '-z', str(START_Z),
                   '-Y', str(START_YAW)],
    )

    # 4) ROS↔GZ 브릿지 (gazebo.launch.py 와 동일, 단 잘못된 static TF 는 뺌)
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/model/L_AND_F/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/model/L_AND_F/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        remappings=[('/model/L_AND_F/cmd_vel', '/cmd_vel'),
                    ('/model/L_AND_F/tf', '/tf')],
        parameters=[{'use_sim_time': True}],
    )

    # 5) Nav2 Localization (map_server + AMCL), 8초 뒤 (Gazebo 안정화 후)
    localization = TimerAction(period=8.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')),
            launch_arguments={'map': MAP, 'params_file': nav2_params,
                              'use_sim_time': 'true', 'autostart': 'true'}.items(),
        )])

    # 6) Nav2 Navigation, 13초 뒤
    navigation = TimerAction(period=13.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={'params_file': nav2_params,
                              'use_sim_time': 'true', 'autostart': 'true'}.items(),
        )])

    # 초기위치는 nav2_params.yaml 의 amcl set_initial_pose(1.3924,-4.0099,yaw90) 로 확정.
    # spawn 좌표와 동일해야 함 — 둘 다 START_* 기준. (타이밍 의존 /initialpose publish 제거)

    # RViz
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', RVIZ],
        parameters=[{'use_sim_time': True}],
    )

    # 7) lidar_approach (Nav2 뜬 뒤 18초) — 시뮬도 grab_mission 처럼 한 런치에 포함.
    #    시뮬이라 use_sim_time:=true 필수.
    approach = TimerAction(period=18.0, actions=[ExecuteProcess(
        cmd=['python3', '/home/min/capstone/scripts/lidar_approach.py',
             '--ros-args', '-p', 'use_sim_time:=true', '-p', 'stop_distance:=0.45'],
        output='screen', name='lidar_approach')])

    return LaunchDescription([
        gazebo, rsp, spawn, bridge,
        localization, navigation, rviz, approach,
    ])
