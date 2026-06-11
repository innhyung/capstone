"""
offline_localization.launch.py — rosbag 재생으로 로봇 없이 AMCL(위치추정) 재현·튜닝.

실물에서 녹화한 bag(-a 든 /scan·/odom·/imu/data 포함이든)을 재생하면서, map_server + AMCL +
EKF 를 use_sim_time 으로 새로 돌려 "복귀 중 AMCL 이 발산하는가"를 로봇 없이 반복 검증한다.

핵심 진단:
  - 오프라인에서도 AMCL 이 튄다  → 설정/맵 문제 (nav2_params 의 amcl 파라미터 튜닝)
  - 오프라인에선 멀쩡하다        → 실물 발산은 CPU 굶주림(녹화·부하) 탓 → 라이브에서 녹화 끄면 됨

이 launch 에는 serial_bridge / 펌웨어 / 라파 SSH / Nav2 주행이 없다. 입력은 모두 bag.
★ bag 의 동적 /tf(map→odom, odom→base)·/map·/amcl_pose 는 재생하지 말 것 — 여기서 새로 만든다.

--- 재생 + 재-AMCL (집에서, 반복) ---
  # 터미널 A
  ros2 launch robot_bringup offline_localization.launch.py
  # 터미널 B (--clock 필수, 충돌 토픽 제외하고 raw 만 재생)
  ros2 bag play ~/grab_0611_1822 --clock --topics /scan /odom /imu/data
  # (복귀 구간만 빨리 보려면: --start-offset 160)

  파라미터 바꾸려면: 둘 다 끄고 → nav2_params.yaml 의 amcl 수정 → 다시 A, B.

판정: RViz 에서 스캔(빨강)이 맵(검정)에 계속 붙어 있으면 정상. 복귀 구간에서 로봇이
      맵 밖으로 튀면 → 오프라인에서도 발산 = 설정 문제.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

RVIZ_CONFIG = '/home/min/capstone/nav.rviz'


def generate_launch_description():
    pkg_urdf = get_package_share_directory('ur3_4040_urdf')
    pkg_bringup = get_package_share_directory('robot_bringup')

    xacro_file = os.path.join(pkg_urdf, 'urdf', 'full_robot.urdf.xacro')
    ekf_params = os.path.join(pkg_urdf, 'config', 'ekf.yaml')
    nav2_params = os.path.join(pkg_urdf, 'config', 'nav2_params.yaml')
    default_map = os.path.join(pkg_bringup, 'maps', 'real_real_final_0604.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    rviz = LaunchConfiguration('rviz')

    declare_sim = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='bag 재생 시간 사용(오프라인 기본). bag play 는 --clock 으로.')
    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map, description='AMCL 위치추정용 맵 yaml.')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true', description='RViz 동시 실행(발산 눈으로 확인).')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_sim:=false']), value_type=str)
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}])
    jsp = Node(
        package='joint_state_publisher', executable='joint_state_publisher',
        name='joint_state_publisher', output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    # odom→base_footprint TF: bag 의 /odom(vx) + /imu/data(vyaw) 융합 (실물과 동일)
    ekf = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        output='screen', parameters=[ekf_params, {'use_sim_time': use_sim_time}])

    # map_server: 저장맵 로드
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml, 'use_sim_time': use_sim_time}])

    # AMCL: bag 의 /scan + map + odom→base → map→odom (실물과 동일한 nav2_params amcl 설정)
    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}])

    # 라이프사이클 매니저: map_server·amcl 자동 활성화
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                     'node_names': ['map_server', 'amcl']}])

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', RVIZ_CONFIG],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz))

    return LaunchDescription([
        declare_sim, declare_map, declare_rviz,
        rsp, jsp, ekf, map_server, amcl, lifecycle, rviz_node,
    ])
