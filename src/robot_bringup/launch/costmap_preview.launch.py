"""
costmap_preview.launch.py — 실물 로봇 없이 글로벌 코스트맵 미리보기

용도:
  저장된 맵(.pgm/.yaml) + nav2_params.yaml 만으로 글로벌 코스트맵을 생성해
  RViz 에서 확인. 충돌하는 벽 근처 inflation 이 얼마나 부풀어 있는지,
  Footprint 가 의도대로 적용됐는지 등을 실물 없이 사전 검증.

확인 가능한 것:
  - Static layer (저장된 맵의 벽/장애물)
  - Inflation layer (파라미터 적용된 안전거리 그라데이션)
  - Footprint 가 의도된 크기로 적용됐는지

확인 불가:
  - Obstacle layer (라이다 실시간 입력 없으므로 비어있음)
  - Local costmap (실시간 센서 데이터 필요)
  - AMCL 위치 추정 오류 (라이다 필요)

전제:
  - 저장된 맵 파일 (~/capstone/src/robot_bringup/maps/my_real_map.{pgm,yaml})
  - nav2_params.yaml 의 costmap 파라미터

사용법:
  ros2 launch robot_bringup costmap_preview.launch.py
  ros2 launch robot_bringup costmap_preview.launch.py map:=/경로/다른맵.yaml

  파라미터 바꿔보고 싶으면 nav2_params.yaml 수정 → 재실행.

RViz 설정 (수동):
  1. Fixed Frame: map
  2. Add → By topic → /map → Map  (정적 맵)
       └ Durability: Transient Local
  3. Add → By topic → /global_costmap/costmap → Map  (코스트맵)
       └ Durability: Transient Local
       └ Color Scheme: costmap  ⭐ 이게 핵심
  4. (선택) Add → Polygon → /global_costmap/published_footprint  (로봇 외형)

  로봇은 맵 원점에 정지 상태로 표시됨.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_urdf = get_package_share_directory('ur3_4040_urdf')
    pkg_bringup = get_package_share_directory('robot_bringup')

    xacro_file = os.path.join(pkg_urdf, 'urdf', 'full_robot.urdf.xacro')
    nav2_params_file = os.path.join(pkg_urdf, 'config', 'nav2_params.yaml')
    default_map = os.path.join(pkg_bringup, 'maps', 'my_real_map.yaml')

    # ---- launch 인자 ----
    map_file = LaunchConfiguration('map')
    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='검사할 맵 yaml 경로')

    # ---- URDF: base_footprint → base_link → lidar_link 등 정적 TF ----
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

    # ---- 가짜 TF: map → odom → base_footprint (로봇을 map 원점에 정지) ----
    # costmap 노드들이 TF 체인 필요로 함. 로봇은 (0,0)에 가만히 있다고 가정.
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )
    static_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
    )

    # ---- Map server: 저장된 .pgm/.yaml 로드 → /map 발행 ----
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': map_file,
            'topic_name': 'map',
            'frame_id': 'map',
        }],
    )

    # ---- Planner server: global costmap 생성 (nav2_params.yaml 의 설정 그대로) ----
    # 이 노드가 활성화되면 /global_costmap/costmap 토픽이 살아남.
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': False}],
    )

    # ---- Lifecycle manager: map_server + planner_server 자동 활성화 ----
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap_preview',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'planner_server'],
        }],
    )

    # ---- RViz ----
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        declare_map,
        LogInfo(msg='[costmap_preview] 글로벌 코스트맵 미리보기 시작'),
        LogInfo(msg='[costmap_preview] RViz: /global_costmap/costmap 추가 + Color Scheme=costmap'),
        robot_state_publisher,
        joint_state_publisher,
        static_map_to_odom,
        static_odom_to_base,
        map_server,
        planner_server,
        lifecycle_manager,
        rviz,
    ])
