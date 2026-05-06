"""
map_building_simple_room.launch.py  (real_robot 패키지)

simple_room 환경에서 SLAM + Nav2 + explore_lite 로 자율 매핑.
- gazebo include: real_robot/launch/gazebo_simple_room.launch.py
- nav2_params: ur3_4040_urdf/config/nav2_params.yaml
- 매핑 결과는 map_saver_cli 로 real_robot/maps/ 에 simple_room_map 등 원하는 이름으로 저장.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_ur3 = get_package_share_directory('ur3_4040_urdf')   # nav2_params
    pkg_real = get_package_share_directory('real_robot')     # gazebo launch
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    explore_lite_dir = get_package_share_directory('explore_lite')

    nav2_params_file = os.path.join(pkg_ur3, 'config', 'nav2_params.yaml')

    # simple_room 이 좁아서 보수적 주행 위해 inflation 강화 + 장애물 근접 시 감속.
    # 좁은 방에서 벽 가까이 가는 걸 미리 회피하도록 마진 확대.
    # padding 0.1 시도 시 통로(1.4m) 통과 못함 → 0.05 로 절반 적용 (UR3 팔 sweep 일부 보호).
    mapping_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={
            'yaw_goal_tolerance': '3.14',
            'inflation_radius': '0.4',          # 0.25 → 0.4 (벽 회피 마진 ↑)
            'footprint_padding': '0.05',        # NEW: robot 둘레 5cm 가상 마진 (UR3 일부 보호)
            'cost_scaling_dist': '1.0',         # 0.6 → 1.0 (장애물 1m 안으로 들어오면 감속 시작)
            'cost_scaling_gain': '2.5',         # 1.5 → 2.5 (감속 강도 ↑)
            'lookahead_dist': '0.3',            # 0.4 → 0.3 (carrot 더 가까이, 좁은 공간 회전 안정)
            'max_lookahead_dist': '0.45',       # 0.6 → 0.45
            'max_allowed_time_to_collision_up_to_carrot': '1.5',  # 1.0 → 1.5 (충돌 미리 검사)
            'desired_linear_vel': '0.25',       # 0.35 → 0.25 (좁은 방 천천히)
        },
        convert_types=True,
    )

    sim_time_arg = {'use_sim_time': 'true'}.items()

    # 1. Gazebo (simple_room)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_real, 'launch', 'gazebo_simple_room.launch.py')
        )
    )

    # 2. SLAM Toolbox — simple_room 전용 yaml 사용
    # 기본 yaml 의 minimum_score(0.5), minimum_travel_distance(0.5),
    # minimum_travel_heading(0.5), map_update_interval(5.0) 이 좁은 단조 방에 너무 보수적.
    # → 회전 중 scan match 거부로 매핑 갱신 안 됨.
    # 우리 yaml: minimum_score 0.2, travel_distance/heading 0.1, map_update 1.0 으로 완화.
    slam_params_file = os.path.join(pkg_real, 'config', 'slam_params_simple_room.yaml')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': slam_params_file,
        }.items()
    )

    # 3. Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': mapping_nav2_params,
        }.items()
    )

    # 4. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 4b. world → map 정적 TF
    # SLAM 의 map (0,0) = robot spawn 위치 (Gazebo world 의 0.8, -1.0, yaw=π/2).
    # 비전(CCTV) 캘리브레이션은 방 정중앙을 (0,0) 으로 잡을 거니, world frame 추가.
    # 비전 좌표(world) ↔ robot map frame 자동 변환됨.
    world_to_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map_static_tf',
        # x  y    z   yaw     pitch roll  parent child
        arguments=['0.8', '-1.0', '0', '1.5708', '0', '0', 'world', 'map'],
        output='screen'
    )

    # 5. SLAM 런타임 파라미터 보정
    tune_slam = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c',
                     'ros2 param set /slam_toolbox map_update_interval 1.0; '
                     'ros2 param set /slam_toolbox minimum_travel_distance 0.25; '
                     'ros2 param set /slam_toolbox minimum_travel_heading 0.2'],
                output='screen'
            )
        ]
    )

    # 6. 자동 회전 부트스트랩 — 원형 궤적 (linear + angular)
    # 단순 회전 시 SLAM Toolbox 가 keyframe 등록을 거부 (rotation-only 정책 의심).
    # linear.x=0.05 + angular.z=0.3 → 반경 0.17m 작은 원으로 돌면서 매 keyframe
    # 등록을 trigger. 22s × 0.05m/s = 1.1m 이동하지만 원형이라 spawn 부근에 머무름.
    rotate_bootstrap = TimerAction(
        period=14.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c',
                     'echo "=== auto-rotate 22s (circular: r≈0.17m) for mapping bootstrap ==="; '
                     'end=$(($(date +%s) + 22)); '
                     'while [ $(date +%s) -lt $end ]; do '
                     '  gz topic -t /model/L_AND_F/cmd_vel -m gz.msgs.Twist -p "linear: {x: 0.05}, angular: {z: 0.3}" > /dev/null 2>&1; '
                     '  sleep 0.1; '
                     'done; '
                     'gz topic -t /model/L_AND_F/cmd_vel -m gz.msgs.Twist -p "linear: {x: 0}, angular: {z: 0}" > /dev/null 2>&1; '
                     'echo "=== rotate done ==="'],
                output='screen'
            )
        ]
    )

    # 7. Explore Lite
    explore_launch = TimerAction(
        period=45.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(explore_lite_dir, 'launch', 'explore.launch.py')
                ),
                launch_arguments=sim_time_arg
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        slam_launch,
        nav2_launch,
        rviz_node,
        world_to_map_tf,
        tune_slam,
        rotate_bootstrap,
        explore_launch,
    ])
