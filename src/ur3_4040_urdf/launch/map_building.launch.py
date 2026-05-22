import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_share = get_package_share_directory('ur3_4040_urdf')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    explore_lite_dir = get_package_share_directory('explore_lite')

    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # 매핑 시 explore_lite는 frontier 도달만 신경쓰면 됨. yaw 일치 강제하면
    # frontier blacklist 쌓여 매핑 일찍 종료. yaw_goal_tolerance를 매핑 동안만 3.14 (=180°)로 override.
    mapping_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={'yaw_goal_tolerance': '3.14'},
        convert_types=True,
    )

    sim_time_arg = {'use_sim_time': 'true'}.items()

    # 1. Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
        )
    )

    # 1b. EKF 임시 제거 (yaml 설정 문제로 시작 직후 crash)
    # 다음 단계로 디버깅. 현재는 wheel diff_drive 의 publish_odom_tf=true 로 TF 발행

    # 2. SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments=sim_time_arg
    )

    # 3. Nav2 (자체 yaml 사용 + RewrittenYaml로 매핑 전용 override 적용)
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

    # 5. SLAM 매핑 부진 방지 (yaml로 못 바꾸는 슬램 파라미터만 런타임 set)
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

    # 6. 자동 회전 부트스트랩
    # 매핑 안 된 시작 시점에 robot 주변이 unknown 이라 explore가 frontier 못 찾고 dead lock.
    # Gazebo 직접 명령으로 한 바퀴 회전시켜 lidar 스캔이 360도 들어오게 한다.
    rotate_bootstrap = TimerAction(
        period=14.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c',
                     'echo "=== auto-rotate 14s for mapping bootstrap ==="; '
                     'end=$(($(date +%s) + 14)); '
                     'while [ $(date +%s) -lt $end ]; do '
                     '  gz topic -t /model/L_AND_F/cmd_vel -m gz.msgs.Twist -p "angular: {z: 0.5}" > /dev/null 2>&1; '
                     '  sleep 0.1; '
                     'done; '
                     'gz topic -t /model/L_AND_F/cmd_vel -m gz.msgs.Twist -p "linear: {x: 0}, angular: {z: 0}" > /dev/null 2>&1; '
                     'echo "=== rotate done ==="'],
                output='screen'
            )
        ]
    )

    # 7. Explore Lite (회전 부트스트랩 끝난 후 시작 + nav2 lifecycle fully active 보장)
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
        tune_slam,
        rotate_bootstrap,
        explore_launch,
    ])
