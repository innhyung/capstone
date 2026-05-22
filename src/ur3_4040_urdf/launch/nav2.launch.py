import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. 천재님의 패키지 경로 찾기
    my_pkg_dir = get_package_share_directory('ur3_4040_urdf')

    # 2. 맵 파일과 파라미터 파일 경로 지정
    map_file = os.path.join(my_pkg_dir, 'maps', 'my_capstone_map.yaml')
    params_file = os.path.join(my_pkg_dir, 'config', 'nav2_params.yaml')

    # # 3. Nav2 기본 런치 파일을 불러오면서 내 설정 파일 덮어쓰기
    # nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    # bringup_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
    #     ),
    #     launch_arguments={
    #         'map': map_file,
    #         'params_file': params_file,
    #         'use_sim_time': 'true'  # Gazebo 시뮬레이션용 (실제 로봇 구동 시에는 'false'로 변경!)
    #     }.items()
    # )

    # 3. Nav2 길 찾기(내비게이션) 기능만 순수하게 불러오기
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            # ⭐️ 수정 1: bringup_launch.py -> navigation_launch.py 로 변경!
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            # ⭐️ 수정 2: SLAM이 맵을 줄 것이므로 기존 map 인자는 삭제(또는 주석 처리)!
            # 'map': map_file, 
            'params_file': params_file,
            'use_sim_time': True
        }.items()
    )

    return LaunchDescription([
        bringup_launch
    ])