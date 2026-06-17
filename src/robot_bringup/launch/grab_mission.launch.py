"""
grab_mission.launch.py — 실물 로봇 통합 미션 런치 (한 번 실행으로 모두).

navigation.launch.py (실물 브링업 + Nav2 + RViz) 를 통째로 포함하고,
거기에 lidar_approach 노드(2단계: Nav2 로 물건 근처까지 → 라이다 40cm 접근)를 더한다.
사용자는 다른 터미널에서 **목표(물건 위치)만 토픽으로** 발행하면 됨.

--- 실행 ---
  ⚠️ 모터 전원 OFF 로 시작 (펌웨어 자동 업로드). 헬스체크 ✅ 후 모터 전원 ON.
  ros2 launch robot_bringup grab_mission.launch.py

--- 목표 발행 (다른 터미널) ---
  ros2 topic pub --once /clicked_point geometry_msgs/msg/PointStamped \
    "{header: {frame_id: 'map'}, point: {x: -2.8, y: 1.3, z: 0.0}}"
  # 또는 RViz 의 "Publish Point" 툴로 클릭

동작: 목표 받으면 → [1단계] Nav2 가 벽 피해 물건 근처 staging 까지 이동 →
      [2단계] 라이다 보며 전진, 앞 stop_distance(기본 0.20m) 에서 정지 → 팔로 잡기.

전제·주의:
  - 초기위치는 nav2_params.yaml 의 amcl set_initial_pose — 로봇 놓는 자리에 맞게 수정할 것.
  - lidar_approach 는 PC 에서 실행됨 (Nav2·serial_bridge 와 같은 PC).
  - 옵션 인자: map:=<경로> / skip_upload:=true / skip_rpi:=true / stop_distance:=0.30
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


LIDAR_APPROACH = '/home/min/capstone/scripts/lidar_approach.py'


def generate_launch_description():
    pkg_bringup = get_package_share_directory('robot_bringup')
    default_map = os.path.join(pkg_bringup, 'maps', 'clean_0611.yaml')  # 6/11 재매핑 맵을 기본값으로

    map_file = LaunchConfiguration('map')
    skip_upload = LaunchConfiguration('skip_upload')
    skip_rpi = LaunchConfiguration('skip_rpi')
    stop_distance = LaunchConfiguration('stop_distance')
    standoff = LaunchConfiguration('standoff')
    wall_scan_radius = LaunchConfiguration('wall_scan_radius')
    pause_sec = LaunchConfiguration('pause_sec')
    approach_run = LaunchConfiguration('approach_run')
    back_away_dist = LaunchConfiguration('back_away_dist')
    retrace_return = LaunchConfiguration('retrace_return')

    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map, description='네비게이션 맵 yaml 경로')
    declare_skip_upload = DeclareLaunchArgument(
        'skip_upload', default_value='true',
        description='기본 true (펌웨어 업로드 건너뜀 — 펌웨어 고정됨 2026-06-15, 모터 전진 글리치 방지). 펌웨어 수정 시만 skip_upload:=false.')
    declare_skip_rpi = DeclareLaunchArgument(
        'skip_rpi', default_value='false', description='true 면 라파 SSH 건너뜀.')
    declare_stop = DeclareLaunchArgument(
        'stop_distance', default_value='0.20',
        description='[개방공간 모드] 라이다 정지 거리(m) — 앞 물체 이 거리에서 정지. '
                    'lidar_link 가 프레임 맨앞(x=0.325, 프레임보다 돌출)이라 이 값은 거의 "앞범퍼↔벽" 거리. '
                    '0.40→0.35→0.20 (2026-06-17): 복귀를 retrace(왔던길)로 해 충돌위험↓ + 팔 reach 가깝게. '
                    '물체가 팔베이스서 ~0.325m(0.2+0.125)라 UR3 reach 안.')
    declare_standoff = DeclareLaunchArgument(
        'standoff', default_value='0.50',
        description='[책상/벽 수직접근 모드] 책상 앞면~로봇 중심 정지거리(m). 맵으로 멈춰서 실물 오차 흡수 위해 여유 두기(0.6~0.7). 너무 가까우면 키울 것.')
    declare_wall_radius = DeclareLaunchArgument(
        'wall_scan_radius', default_value='0.55',
        description='목표 주변 책상/벽 면을 찾는 반경(m). 물건이 책상 위 깊이 찍히면 키워야 면을 잡음(작으면 개방공간 폴백→엉뚱한 곳). 코너 섞이면 줄일 것.')
    declare_pause = DeclareLaunchArgument(
        'pause_sec', default_value='1.5',
        description='[수직접근] 정렬(회전) 끝나고 직진 시작 전 멈춰 쉬는 시간(s). odom 드리프트 안정화.')
    declare_run = DeclareLaunchArgument(
        'approach_run', default_value='0.30',
        description='[수직접근] Nav2 가 최종보다 이만큼 뒤에 멈춤 → 그만큼 마지막에 곧장 직진(라이다 정지). 직진 구간 길이.')
    declare_back = DeclareLaunchArgument(
        'back_away_dist', default_value='0.55',
        description='파지 완료 후 복귀 전, 회전 없이 곧장 뒤로 빠질 거리(m). 책상 코앞 180° 턴어라운드 '
                    'AMCL 발산 회피용. 0.40→0.55 로 조금 더 빠진 뒤 열린 공간에서 복귀. 0이면 끔(바로 복귀). '
                    '※ retrace_return:=true(기본)면 이 값 대신 retrace 가 복귀를 담당.')
    declare_retrace = DeclareLaunchArgument(
        'retrace_return', default_value='true',
        description='복귀 방식. true(기본)=들어온 길 되짚기(역순 후진→출발서 1/3 지점서 180°→전진, 코앞·좁은통로 회전·뒤박힘 회피). '
                    'false=옛 Nav2 복귀(back_away 후진 후 새 경로). ★문제 생기면 retrace_return:=false 로 즉시 복원.')

    # 1) 실물 네비게이션 통째 포함 (RPi 센서 + serial_bridge + EKF + Nav2 + RViz)
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'map': map_file,
            'skip_upload': skip_upload,
            'skip_rpi': skip_rpi,
        }.items(),
    )

    # 2) lidar_approach — Nav2 가 다 뜬 뒤 시작 (20초 후)
    approach = TimerAction(period=20.0, actions=[ExecuteProcess(
        cmd=['python3', LIDAR_APPROACH,
             '--ros-args', '-p', ['stop_distance:=', stop_distance],
             '-p', ['standoff:=', standoff],
             '-p', ['wall_scan_radius:=', wall_scan_radius],
             '-p', ['pause_sec:=', pause_sec],
             '-p', ['approach_run:=', approach_run],
             '-p', ['back_away_dist:=', back_away_dist],
             '-p', ['retrace_return:=', retrace_return]],
        output='screen',
        name='lidar_approach')])

    return LaunchDescription([
        declare_map, declare_skip_upload, declare_skip_rpi, declare_stop, declare_standoff,
        declare_wall_radius, declare_pause, declare_run, declare_back, declare_retrace,
        navigation, approach,
    ])
