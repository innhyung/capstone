"""
navigation.launch.py — 자율주행 통합 launch (PC 한 곳에서 ros2 launch 한 번이면 끝)

저장된 맵을 불러와 AMCL 로 위치추정 + Nav2 로 자율주행.
mapping.launch.py 와 동일한 자동화 구조 — 한 번 실행하면 라파 센서까지 다 뜸.

자동으로 처리:
  1) (옵션) ESP32 펌웨어 업로드 — 기본 OFF. upload 필요할 때만 skip_upload:=false.
  2) 라즈베리파이 SSH 접속 → sensors.launch.py 원격 기동 (라이다 + IMU)
  3) PC 로컬 노드: robot_state_publisher, joint_state_publisher, serial_bridge, ekf
  4) Nav2 Localization (3초 후) → Navigation (10초 후) 단계적 기동
  5) RViz2 자동 실행
  6) 15초 뒤 헬스체크: /scan, /imu/data, /odom, /map 들어오는지 표시

사용자가 별도 터미널에서 띄워야 할 것:
  - 없음. RViz2 가 자동으로 뜸. RViz 에서:
    * "2D Pose Estimate" 로 초기 위치를 시각적으로 클릭·드래그
    * "Nav2 Goal" 로 목표 지점 클릭 → 자율주행

전제:
  - PC, 라파 양쪽 모두 ROS_DOMAIN_ID=28, capstone 워크스페이스 빌드 완료
  - 라파 SSH 키 인증 설정 완료
  - ESP32 가 PC USB 에 연결됨 (udev 로 /dev/esp32 심볼릭 자동 생성)
  - mapping.launch.py 로 매핑 후 map_saver_cli 로 저장한 맵이 있음

옵션 인자:
  map:=/경로/맵.yaml               # 다른 맵 사용
  port:=/dev/ttyUSB1               # udev 안 쓸 때 ESP32 USB 포트 직접 지정
  rpi_host:=danial@<IP>            # 라파 호스트 바뀌었을 때
  skip_upload:=false               # 펌웨어 업로드 실행 (기본은 생략 — 모터 안전)
  skip_rpi:=true                   # 라파 SSH 건너뛰기 (PC 단독 디버그)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# ---- 환경 고정값 ----
PIO_VENV = '/home/min/.platformio/penv/bin/activate'
FIRMWARE_DIR = '/home/min/capstone/firmware/esp32_pid'
ROS_SETUP_PC = '/opt/ros/jazzy/setup.bash'
WS_SETUP_PC = '/home/min/capstone/install/setup.bash'
DOMAIN_ID = '28'


def generate_launch_description():
    pkg_urdf = get_package_share_directory('ur3_4040_urdf')
    pkg_bringup = get_package_share_directory('robot_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    xacro_file = os.path.join(pkg_urdf, 'urdf', 'full_robot.urdf.xacro')
    nav2_params_file = os.path.join(pkg_urdf, 'config', 'nav2_params.yaml')
    ekf_params = os.path.join(pkg_urdf, 'config', 'ekf.yaml')
    default_map = os.path.join(pkg_bringup, 'maps', 'my_real_map.yaml')

    # ---- launch 인자 ----
    port = LaunchConfiguration('port')
    rpi_host = LaunchConfiguration('rpi_host')
    skip_upload = LaunchConfiguration('skip_upload')
    skip_rpi = LaunchConfiguration('skip_rpi')
    map_file = LaunchConfiguration('map')

    declare_port = DeclareLaunchArgument(
        'port', default_value='/dev/esp32',
        description='ESP32 시리얼 포트. udev 규칙으로 /dev/esp32 가 항상 ESP32를 가리킴.')
    declare_rpi = DeclareLaunchArgument(
        'rpi_host', default_value='danial@10.42.0.78',
        description='라즈베리파이 SSH 대상 (user@ip).')
    declare_skip_upload = DeclareLaunchArgument(
        'skip_upload', default_value='false',
        description='기본 false (매 launch 마다 펌웨어 자동 업로드). 펌웨어와 launch param 일치 보장. '
                    '⚠️ 업로드 중 ESP32 부트로더 단계에서 GPIO 가 떠 모터가 잠깐 구동될 수 있으니 '
                    '반드시 모터 전원 OFF 상태에서 launch 시작할 것.')
    declare_skip_rpi = DeclareLaunchArgument(
        'skip_rpi', default_value='false',
        description='true 면 라파 SSH 단계 건너뜀 (PC 단독 디버그용).')
    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='네비게이션에 사용할 맵 yaml 경로')

    # ---- 1) ESP32 펌웨어 빌드 + 업로드 (옵션) ----
    pio_upload = ExecuteProcess(
        name='esp32_upload',
        cmd=[
            'bash', '-c',
            f'echo "[bringup] ▶ ESP32 펌웨어 빌드 + 업로드 ..." && '
            f'source {PIO_VENV} && '
            f'cd {FIRMWARE_DIR} && '
            f'pio run -t upload 2>&1 | tail -8 && '
            f'echo "[bringup] ✅ ESP32 펌웨어 업로드 완료"'
        ],
        output='screen',
        condition=UnlessCondition(skip_upload),
    )

    # ---- 2) 라즈베리파이 SSH 로 sensors 기동 ----
    rpi_sensors = ExecuteProcess(
        name='rpi_sensors_ssh',
        cmd=[
            'ssh', '-tt',
            '-o', 'StrictHostKeyChecking=accept-new',
            '-o', 'ServerAliveInterval=10',
            '-o', 'ServerAliveCountMax=3',
            rpi_host,
            f'export ROS_DOMAIN_ID={DOMAIN_ID} && '
            'source /opt/ros/jazzy/setup.bash && '
            'source ~/capstone/install/setup.bash && '
            'echo "[rpi] sensors.launch.py 기동 ..." && '
            'ros2 launch robot_bringup sensors.launch.py'
        ],
        output='screen',
        condition=UnlessCondition(skip_rpi),
    )

    # ---- 3) PC 로컬 노드 ----
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
            'wheelbase': 0.58,
            'ticks_per_rev': 11400.0,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            # EKF 가 odom→base_footprint TF 를 발행하므로 여긴 끔.
            'publish_tf': False,
        }],
    )

    # robot_localization EKF — 휠 오도메트리(/odom) + IMU(/imu/data) 융합.
    # 출력: /odometry/filtered 토픽 + odom→base_footprint TF.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': False}],
    )

    # ---- 4) 펌웨어 업로드가 끝난 뒤에 serial_bridge 시작 ----
    serial_after_upload = RegisterEventHandler(
        OnProcessExit(
            target_action=pio_upload,
            on_exit=[serial_bridge],
        ),
        condition=UnlessCondition(skip_upload),
    )
    serial_immediate = TimerAction(
        period=0.0,
        actions=[serial_bridge],
        condition=IfCondition(skip_upload),
    )

    # ---- 5) Nav2 Localization (3초 후) ----
    # map_server + AMCL — map→odom TF 발행
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

    # ---- 6) Nav2 Navigation (10초 후) ----
    # planner / controller / behavior / bt_navigator
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

    # ---- 7) RViz2 ----
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # ---- 8) 헬스체크 (15초 뒤) ----
    healthcheck = TimerAction(
        period=15.0,
        actions=[ExecuteProcess(
            name='healthcheck',
            cmd=['bash', '-c',
                 f'source {ROS_SETUP_PC} && source {WS_SETUP_PC} && '
                 f'export ROS_DOMAIN_ID={DOMAIN_ID} && '
                 'echo ""; '
                 'echo "═══════════════ 내비게이션 시스템 헬스체크 ═══════════════"; '
                 'for t in /scan /imu/data /odom /map /odometry/filtered; do '
                 '  if timeout 4 ros2 topic echo --once "$t" >/dev/null 2>&1; then '
                 '    printf "  \\033[32m✅\\033[0m %-20s 정상\\n" "$t"; '
                 '  else '
                 '    printf "  \\033[31m❌\\033[0m %-20s 메시지 안 들어옴\\n" "$t"; '
                 '  fi; '
                 'done; '
                 'echo "─────────────────────────────────────────────────────"; '
                 'echo "🟢 RViz 에서:"; '
                 'echo "   1. Fixed Frame: map"; '
                 'echo "   2. \\"2D Pose Estimate\\" 로 로봇 실제 위치 클릭·드래그"; '
                 'echo "   3. \\"Nav2 Goal\\" 로 목표 지점 클릭 → 자율주행"; '
                 'echo "═════════════════════════════════════════════════════"; '
            ],
            output='screen',
        )],
    )

    return LaunchDescription([
        declare_port, declare_rpi, declare_skip_upload, declare_skip_rpi, declare_map,
        LogInfo(msg='[bringup] navigation 통합 launch 시작'),
        pio_upload,
        serial_after_upload,
        serial_immediate,
        rpi_sensors,
        robot_state_publisher,
        joint_state_publisher,
        ekf_node,
        localization,
        navigation,
        rviz,
        healthcheck,
    ])
