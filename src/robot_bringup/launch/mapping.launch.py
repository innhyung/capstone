"""
mapping.launch.py — 매핑 통합 launch (PC 한 곳에서 ros2 launch 한 번이면 끝)

자동으로 처리:
  1) (옵션) ESP32 펌웨어 빌드 + 업로드 — 기본 OFF. upload:=true 로만 켬.
     ※ 업로드 중 ESP32 는 부트로더 모드라 GPIO 가 떠 모터가 의도치 않게 구동될 수 있음.
       그래서 매 launch 마다 자동 업로드하지 않는다. 펌웨어 바꿨을 때만 켤 것.
  2) 라즈베리파이 SSH 접속 → sensors.launch.py 원격 기동 (라이다 + IMU)
  3) PC 로컬 노드: robot_state_publisher, joint_state_publisher, serial_bridge, slam_toolbox
  4) (upload 시) 업로드 끝난 뒤에야 serial_bridge 시작 (시리얼 포트 충돌 방지)
  5) 15초 뒤 헬스체크: /scan, /imu/data, /odom 들어오는지 표시

사용자가 별도 터미널에서 띄워야 할 것:
  - 텔레옵   : ros2 run teleop_twist_keyboard teleop_twist_keyboard
               --ros-args -p speed:=0.15 -p turn:=0.4
  - 시각화   : ros2 run rviz2 rviz2     (Fixed Frame: map · Map display Durability=Transient Local)

전제:
  - PC, 라파 양쪽 모두 ROS_DOMAIN_ID=28, capstone 워크스페이스 빌드 완료
  - 라파 SSH 키 인증 설정 완료 (~/.ssh/authorized_keys 에 PC 공개키 등록)
  - ESP32 가 PC 의 /dev/ttyUSB0 (또는 port:= 인자) 에 연결됨

옵션 인자:
  port:=/dev/ttyUSB1               # ESP32 USB 포트 번호 바뀌었을 때
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
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    xacro_file = os.path.join(pkg_urdf, 'urdf', 'full_robot.urdf.xacro')
    slam_params = os.path.join(pkg_bringup, 'config', 'slam_real.yaml')

    # ---- launch 인자 ----
    port = LaunchConfiguration('port')
    rpi_host = LaunchConfiguration('rpi_host')
    skip_upload = LaunchConfiguration('skip_upload')
    skip_rpi = LaunchConfiguration('skip_rpi')

    declare_port = DeclareLaunchArgument(
        'port', default_value='/dev/esp32',
        description='ESP32 시리얼 포트. udev 규칙(/etc/udev/rules.d/99-esp32.rules)으로 '
                    '/dev/esp32 심볼릭 링크가 ESP32를 항상 가리킴. '
                    '심볼릭 없으면 /dev/ttyUSB0 등으로 명시.')
    declare_rpi = DeclareLaunchArgument(
        'rpi_host', default_value='danial@10.42.0.78',
        description='라즈베리파이 SSH 대상 (user@ip).')
    declare_skip_upload = DeclareLaunchArgument(
        'skip_upload', default_value='true',
        description='기본 true (펌웨어 업로드 건너뜀 — 펌웨어 고정됨 2026-06-15, 모터 전진 글리치 방지). '
                    '펌웨어 수정했을 때만 skip_upload:=false 로 업로드(이땐 모터 전원 OFF로 시작).')
    declare_skip_rpi = DeclareLaunchArgument(
        'skip_rpi', default_value='false',
        description='true 면 라파 SSH 단계 건너뜀 (PC 단독 디버그용).')

    # ---- 1) ESP32 펌웨어 빌드 + 업로드 ----
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
    # bash -lc 가 아니라 export 를 명시적으로 — non-interactive ssh 에서 안전.
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
    # ekf.yaml 은 ur3_4040_urdf 패키지의 config/ 에 있다.
    # 출력: /odometry/filtered 토픽 + odom→base_footprint TF.
    pkg_urdf_share = get_package_share_directory('ur3_4040_urdf')
    ekf_params = os.path.join(pkg_urdf_share, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': False}],
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': slam_params,
        }.items(),
    )

    # ---- 4) 펌웨어 업로드가 끝난 뒤에 serial_bridge 시작 ----
    # 업로드 중에는 ESP32 USB 가 esptool 한테 잡혀 있으므로 미리 띄우면 실패한다.
    serial_after_upload = RegisterEventHandler(
        OnProcessExit(
            target_action=pio_upload,
            on_exit=[serial_bridge],
        ),
        condition=UnlessCondition(skip_upload),
    )
    # skip_upload=true 면 즉시 띄움 (이미 펌웨어가 올라가 있다고 가정)
    serial_immediate = TimerAction(
        period=0.0,
        actions=[serial_bridge],
        condition=IfCondition(skip_upload),
    )

    # ---- 5) 헬스체크 (15초 뒤) ----
    healthcheck = TimerAction(
        period=15.0,
        actions=[ExecuteProcess(
            name='healthcheck',
            cmd=['bash', '-c',
                 f'source {ROS_SETUP_PC} && source {WS_SETUP_PC} && '
                 f'export ROS_DOMAIN_ID={DOMAIN_ID} && '
                 'echo ""; '
                 'echo "═══════════════ 매핑 시스템 헬스체크 ═══════════════"; '
                 'for t in /scan /imu/data /odom; do '
                 '  if timeout 4 ros2 topic echo --once "$t" >/dev/null 2>&1; then '
                 '    printf "  \\033[32m✅\\033[0m %-12s 정상\\n" "$t"; '
                 '  else '
                 '    printf "  \\033[31m❌\\033[0m %-12s 메시지 안 들어옴\\n" "$t"; '
                 '  fi; '
                 'done; '
                 'echo "───────────────────────────────────────────────────"; '
                 'echo "🟢 별도 터미널에서:"; '
                 'echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard \\\\"; '
                 'echo "     --ros-args -p speed:=0.15 -p turn:=0.4"; '
                 'echo "   ros2 run rviz2 rviz2     # Fixed Frame: map (매핑 중)"; '
                 'echo "   └ Map display 의 Durability Policy 를 Transient Local 로!"; '
                 'echo "═══════════════════════════════════════════════════"; '
            ],
            output='screen',
        )],
    )

    return LaunchDescription([
        declare_port, declare_rpi, declare_skip_upload, declare_skip_rpi,
        LogInfo(msg='[bringup] mapping 통합 launch 시작'),
        pio_upload,
        serial_after_upload,
        serial_immediate,
        rpi_sensors,
        robot_state_publisher,
        joint_state_publisher,
        ekf_node,
        slam,
        healthcheck,
    ])
