"""
mapping.launch.py  —  실기 로봇 매핑 단계 (PC에서 실행)

전제:
- 라즈베리파이에서 sensors.launch.py 가 떠서 /scan (+ /imu) publish 중
- ESP32 모터컨트롤러가 PC에 USB 시리얼로 연결됨
- ROS_DOMAIN_ID 가 PC ↔ 라즈베리파이 동일

이 launch 가 PC에서 띄우는 것:
- robot_state_publisher : URDF 기반 TF (base_footprint→base_link→lidar_link 등)
- serial_bridge         : ESP32 ↔ ROS2. /cmd_vel 수신, /odom + odom→base_footprint TF 발행
- slam_toolbox          : /scan + /odom 으로 맵 작성 (map→odom TF 발행)
- rviz2

매핑 절차:
1) 라즈베리파이: ros2 launch robot_bringup sensors.launch.py
2) PC          : ros2 launch robot_bringup mapping.launch.py
3) PC (별도 터미널): ros2 run teleop_twist_keyboard teleop_twist_keyboard
   으로 로봇을 천천히 운전하며 방을 한 바퀴 돈다.
4) 맵 저장     : ros2 run nav2_map_server map_saver_cli -f ~/capstone/src/robot_bringup/maps/my_real_map
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_urdf = get_package_share_directory('ur3_4040_urdf')
    pkg_bringup = get_package_share_directory('robot_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    xacro_file = os.path.join(pkg_urdf, 'urdf', 'full_robot.urdf.xacro')
    slam_params = os.path.join(pkg_bringup, 'config', 'slam_real.yaml')

    # ---- launch 인자 ----
    port = LaunchConfiguration('port')
    declare_port = DeclareLaunchArgument(
        'port', default_value='/dev/ttyUSB0',
        description='ESP32 모터컨트롤러 USB 시리얼 포트 (replug 후 ttyUSB1 로 바뀔 수 있음)')

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

    # 바퀴/팔 등 non-fixed 조인트의 TF 를 채우기 위해 0 값 발행
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # ESP32 모터 베이스 브리지
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

    # SLAM Toolbox (online async) — 실기용 slam_real.yaml
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': slam_params,
        }.items(),
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
        robot_state_publisher,
        joint_state_publisher,
        serial_bridge,
        slam,
        rviz,
    ])
