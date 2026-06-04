"""
offline_slam.launch.py — rosbag 재생용 오프라인 SLAM (로봇 없이 집에서 튜닝).

실물에서 한 번 녹화한 bag 을 재생하면서, EKF + slam_toolbox 를 use_sim_time 으로
새로 돌려 매핑을 재현한다. slam_real.yaml 파라미터를 바꿔가며 결과를 무한 반복
비교할 수 있다 (로봇을 다시 끌고 다닐 필요 없음).

이 launch 에는 serial_bridge / 펌웨어 업로드 / 라즈베리파이 SSH 가 없다 (실물 전용 제거).
필요한 입력은 모두 bag 에서 온다.

--- 녹화 (실물에서 매핑 주행하며, 한 번만) ---
  ros2 bag record /scan /odom /imu/data /tf_static -o mapping_run1
  (raw 센서 + 고정TF 만. 동적 /tf 는 제외 — 여기서 EKF/slam 을 새로 돌리므로 충돌 방지)

--- 재생 + 재-SLAM (집에서, 반복) ---
  # 터미널 A
  ros2 launch robot_bringup offline_slam.launch.py
  # 터미널 B (--clock 필수: bag 의 시간을 sim time 으로 공급)
  ros2 bag play mapping_run1 --clock

  파라미터 바꾸려면: 둘 다 끄고 → slam_real.yaml 수정 → 다시 A, B 실행.

--- 맵 저장 ---
  ros2 run nav2_map_server map_saver_cli -f ~/capstone/src/robot_bringup/maps/clean_map
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# 실물 launch 와 동일한 rviz 설정 (없으면 --rviz false 로 끄거나 경로 변경)
RVIZ_CONFIG = '/home/min/capstone/src/rviz_settings.rviz'


def generate_launch_description():
    pkg_urdf = get_package_share_directory('ur3_4040_urdf')
    pkg_bringup = get_package_share_directory('robot_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    xacro_file = os.path.join(pkg_urdf, 'urdf', 'full_robot.urdf.xacro')
    slam_params = os.path.join(pkg_bringup, 'config', 'slam_real.yaml')
    ekf_params = os.path.join(pkg_urdf, 'config', 'ekf.yaml')

    # use_sim_time 은 오프라인 재생이라 기본 true (bag 의 /clock 사용)
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    use_ekf = LaunchConfiguration('use_ekf')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='bag 재생 시간을 쓰려면 true (오프라인 기본). bag play 는 --clock 으로.')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true', description='RViz 동시 실행 여부.')
    declare_use_ekf = DeclareLaunchArgument(
        'use_ekf', default_value='true',
        description='true=EKF(wheel vx + IMU vyaw) 로 odom->base TF. '
                    'false=wheel odom 만으로 odom->base TF (EKF on/off 비교용).')

    # ---- robot_state_publisher: URDF → base_footprint→...→lidar_link TF ----
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
                     'use_sim_time': use_sim_time}],
    )
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ---- odom→base_footprint TF: EKF (use_ekf=true) 또는 wheel-only (false) ----
    # use_ekf=true: bag 의 /odom(vx) + /imu/data(vyaw) 융합
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': use_sim_time}],
        condition=IfCondition(use_ekf),
    )
    # use_ekf=false: bag 의 wheel /odom pose 를 그대로 TF 로 (IMU 미사용, EKF off 비교)
    odom_to_tf = ExecuteProcess(
        name='odom_to_tf',
        cmd=['python3', '/home/min/capstone/scripts/odom_to_tf.py',
             '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen',
        condition=UnlessCondition(use_ekf),
    )

    # ---- slam_toolbox: bag 의 /scan + TF → map ----
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params,
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', RVIZ_CONFIG],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz,
        declare_use_ekf,
        robot_state_publisher,
        joint_state_publisher,
        ekf_node,
        odom_to_tf,
        slam,
        rviz_node,
    ])
