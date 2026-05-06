"""
scan-only SLAM (실기, 라파에서 들어오는 /scan 만 사용)

전제:
- 라파에서 ydlidar_ros2_driver 가 /scan publish 중
- 라파에서 base_link → laser_frame static TF publish 중 (ydlidar_launch.py)
- ROS_DOMAIN_ID 양쪽 동일 (28)

이 launch 가 노트북에서 띄우는 것:
- odom → base_link static TF (가짜. base_link 정지 가정. SLAM 이 map→odom 으로 lidar 움직임 반영)
- slam_toolbox online_async (scan matching only)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('real_robot')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    slam_params = os.path.join(pkg_share, 'config', 'slam_real.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='실기에서는 false. 시뮬은 true.'
    )

    # 가짜 odom → base_link
    static_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen',
    )

    # slam_toolbox (online async)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params,
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        static_odom_to_base,
        slam_launch,
    ])
