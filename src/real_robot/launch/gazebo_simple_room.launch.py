"""
gazebo_simple_room.launch.py  (real_robot 패키지)

simple_room.sdf 시뮬 환경에서 robot 을 spawn 하는 launch.
robot URDF 는 ur3_4040_urdf 패키지에서, world sdf 는 real_robot 패키지에서 로드.

robot spawn:
  - 위치 (1.0, -0.9, 0.15) — simple_room 우측 하단
  - yaw = 90° (위 방향, +Y 축)
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ur3 = get_package_share_directory('ur3_4040_urdf')   # robot URDF
    pkg_real = get_package_share_directory('real_robot')     # world sdf

    # 1. URDF (robot 모델은 ur3_4040_urdf 에서)
    xacro_file = os.path.join(pkg_ur3, 'urdf', 'full_robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # 2. Gazebo (world 는 real_robot 에서)
    world_file = os.path.join(pkg_real, 'worlds', 'simple_room.sdf')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # 3. base_link -> base_footprint static TF
    tf_footprint_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        output='screen'
    )

    # 4. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 5. Robot Spawn — simple_room 우측 하단, 위 방향(yaw=90°)
    # 방 내부 2.8 × 4.5m. 남쪽 벽(-2.25) 까지 1.25m 마진, 우측 벽(+1.4) 까지 0.6m.
    spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'L_AND_F',
            '-string', robot_desc,
            '-x', '0.8', '-y', '-1.0', '-z', '0.15',
            '-Y', '1.5708',  # yaw = 90° (위 방향)
        ],
        output='screen',
    )

    # 6. ROS-GZ Bridge (ur3_4040_urdf/launch/gazebo.launch.py 와 동일)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/L_AND_F/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/model/L_AND_F/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        remappings=[
            ('/model/L_AND_F/cmd_vel', '/cmd_vel'),
            ('/model/L_AND_F/tf', '/tf')
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        tf_footprint_base,
        robot_state_publisher,
        spawn_model,
        bridge
    ])
