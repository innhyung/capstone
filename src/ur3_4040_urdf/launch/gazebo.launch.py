from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro # <--- Xacro 변환을 위해 필수!

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_share = get_package_share_directory('ur3_4040_urdf')
    
    # 1. Xacro 파일 경로 설정 및 URDF 변환
    # 이제 urdf가 아니라 .xacro 파일을 가리킵니다.
    xacro_file = os.path.join(pkg_share, 'urdf', 'full_robot.urdf.xacro')
    
    # xacro 라이브러리를 사용해 파일을 조립하고 URDF 문자열로 뽑아냅니다.
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # 2. Gazebo 실행
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r "https://fuel.gazebosim.org/1.0/danml1/worlds/simple_colored_warehouse"'}.items()
    )

    # 3. 정적 변환 발행자 (base_link -> base_footprint)
    tf_footprint_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        output='screen'
    )

    # 4. Robot State Publisher (조립된 robot_desc를 뿌려줍니다)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 5. 로봇 Spawn (이제 -file 대신 -string을 써서 조립된 데이터를 넘깁니다)
    spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'L_AND_F',
            '-string', robot_desc, # <--- 파일 경로 대신 조립된 문자열을 직접 투척!
            '-x', '-3.0', '-y', '-3.0', '-z', '0.15'
        ],
        output='screen',
    )

    # 6. ROS-GZ Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # ROS cmd_vel → Gazebo robot (단방향)
            #공통으로 쓰는 토픽 이름 @ ros에서 보내는 메시지 타입 ] gz에서 받는 메시지 타입
            '/model/L_AND_F/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',

            # Gazebo → ROS (단방향)
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/model/L_AND_F/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        # ('브릿지가 원래 찾으려던 이름', '대신 주워올 이름')
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