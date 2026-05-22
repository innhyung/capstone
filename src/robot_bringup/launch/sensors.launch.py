"""
sensors.launch.py  —  센서 브링업 (라즈베리파이에서 실행)

라즈베리파이에 연결된 라이다 / IMU 를 띄워 PC로 토픽을 전달한다.
PC 의 mapping.launch.py / navigation.launch.py 가 여기서 나오는 /scan, /imu/data 를 소비한다.

전제:
- 이 워크스페이스(capstone)가 라즈베리파이에도 빌드되어 있어야 한다.
- ROS_DOMAIN_ID 가 PC 와 동일해야 토픽이 보인다.
- IMU(BNO055)는 I2C 연결. bno055_driver 는 adafruit-circuitpython-bno055 +
  adafruit-blinka(board) 파이썬 패키지가 설치돼 있어야 동작한다.

사용 예:
- 라이다+IMU 둘 다      : ros2 launch robot_bringup sensors.launch.py
- IMU 만 (라이다 없을 때): ros2 launch robot_bringup sensors.launch.py lidar:=false

발행 토픽:
- /scan      (sensor_msgs/LaserScan, frame=lidar_link)
- /imu/data  (sensor_msgs/Imu,       frame=imu_link)
- /imu/mag   (sensor_msgs/MagneticField)

주의 — TF:
- base_link→lidar_link / imu_link TF 는 PC 의 robot_state_publisher(URDF)가 발행하므로
  여기서 static_transform_publisher 를 따로 띄우지 않는다.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg


def generate_launch_description():
    ydlidar_share = get_package_share_directory('ydlidar_ros2_driver')

    # ---- launch 인자 ----
    use_lidar = LaunchConfiguration('lidar')
    use_imu = LaunchConfiguration('imu')
    params_file = LaunchConfiguration('params_file')

    declare_lidar = DeclareLaunchArgument(
        'lidar', default_value='true',
        description='YDLidar 드라이버 실행 여부. 라이다가 아직 없으면 false.')
    declare_imu = DeclareLaunchArgument(
        'imu', default_value='true',
        description='BNO055 IMU 드라이버 실행 여부.')
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(ydlidar_share, 'params', 'ydlidar.yaml'),
        description='YDLidar 파라미터 yaml (frame_id 는 lidar_link 로 설정됨).')

    # ----------------------------------------------------------------------
    # YDLidar — lifecycle 노드
    # ----------------------------------------------------------------------
    ydlidar_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        namespace='/',
        output='screen',
        emulate_tty=True,
        parameters=[params_file],
        condition=IfCondition(use_lidar),
    )

    # unconfigured → configure
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(ydlidar_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(use_lidar),
    )

    # inactive 상태가 되면 자동으로 activate (그래야 /scan 이 나온다)
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=ydlidar_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(ydlidar_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        ),
        condition=IfCondition(use_lidar),
    )

    # ----------------------------------------------------------------------
    # BNO055 IMU — 사용자 작성 Python 드라이버 (adafruit_bno055 사용).
    # sensor_msgs/Imu 를 /imu/data, MagneticField 를 /imu/mag 로 발행.
    # i2c_address 0x29 = BNO055 ADR 핀 HIGH (드라이버 기본값).
    # ----------------------------------------------------------------------
    imu_node = Node(
        package='bno055_driver',
        executable='bno055_node',
        name='bno055_node',
        output='screen',
        parameters=[{
            'i2c_address': 0x29,
            'frame_id': 'imu_link',
            'publish_rate_hz': 50.0,
        }],
        condition=IfCondition(use_imu),
    )

    return LaunchDescription([
        declare_lidar,
        declare_imu,
        declare_params,
        activate_event,   # configure 보다 먼저 등록되어야 inactive 전이를 잡는다
        ydlidar_node,
        configure_event,
        imu_node,
    ])
