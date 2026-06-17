"""arm_full.launch.py — 로봇팔 전체 스택 한 방에 (도메인 77).

포함:
  1) UR 드라이버 (ur_robot_driver/ur_control)         — joint_states + 컨트롤러
  2) RealSense (aligned depth + ordered pointcloud)    — publish_tf:=false
  3) move_group (ur3_moveit_config_custom)             — ★ gripper_tcp 모델 + KDL50ms + planning_scene_monitor
  4) static TF (gripper_link / gripper_tcp / camera)   — plain-UR RSP 에 빠진 고정 프레임 보충
  5) 팔 노드 (arm/pointcloud_all_test.py)              — venv python
  6) 도메인 브릿지 (scripts/arm_domain_bridge.py)       — 28<->77 핸드셰이크 중계

설계 메모 (2026-06-17):
  - generic `ur_moveit_config`/`ur_moveit_octomap` 은 쓰지 않는다(plain-UR RSP·-27 주범).
  - 드라이버 RSP 는 plain UR(gripper_tcp 없음) → URDF 합치는 위험 대신, 빠진 고정 프레임만
    static_transform_publisher 로 얹어 완전한 TF 트리를 만든다(중복 RSP 없음).
  - static TF 값 출처: src/ur3_control/urdf/ur3_robot.urdf.xacro
  - 반드시 scripts/run_arm_full.sh 로 실행(ROS·install·venv 소싱 + 도메인 77).
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

HOME = os.path.expanduser("~")
CAP = os.path.join(HOME, "capstone")


def _stf(name, x, y, z, roll, pitch, yaw, parent, child):
    """고정 변환 static_transform_publisher 노드."""
    return Node(
        package="tf2_ros", executable="static_transform_publisher", name=name,
        arguments=["--x", str(x), "--y", str(y), "--z", str(z),
                   "--roll", str(roll), "--pitch", str(pitch), "--yaw", str(yaw),
                   "--frame-id", parent, "--child-frame-id", child],
        output="log")


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    reverse_ip = LaunchConfiguration("reverse_ip")

    ur_driver = get_package_share_directory("ur_robot_driver")
    realsense = get_package_share_directory("realsense2_camera")
    moveit_custom = get_package_share_directory("ur3_moveit_config_custom")

    decls = [
        # ★ 실제 로봇팔 = UR3e (2026-06-17). 단 move_group/URDF/해석IK 는 아직 UR3 기준이라
        #   드라이버(ur3e)와 기구학 불일치가 남아 있음 → 아래 정합성 메모 참고(별도 마이그레이션 필요).
        DeclareLaunchArgument("ur_type", default_value="ur3e"),
        DeclareLaunchArgument("robot_ip", default_value="192.168.1.101"),
        DeclareLaunchArgument("reverse_ip", default_value="192.168.1.102"),
        # 팔 스택은 도메인 77 (nav=28 과 분리). 브릿지가 28<->77 중계.
        SetEnvironmentVariable("ROS_DOMAIN_ID", "77"),
    ]

    # 1) UR 드라이버
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_driver, "launch", "ur_control.launch.py")),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "reverse_ip": reverse_ip,
            "launch_rviz": "false",
        }.items())

    # 2) RealSense (aligned depth + ordered cloud, 카메라 자체 TF 끔)
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(realsense, "launch", "rs_launch.py")),
        launch_arguments={
            "align_depth.enable": "true",
            "pointcloud.enable": "true",
            "pointcloud.ordered_pc": "true",
            "rgb_camera.color_profile": "848x480x30",
            "depth_module.depth_profile": "848x480x30",
            "publish_tf": "false",
        }.items())

    # 3) move_group (custom — gripper_tcp 모델)
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_custom, "launch", "move_group.launch.py")))

    # 4) 빠진 고정 프레임 static TF (ur3_robot.urdf.xacro 값)
    static_tfs = [
        _stf("stf_tool0_gripper", 0, 0, 0.0678750000000002, 0, -1.5707963267949, 3.14159265358979,
             "tool0", "gripper_link"),
        _stf("stf_gripper_tcp", 0.107175, 0, 0, 0, 0, 0,
             "gripper_link", "gripper_tcp"),
        _stf("stf_camera_link", -0.00842500000000146, -0.00500000000000025, 0.0386999998335454, 0, 0, 0,
             "gripper_link", "camera_link"),
        _stf("stf_camera_color", 0, 0, 0, -1.5708, 0, -1.5708,
             "camera_link", "camera_color_optical_frame"),
        _stf("stf_camera_depth", 0, 0, 0, 0, 0, 0,
             "camera_color_optical_frame", "camera_depth_optical_frame"),
    ]

    # 5) 팔 노드 (venv python 은 래퍼가 PATH 에 올림). cwd=capstone (YOLO 모델 상대경로)
    arm_node = ExecuteProcess(
        cmd=["python3", os.path.join(CAP, "arm", "pointcloud_all_test.py")],
        cwd=CAP, output="screen")

    # 6) 도메인 브릿지 (자체 28/77 컨텍스트 생성 → 도메인 무관)
    bridge = ExecuteProcess(
        cmd=["python3", os.path.join(CAP, "scripts", "arm_domain_bridge.py")],
        output="screen")

    # 드라이버/카메라/move_group 가 뜬 뒤 팔 노드 시작 (TF/계획 준비 대기)
    delayed_arm = TimerAction(period=12.0, actions=[arm_node])

    return LaunchDescription(
        decls + [driver, camera, move_group] + static_tfs + [bridge, delayed_arm])
