#!/usr/bin/env bash
# run_arm_full.sh — 로봇팔 전체 스택을 한 명령으로 (도메인 77).
#   드라이버 + 카메라 + move_group(custom) + static TF + 팔노드 + 브릿지 를 launch/arm_full.launch.py 로 묶어 실행.
#
# 기존 3터미널(run_arm.sh + ur_control + ur_moveit + 브릿지)을 대체.
#   ★ generic ur_moveit_config / ur_moveit_octomap 는 더 이상 쓰지 않음(plain-UR RSP·-27 주범).
#
# 사용:
#   bash scripts/run_arm_full.sh                 # ur3e 기본 (실제 로봇팔)
#   bash scripts/run_arm_full.sh ur_type:=ur3    # 굳이 ur3 로 돌릴 때
# 주의: move_group/URDF/해석IK 는 아직 UR3 기준이라 드라이버(ur3e)와 기구학(d4 ~2cm) 불일치가 남음.
#
# 전제: nav 는 평소대로 도메인 28 에서 따로 (ros2 launch robot_bringup navigation.launch.py).
#       로봇팔 전원 ON + 192.168.1.101 연결 상태에서 실행.
set -e
source /opt/ros/jazzy/setup.bash
source ~/capstone/install/setup.bash
source ~/venv/jazzy/bin/activate
export ROS_DOMAIN_ID=77    # 팔 스택 전부 77 (nav=28 과 분리). 브릿지가 28<->77 중계.
echo "[run_arm_full] ROS_DOMAIN_ID=$ROS_DOMAIN_ID — 드라이버+카메라+move_group(custom)+staticTF+팔노드+브릿지 일괄 실행"
exec ros2 launch ~/capstone/launch/arm_full.launch.py "$@"
