#!/usr/bin/env bash
# run_arm.sh — 로봇팔(pointcloud_all.py)을 nav 와 분리된 도메인(77)에서 실행.
#
# 왜: 팔 RSP 의 base_link 이 nav 의 base_link 과 같은 도메인(28)에선 충돌해 nav TF 가 깨짐.
#     도메인 77 로 띄우면 TF 완전 격리 → 충돌 없음. 도착/파지 신호는 별도 터미널에서
#     scripts/arm_domain_bridge.py 가 28<->77 중계해 줌.
#
# 실행 순서 (터미널 3개):
#   1) nav  : (평소대로, 도메인 28)  ros2 launch robot_bringup navigation.launch.py
#   2) 브릿지: source /opt/ros/jazzy/setup.bash; python3 scripts/arm_domain_bridge.py
#   3) 팔   : bash scripts/run_arm.sh   ← 이 파일
source /opt/ros/jazzy/setup.bash
source ~/capstone/install/setup.bash
source ~/venv/jazzy/bin/activate
export ROS_DOMAIN_ID=77    # ★ .bashrc 의 28 을 덮어씀 — 팔과 그 하위(UR/MoveIt/realsense) 전부 77
echo "[run_arm] ROS_DOMAIN_ID=$ROS_DOMAIN_ID (nav=28 과 분리). 브릿지가 핸드셰이크 중계."
cd ~/capstone
python3 arm/pointcloud_all.py
