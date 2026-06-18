#!/bin/bash
# 저장된 티칭 경로 재생
# 사용법: bash scripts/run_replay.sh [파일] [속도]
# 예시:   bash scripts/run_replay.sh arm/teach_data/grasp1.json 0.5

FILE=${1:-arm/teach_data/grasp1.json}
SPEED=${2:-1.0}

cd ~/capstone
source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID=77

# 컨트롤러 활성화 (이미 active면 무시)
ros2 control switch_controllers \
    --activate scaled_joint_trajectory_controller 2>/dev/null || true

python3 arm/teach_record.py --replay "$FILE" --speed "$SPEED" --auto
