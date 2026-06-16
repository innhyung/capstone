#!/usr/bin/env bash
# send_goal.sh — 목표 좌표를 lidar_approach 에 "확실히" 전달하는 손테스트 헬퍼.
#
# 왜 필요한가:
#   `ros2 topic pub --once` 는 프로세스가 수 ms 만에 죽어, DDS 연결(discovery)이
#   완성되기 전에 메시지를 쏴버려 자주 드롭된다 → "여러 번 줘야 출발" 증상.
#   여기선 잠깐 살려두고 여러 번 발행해 확실히 전달한다.
#   (lidar_approach 는 latch 라 1개만 받아도 출발 → --times 6 이면 충분, 자동 종료)
#
#   ※ 실제 카메라(비전) 노드는 미션 내내 살아있는 publisher 라 한 번만 발행해도
#     이미 연결이 끝나 있어 안 드롭된다 → 카메라엔 이 스크립트 불필요. CLI 전용.
#
# 사용법:
#   scripts/send_goal.sh 4.472 2.716     # 목표 좌표 발행
#   scripts/send_goal.sh grasp           # 파지 완료(/grasp_done) → 복귀 시작
set -e
source /opt/ros/jazzy/setup.bash
source ~/capstone/install/setup.bash

if [ "$1" = "grasp" ]; then
  echo "파지 완료 신호 발행 (/grasp_done=true) — 복귀 시작 ..."
  ros2 topic pub --times 6 -r 2 /grasp_done std_msgs/msg/Bool "{data: true}"
else
  X=${1:?"사용법: send_goal.sh X Y   (예: send_goal.sh 4.472 2.716)"}
  Y=${2:?"사용법: send_goal.sh X Y   (예: send_goal.sh 4.472 2.716)"}
  echo "목표 발행 ($X, $Y) → lidar_approach (출발할 때까지 6회 반복발행) ..."
  ros2 topic pub --times 6 -r 2 /target_bottle_position geometry_msgs/msg/PointStamped \
    "{header: {frame_id: 'map'}, point: {x: $X, y: $Y, z: 0.0}}"
fi
echo "완료."
