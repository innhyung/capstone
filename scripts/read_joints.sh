#!/bin/bash
# read_joints.sh — 현재 UR3 관절값을 BED_GRASP_* 에 바로 붙일 수 있는 형식으로 출력.
#
# 사용법:
#   1) 펜던트(freedrive/jog)로 그리퍼를 물병 잡는 위치까지 이동
#   2) bash scripts/read_joints.sh
#   3) 출력된 6개 값을 알려주면 BED_GRASP_RIGHT/CENTER/LEFT 에 넣음
#
# /joint_states 의 이름 순서가 뒤섞여 나와도 UR3 표준 순서로 재정렬해 출력함.
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=77

python3 - <<'PY'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ORDER = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
         'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

rclpy.init()
node = Node('read_joints')
got = {}

def cb(msg):
    for n, p in zip(msg.name, msg.position):
        got[n] = p

sub = node.create_subscription(JointState, '/joint_states', cb, 50)

# 모든 관절이 들어올 때까지 잠깐 spin
import time
t_end = time.time() + 3.0
while time.time() < t_end and not all(j in got for j in ORDER):
    rclpy.spin_once(node, timeout_sec=0.1)

if not all(j in got for j in ORDER):
    print('joint_states 수신 실패 — UR 드라이버 켜져 있나요?')
else:
    vals = [got[j] for j in ORDER]
    print('\n현재 관절값 (UR3 순서):')
    print('[')
    print(f'    {vals[0]:.16g}, {vals[1]:.16g}, {vals[2]:.16g},')
    print(f'    {vals[3]:.16g}, {vals[4]:.16g}, {vals[5]:.16g},')
    print(']')

node.destroy_node()
rclpy.shutdown()
PY
