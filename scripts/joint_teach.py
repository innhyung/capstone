#!/usr/bin/env python3
"""
joint_teach.py — 관절값 티칭 & 스무스 재생 (도메인 77, grasp 노드와 별개).

쓰는 법:
  1) 팔을 freedrive(티치모드)로 두고 손으로 원하는 자세로 옮긴다.
  2) 자세마다 record 명령 → 현재 관절값 기록. (여러 자세 반복)
  3) replay 명령 → 기록한 점들을 '한 궤적'으로 묶어 부드럽게(점에서 안 멈추고) 통과.

실행:
  source /opt/ros/jazzy/setup.bash && python3 ~/capstone/scripts/joint_teach.py

명령(다른 터미널, 도메인 77):
  ros2 topic pub --once /joint_teach std_msgs/msg/String "{data: record}"   # 현재 자세 1점 기록
  ros2 topic pub --once /joint_teach std_msgs/msg/String "{data: replay}"   # 스무스 재생
  ros2 topic pub --once /joint_teach std_msgs/msg/String "{data: clear}"    # 기록 비우기
  ros2 topic pub --once /joint_teach std_msgs/msg/String "{data: list}"     # 기록 출력
  ros2 topic pub --once /joint_teach std_msgs/msg/String "{data: save}"     # 파일 저장
  ros2 topic pub --once /joint_teach std_msgs/msg/String "{data: load}"     # 파일 불러오기
  # replay 속도 조절: "{data: 'replay 0.3'}" (max 관절속도 rad/s, 작을수록 느림)

★ 스무스 원리: 모든 점을 한 JointTrajectory.points 로 보내고, 중간점 velocities 를
   중앙차분으로 채워 컨트롤러가 점마다 멈추지 않고(구분동작 X) cubic 보간해 통과.
"""
import os
os.environ['ROS_DOMAIN_ID'] = os.environ.get('ARM_DOMAIN', '77')

import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_ORDER = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
TRAJ_TOPIC = "/scaled_joint_trajectory_controller/joint_trajectory"
SAVE_PATH = os.path.expanduser("~/capstone/bags/joint_teach_waypoints.json")


class JointTeach(Node):
    def __init__(self):
        super().__init__('joint_teach')
        self.cur = None                  # 최신 관절값(JOINT_ORDER 순서)
        self.waypoints = []              # 기록한 점들
        self.max_vel = 0.4               # replay 기본 최대 관절속도(rad/s) — 작을수록 느리고 안전
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)
        self.create_subscription(String, '/joint_teach', self.on_cmd, 10)
        self.traj_pub = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)
        self.get_logger().info(
            f"joint_teach 시작. /joint_teach 로 record/replay/clear/list/save/load 명령. "
            f"재생토픽={TRAJ_TOPIC}")

    def on_js(self, m):
        try:
            d = dict(zip(m.name, m.position))
            self.cur = [float(d[j]) for j in JOINT_ORDER]
        except Exception:
            pass

    def on_cmd(self, m):
        parts = m.data.strip().split()
        cmd = parts[0] if parts else ''
        if cmd == 'record':
            if self.cur is None:
                self.get_logger().warn("record 실패: /joint_states 아직 없음"); return
            self.waypoints.append(list(self.cur))
            self.get_logger().info(
                f"기록 {len(self.waypoints)}점: "
                + ", ".join(f"{v:+.3f}" for v in self.cur))
        elif cmd == 'clear':
            self.waypoints = []
            self.get_logger().info("기록 비움")
        elif cmd == 'list':
            self.get_logger().info(f"총 {len(self.waypoints)}점:")
            for i, wp in enumerate(self.waypoints):
                self.get_logger().info(f"  [{i}] " + ", ".join(f"{v:+.3f}" for v in wp))
        elif cmd == 'save':
            try:
                with open(SAVE_PATH, 'w') as f:
                    json.dump(self.waypoints, f)
                self.get_logger().info(f"{len(self.waypoints)}점 저장 → {SAVE_PATH}")
            except Exception as e:
                self.get_logger().warn(f"저장 실패: {e}")
        elif cmd == 'load':
            try:
                with open(SAVE_PATH) as f:
                    self.waypoints = json.load(f)
                self.get_logger().info(f"{len(self.waypoints)}점 불러옴 ← {SAVE_PATH}")
            except Exception as e:
                self.get_logger().warn(f"불러오기 실패: {e}")
        elif cmd == 'replay':
            mv = self.max_vel
            if len(parts) > 1:
                try: mv = float(parts[1])
                except Exception: pass
            self.replay(mv)
        else:
            self.get_logger().warn(f"알 수 없는 명령: '{m.data}' (record/replay/clear/list/save/load)")

    def replay(self, max_vel):
        if len(self.waypoints) < 2:
            self.get_logger().warn(f"재생 실패: 점 {len(self.waypoints)}개(2개 이상 필요)")
            return
        # 현재 자세를 시작점으로 추가(현 위치→첫 기록점도 부드럽게)
        wps = [list(self.cur)] + [list(w) for w in self.waypoints] if self.cur is not None \
            else [list(w) for w in self.waypoints]

        n = len(wps)
        # 1) 시간 배정: 구간별 최대 관절이동 / max_vel (최소 1.0s 보장)
        times = [0.0]
        for i in range(1, n):
            dmax = max(abs(wps[i][j] - wps[i - 1][j]) for j in range(6))
            seg = max(1.0, dmax / max(1e-3, max_vel))
            times.append(times[-1] + seg)

        # 2) 중간점 속도 = 중앙차분(점에서 안 멈추고 통과), 양끝은 0
        vels = []
        for i in range(n):
            if i == 0 or i == n - 1:
                vels.append([0.0] * 6)
            else:
                dt = times[i + 1] - times[i - 1]
                vels.append([(wps[i + 1][j] - wps[i - 1][j]) / dt for j in range(6)])

        traj = JointTrajectory()
        traj.joint_names = JOINT_ORDER
        traj.header.stamp = self.get_clock().now().to_msg()
        for i in range(n):
            p = JointTrajectoryPoint()
            p.positions = wps[i]
            p.velocities = vels[i]          # ★ 속도 채워 cubic 보간 → 스무스 통과
            t = times[i]
            p.time_from_start.sec = int(t)
            p.time_from_start.nanosec = int((t - int(t)) * 1e9)
            traj.points.append(p)
        self.traj_pub.publish(traj)
        self.get_logger().info(
            f"스무스 재생: {n}점(현재→기록{len(self.waypoints)}), 총 {times[-1]:.1f}s, max_vel={max_vel:.2f}rad/s")


def main():
    from rclpy.executors import ExternalShutdownException
    rclpy.init()
    node = JointTeach()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
