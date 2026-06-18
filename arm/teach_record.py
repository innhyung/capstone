#!/usr/bin/env python3
"""
teach_record.py — UR3 티칭 녹화 / 재생  (ROS_DOMAIN_ID=77)

  녹화: python3 arm/teach_record.py --record arm/teach_data/my_pose.json
  재생: python3 arm/teach_record.py --replay arm/teach_data/my_pose.json
  재생(느리게): python3 arm/teach_record.py --replay arm/teach_data/my_pose.json --speed 0.5

녹화 흐름:
  1. 스크립트 실행
  2. Enter → 녹화 시작 (로봇 손으로 이동)
  3. Enter → 녹화 정지 & 저장

재생 흐름:
  1. 스크립트 실행
  2. Enter → scaled_joint_trajectory_controller 로 전송 & 완료 대기
"""
import argparse
import json
import os
import sys
import threading

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

UR3_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

CONTROLLER_ACTION = '/scaled_joint_trajectory_controller/follow_joint_trajectory'

RESAMPLE_INTERVAL = 0.02  # 20 ms 간격으로 리샘플

# UR3 관절 속도 한계 (rad/s) — 안전 여유 50%
UR3_MAX_VEL = [2.0, 2.0, 2.0, 3.0, 3.0, 3.0]

HOME_POSITION = [
    -1.724648300801412,   # shoulder_pan_joint
    -0.3419717115214844,  # shoulder_lift_joint
    -1.4476462602615356,  # elbow_joint
    -2.2719394169249476,  # wrist_1_joint
     1.6401886940002441,  # wrist_2_joint
    -1.6181018988238733,  # wrist_3_joint
]


# ─────────────────────────────────────────────
class TeachRecorder(Node):
    def __init__(self, output_file: str):
        super().__init__('teach_recorder')
        self.output_file = output_file
        self.recording = False
        self._records: list[dict] = []
        self._start_ns: int = 0

        self._sub = self.create_subscription(
            JointState, '/joint_states', self._cb, 200)
        self._action = ActionClient(self, FollowJointTrajectory, CONTROLLER_ACTION)
        print('[녹화] 컨트롤러 서버 대기...')
        self._action.wait_for_server(timeout_sec=10.0)
        print('[녹화] 초기 자세로 이동합니다...')

    def go_home(self, executor):
        print('[녹화] 초기 자세로 이동 중 (5초)...')
        traj = JointTrajectory()
        traj.joint_names = UR3_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = HOME_POSITION
        pt.time_from_start = Duration(sec=5, nanosec=0)
        traj.points.append(pt)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        future = self._action.send_goal_async(goal)
        executor.spin_until_future_complete(future)
        handle = future.result()
        if handle and handle.accepted:
            executor.spin_until_future_complete(handle.get_result_async())
            print('[녹화] 초기 자세 도착! Enter 누르면 녹화 시작...')
        else:
            print('[녹화] 초기 자세 이동 실패 — 그냥 진행합니다')

    def _cb(self, msg: JointState):
        if not self.recording:
            return
        # UR3 joints 만 필터 (gripper 등 제외)
        name_pos = {n: p for n, p in zip(msg.name, msg.position)}
        if not all(j in name_pos for j in UR3_JOINTS):
            return
        t = (self.get_clock().now().nanoseconds - self._start_ns) * 1e-9
        self._records.append({
            't': round(t, 4),
            'pos': [round(name_pos[j], 6) for j in UR3_JOINTS],
        })

    def start_recording(self):
        self._records.clear()
        self._start_ns = self.get_clock().now().nanoseconds
        self.recording = True
        print('[녹화] ★ 시작! 로봇 손으로 이동하세요. Enter 누르면 정지...')

    def stop_and_save(self):
        self.recording = False
        n = len(self._records)
        if n == 0:
            print('[녹화] 데이터 없음 — 저장 취소')
            return
        dur = self._records[-1]['t']
        print(f'[녹화] {n}프레임 ({dur:.2f}초) 정지. 저장 중...')
        os.makedirs(os.path.dirname(self.output_file) or '.', exist_ok=True)
        with open(self.output_file, 'w') as f:
            json.dump({'joints': UR3_JOINTS, 'frames': self._records}, f)
        print(f'[녹화] 저장 완료 → {self.output_file}')


# ─────────────────────────────────────────────
class TeachReplayer(Node):
    def __init__(self, input_file: str, speed: float):
        super().__init__('teach_replayer')
        self.speed = speed

        with open(input_file) as f:
            data = json.load(f)
        self._joints: list[str] = data['joints']
        self._frames: list[dict] = data['frames']

        self._client = ActionClient(self, FollowJointTrajectory, CONTROLLER_ACTION)
        print(f'[재생] {len(self._frames)}프레임 로드. 컨트롤러 서버 대기...')
        if not self._client.wait_for_server(timeout_sec=10.0):
            print('[재생] 컨트롤러 서버 없음 — UR 드라이버가 실행 중인지 확인하세요')
            sys.exit(1)
        print('[재생] 준비. Enter 누르면 재생...')

    def _send_traj(self, traj, executor, label):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        future = self._client.send_goal_async(goal)
        executor.spin_until_future_complete(future)
        handle = future.result()
        if not handle.accepted:
            print(f'[재생] {label} 목표 거부됨')
            return False
        result_f = handle.get_result_async()
        executor.spin_until_future_complete(result_f)
        return True

    @staticmethod
    def _smooth(frames, sigma=8):
        """관절별 가우시안 스무딩 — sigma 클수록 더 부드럽게 (단위: 샘플 수)"""
        pos = np.array([f['pos'] for f in frames], dtype=float)
        from scipy.ndimage import gaussian_filter1d
        smoothed = gaussian_filter1d(pos, sigma=sigma, axis=0)
        result = [dict(f) for f in frames]
        for i, f in enumerate(result):
            f['pos'] = smoothed[i].tolist()
        return result

    def replay(self, executor, smooth=True):
        frames = self._frames
        if not frames:
            print('[재생] 데이터 없음')
            return

        if smooth:
            frames = self._smooth(frames)
            print('[재생] 스무딩 적용 완료')

        # 1) 현재 위치 → 첫 포인트로 천천히 이동 (5초)
        print('[재생] 시작 위치로 이동 중 (5초)...')
        go_start = JointTrajectory()
        go_start.joint_names = self._joints
        pt = JointTrajectoryPoint()
        pt.positions = frames[0]['pos']
        pt.time_from_start = Duration(sec=5, nanosec=0)
        go_start.points.append(pt)
        if not self._send_traj(go_start, executor, '시작이동'):
            return
        print('[재생] 시작 위치 도착. 본 경로 재생...')

        # 20 ms 간격으로 리샘플
        sampled = [frames[0]]
        for fr in frames[1:]:
            if fr['t'] - sampled[-1]['t'] >= RESAMPLE_INTERVAL:
                sampled.append(fr)
        if sampled[-1] is not frames[-1]:
            sampled.append(frames[-1])

        # 속도 한계 기반으로 포인트 간 최소 시간 계산
        abs_times = [0.0]
        for i in range(1, len(sampled)):
            prev = sampled[i - 1]['pos']
            curr = sampled[i]['pos']
            min_dt = max(
                abs(curr[j] - prev[j]) / UR3_MAX_VEL[j]
                for j in range(len(UR3_MAX_VEL))
            )
            min_dt = max(min_dt, 0.02)  # 최소 20ms
            abs_times.append(abs_times[-1] + min_dt)

        traj = JointTrajectory()
        traj.joint_names = self._joints

        for fr, t_s in zip(sampled, abs_times):
            t_s = t_s / self.speed
            pt = JointTrajectoryPoint()
            pt.positions = fr['pos']
            pt.time_from_start = Duration(
                sec=int(t_s),
                nanosec=int((t_s % 1) * 1e9))
            traj.points.append(pt)

        total = abs_times[-1] / self.speed
        print(f'[재생] {len(sampled)}포인트, {total:.1f}초 (속도 x{self.speed}) 전송 중...')

        if self._send_traj(traj, executor, '본경로'):
            print('[재생] 완료!')


# ─────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description='UR3 티칭 녹화/재생')
    grp = ap.add_mutually_exclusive_group(required=True)
    grp.add_argument('--record', metavar='FILE', help='녹화 모드 — 저장 경로')
    grp.add_argument('--replay', metavar='FILE', help='재생 모드 — 로드 경로')
    ap.add_argument('--speed', type=float, default=1.0,
                    help='재생 속도 배율 (0.5=절반 속도, 기본 1.0)')
    ap.add_argument('--auto', action='store_true',
                    help='Enter 확인 없이 바로 실행')
    ap.add_argument('--no-smooth', action='store_true',
                    help='스무딩 비활성화 (기본: 활성)')
    args = ap.parse_args()

    rclpy.init()
    executor = rclpy.executors.SingleThreadedExecutor()

    if args.record:
        import queue
        node = TeachRecorder(args.record)
        executor.add_node(node)
        cmd_q = queue.Queue()

        def _record_interact():
            input()
            cmd_q.put('start')
            input()
            cmd_q.put('stop')

        # 실행 즉시 초기 자세로 이동
        node.go_home(executor)

        threading.Thread(target=_record_interact, daemon=True).start()

        done = False
        while not done:
            executor.spin_once(timeout_sec=0.05)
            try:
                cmd = cmd_q.get_nowait()
            except Exception:
                continue
            if cmd == 'start':
                node.start_recording()
            elif cmd == 'stop':
                node.stop_and_save()
                done = True
    else:
        node = TeachReplayer(args.replay, args.speed)
        executor.add_node(node)
        if not args.auto:
            input()
        node.replay(executor, smooth=not args.no_smooth)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



# # 초기 포지션
# - -0.6168230215655726
# - -2.0829302273192347
# - -2.4906070232391357
# - -0.6784708660892029
# - 0.4651188850402832
# - 0.6663665771484375
#  #오른쪽 물병 타겟 포지션


# - -0.022162739430562794
# - -2.162271638909811
# - -2.4077365398406982
# - -0.9789505761912842
# - 0.5077733993530273
# - 0.8893018364906311
# #중앙 타겟 포지션


# - 0.6906053423881531
# - -1.4578069981983681
# - 2.349764887486593
# - -2.2809258900084437
# - -0.5541742483722132
# - -3.316235367451803


# #왼쪽 타겟 포지션 


