#!/usr/bin/env python3
"""
rotate_hold.py — 제자리 회전을 시간 제한 없이 계속 (PID 게인 튜닝용).

cmd_vel 로 회전 명령을 10Hz 무한 발행한다. teleop 과 달리 손 안 떼도 계속 돌아서,
다른 터미널에서 /pid_gain 으로 게인 바꿔가며 떨림이 잡히는지 실시간 관찰하기 좋다.

폐루프(cmd_vel → serial_bridge → 펌웨어 PID)라 PID 가 실제로 동작한다.
(encoder_diag.py spin 은 개루프라 PID 우회 — 튜닝엔 이 스크립트를 쓸 것)

사용법:
  python3 rotate_hold.py                 # CCW 0.3 rad/s 제자리 회전, Ctrl+C 정지
  python3 rotate_hold.py --speed 0.2     # 더 천천히
  python3 rotate_hold.py --cw            # 시계방향
  python3 rotate_hold.py --linear 0.05   # 전진 섞기 (직진+회전 떨림 보고 싶을 때)

전제:
  - serial_bridge 가 떠 있어 /cmd_vel 을 받는 중 (mapping/navigation launch)
  - teleop 종료 (cmd_vel 충돌 방지)
  - 모터 전원 ON, 바퀴는 바닥 or 띄움 (제자리 회전이라 공간 적게 필요)

게인 튜닝(다른 터미널):
  ros2 topic pub --once /pid_gain std_msgs/Float32MultiArray "{data: [40.0, 8.0, 0.2]}"
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RotateHold(Node):
    def __init__(self, angular, linear):
        super().__init__('rotate_hold')
        self.angular = angular
        self.linear = linear
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # 10Hz — 펌웨어 500ms 페일세이프보다 충분히 자주
        self.create_timer(0.1, self._tick)

    def _tick(self):
        t = Twist()
        t.linear.x = self.linear
        t.angular.z = self.angular
        self.pub.publish(t)

    def stop(self):
        zero = Twist()
        for _ in range(10):
            self.pub.publish(zero)
            time.sleep(0.05)


def main():
    ap = argparse.ArgumentParser(description='제자리 무한 회전 (PID 튜닝용)')
    ap.add_argument('--speed', type=float, default=0.3,
                    help='각속도 rad/s (기본 0.3)')
    ap.add_argument('--cw', action='store_true', help='시계방향 (기본 CCW)')
    ap.add_argument('--linear', type=float, default=0.0,
                    help='전진 속도 m/s 섞기 (기본 0 = 순수 제자리 회전)')
    args = ap.parse_args()

    angular = -args.speed if args.cw else args.speed
    print(f'{"CW" if args.cw else "CCW"} {args.speed} rad/s '
          f'(linear {args.linear} m/s) 무한 회전. 정지: Ctrl+C')
    print('게인 튜닝: 다른 터미널에서')
    print('  ros2 topic pub --once /pid_gain std_msgs/Float32MultiArray '
          '"{data: [40.0, 8.0, 0.2]}"')

    rclpy.init()
    node = RotateHold(angular, args.linear)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n[정지] cmd_vel 0 송신...')
    finally:
        try:
            node.stop()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
