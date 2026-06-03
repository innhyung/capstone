#!/usr/bin/env python3
"""
rotation_test.py — odom 기준 360° 회전 캘리브레이션 테스트.

목적:
  로봇을 /odom 기준 정확히 360° (또는 --target 만큼) 회전시킨 뒤 정지.
  사용자는 실제 로봇이 시각적으로 한 바퀴 돌았는지 확인 → wheelbase 보정.

사용법:
  python3 rotation_test.py                     # CCW 360°, 0.3 rad/s
  python3 rotation_test.py --speed 0.2         # 더 천천히 (슬립 ↓)
  python3 rotation_test.py --cw                # 시계방향
  python3 rotation_test.py --target 720        # 두 바퀴

전제:
  - mapping launch 가 떠 있어 /odom 발행 중
  - 모터 전원 ON
  - 텔레옵 종료 (없으면 cmd_vel 충돌)
  - 로봇 정면에 시각적 마커 (테이프 등)
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def quat_to_yaw(q):
    """Quaternion → yaw (rad, in -pi..pi)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_normalize(d):
    """Normalize angle to (-pi, pi]."""
    while d > math.pi:
        d -= 2 * math.pi
    while d <= -math.pi:
        d += 2 * math.pi
    return d


class RotationTester(Node):
    def __init__(self, target_rad, angular_speed):
        super().__init__('rotation_tester')
        self.target_rad = target_rad
        self.angular_speed = angular_speed

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        # 10Hz 명령 발행 — 펌웨어 500ms 페일세이프 보다 충분히 자주
        self.create_timer(0.1, self._tick)

        self.start_yaw = None
        self.prev_yaw = None
        self.accumulated = 0.0
        self.done = False
        self.start_time = None
        self.last_log_time = 0.0

    def _on_odom(self, msg):
        yaw = quat_to_yaw(msg.pose.pose.orientation)

        if self.start_yaw is None:
            self.start_yaw = yaw
            self.prev_yaw = yaw
            self.start_time = time.time()
            print(f'\n시작 yaw: {math.degrees(yaw):+7.2f}°\n')
            return

        # 누적 회전 계산 (wrap 안전)
        d = angle_normalize(yaw - self.prev_yaw)
        self.accumulated += d
        self.prev_yaw = yaw

        # 1초마다 진행상황
        now = time.time()
        if now - self.last_log_time >= 1.0 and not self.done:
            self.last_log_time = now
            pct = abs(self.accumulated) / self.target_rad * 100.0
            print(f'  진행: {math.degrees(self.accumulated):+7.2f}° / '
                  f'{math.degrees(self.target_rad):+7.2f}° ({pct:5.1f}%)')

        # 목표 도달 → 정지
        if abs(self.accumulated) >= self.target_rad and not self.done:
            self.done = True
            elapsed = time.time() - self.start_time
            # 정지 명령 여러 번 송신
            stop = Twist()
            for _ in range(20):
                self.cmd_pub.publish(stop)
                time.sleep(0.02)
            print(
                f'\n=== 완료 ===\n'
                f'시작 yaw  : {math.degrees(self.start_yaw):+7.2f}°\n'
                f'끝 yaw    : {math.degrees(yaw):+7.2f}°\n'
                f'odom 누적 : {math.degrees(self.accumulated):+7.2f}°\n'
                f'소요 시간 : {elapsed:.1f}s\n'
                f'\n→ 시각적으로 로봇 정면 화살표가 시작 위치 대비:\n'
                f'   • 정확히 같은 곳   : wheelbase 정확 ✅\n'
                f'   • 덜 돌았음 (예 350°): wheelbase 너무 작음 → 키우기\n'
                f'   • 더 돌았음 (예 370°): wheelbase 너무 큼 → 줄이기\n'
                f'   • 보정 공식: new_wb = current_wb × 360 / 실제각도\n'
            )
            rclpy.shutdown()

    def _tick(self):
        if self.start_yaw is None or self.done:
            return
        t = Twist()
        t.angular.z = self.angular_speed
        self.cmd_pub.publish(t)


def main():
    parser = argparse.ArgumentParser(description='360° 회전 캘리브레이션 테스트')
    parser.add_argument('--speed', type=float, default=0.3,
                        help='각속도 rad/s (기본 0.3, 슬립 줄이려면 0.2)')
    parser.add_argument('--cw', action='store_true',
                        help='시계방향 회전 (기본은 CCW)')
    parser.add_argument('--target', type=float, default=360.0,
                        help='목표 회전 각도 ° (기본 360)')
    args = parser.parse_args()

    speed = -args.speed if args.cw else args.speed
    target_rad = math.radians(args.target)
    print(f'설정: {"CW" if args.cw else "CCW"} {args.target}°, '
          f'각속도 {args.speed} rad/s')
    print('정지하려면 Ctrl+C')

    rclpy.init()
    node = RotationTester(target_rad, speed)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n[중단] 정지 명령 송신...')
    finally:
        # 안전 정지
        stop = Twist()
        for _ in range(10):
            try:
                node.cmd_pub.publish(stop)
            except Exception:
                pass
            time.sleep(0.05)
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
