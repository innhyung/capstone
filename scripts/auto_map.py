#!/usr/bin/env python3
"""
auto_map.py — 자동 매핑 주행 (작은 공간용, loop closure 친화적 패턴).

동작:
  1) 시작 위치에서 360° 회전     → 맵 앵커링 (주변 여러 관측으로 초기 안정화)
  2) 점점 커지는 "사각 루프" 주행 → 매 바퀴 출발점 근처로 복귀 = loop closure
  3) 루프 사이마다 360° 회전     → 관측 보강
  odom 피드백으로 직진거리/회전각을 정확히 맞춘다 (odom 정확하게 캘됨 전제).
  전방 라이다로 장애물 가까우면 직진을 일찍 멈춰 충돌을 줄인다.

⚠️ 주의:
  - 맵이 없는 상태라 "지도 기반 회피"는 없음 → 전방 거리 안전정지만 있음.
    벽에 박을 수 있으니 **공간 확보 + 옆에서 지켜보다 Ctrl+C 가능하게**.
  - serial_bridge + slam_toolbox + EKF 가 떠 있어야 함 (mapping launch).
  - teleop 종료 (cmd_vel 충돌).

사용법:
  python3 auto_map.py                       # 기본: 1m→1.5m→2m 사각 3바퀴
  python3 auto_map.py --side 0.8 --loops 4  # 더 작게 시작, 4바퀴
  python3 auto_map.py --speed 0.1           # 더 천천히
  정지: Ctrl+C
"""
import argparse
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def angle_normalize(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class AutoMapper(Node):
    def __init__(self, args):
        super().__init__('auto_map')
        self.speed = args.speed
        self.turn_speed = args.turn
        self.side = args.side
        self.grow = args.grow
        self.loops = args.loops
        self.safety = args.safety
        self.front_half = math.radians(args.front_deg)

        self.x = None
        self.y = None
        self.yaw = 0.0
        self.front_min = float('inf')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)

    def _on_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = quat_to_yaw(msg.pose.pose.orientation)

    def _on_scan(self, msg):
        best = float('inf')
        a = msg.angle_min
        for r in msg.ranges:
            aa = angle_normalize(a)
            if abs(aa) <= self.front_half and msg.range_min < r < msg.range_max:
                if r < best:
                    best = r
            a += msg.angle_increment
        self.front_min = best

    # --- 저수준 동작 ---
    def _spin_some(self):
        rclpy.spin_once(self, timeout_sec=0.02)

    def _wait_odom(self):
        while rclpy.ok() and self.x is None:
            self._spin_some()

    def _stop(self, n=8):
        for _ in range(n):
            self.pub.publish(Twist())
            self._spin_some()

    def drive_straight(self, dist):
        self._wait_odom()
        x0, y0 = self.x, self.y
        target_yaw = self.yaw
        while rclpy.ok():
            self._spin_some()
            if math.hypot(self.x - x0, self.y - y0) >= dist:
                break
            if self.front_min < self.safety:
                self.get_logger().warn(
                    f'전방 {self.front_min:.2f}m 장애물 — 직진 조기 종료')
                break
            t = Twist()
            t.linear.x = self.speed
            # 가벼운 헤딩 유지 (직선 똑바로)
            err = angle_normalize(target_yaw - self.yaw)
            t.angular.z = max(-0.4, min(0.4, 1.5 * err))
            self.pub.publish(t)
        self._stop()

    def turn(self, angle):
        """signed radians (양수=CCW)."""
        self._wait_odom()
        prev = self.yaw
        acc = 0.0
        sign = 1.0 if angle > 0 else -1.0
        while rclpy.ok() and abs(acc) < abs(angle):
            self._spin_some()
            acc += angle_normalize(self.yaw - prev)
            prev = self.yaw
            t = Twist()
            t.angular.z = sign * self.turn_speed
            self.pub.publish(t)
        self._stop()

    # --- 시퀀스 ---
    def run(self):
        self._wait_odom()
        self.get_logger().info('① 초기 360° 회전 (맵 앵커링)')
        self.turn(2 * math.pi)

        for i in range(self.loops):
            side = self.side + i * self.grow
            self.get_logger().info(
                f'② 사각 루프 {i + 1}/{self.loops} — 한 변 {side:.1f}m')
            for edge in range(4):
                self.drive_straight(side)
                self.turn(math.pi / 2)      # 90° (CCW)
            self.get_logger().info('③ 루프 사이 360° 회전 (관측 보강 + loop closure)')
            self.turn(2 * math.pi)

        self.get_logger().info('✅ 자동 매핑 주행 완료 — 맵 저장하세요.')


def main():
    ap = argparse.ArgumentParser(description='자동 매핑 주행 (작은 공간)')
    ap.add_argument('--speed', type=float, default=0.12, help='직진 속도 m/s (기본 0.12)')
    ap.add_argument('--turn', type=float, default=0.4, help='회전 속도 rad/s (기본 0.4)')
    ap.add_argument('--side', type=float, default=1.0, help='첫 사각 한 변 m (기본 1.0)')
    ap.add_argument('--grow', type=float, default=0.5, help='바퀴마다 한 변 증가 m (기본 0.5)')
    ap.add_argument('--loops', type=int, default=3, help='사각 루프 횟수 (기본 3)')
    ap.add_argument('--safety', type=float, default=0.5, help='전방 안전거리 m (기본 0.5)')
    ap.add_argument('--front-deg', type=float, default=25.0,
                    help='전방 감지 부채꼴 반각 ° (기본 25)')
    args = ap.parse_args()

    print(f'자동 매핑: 360°앵커 → 사각 {args.side}m부터 {args.loops}바퀴 '
          f'(+{args.grow}m/바퀴), 속도 {args.speed} m/s. 정지 Ctrl+C')

    rclpy.init()
    node = AutoMapper(args)
    try:
        node.run()
    except KeyboardInterrupt:
        print('\n[중단] 정지...')
    finally:
        try:
            node._stop(10)
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
