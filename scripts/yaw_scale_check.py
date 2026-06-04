#!/usr/bin/env python3
"""
yaw_scale_check.py — IMU 자이로 yaw 스케일 점검 (오프라인 bag 재생용).

bag 의 /odom(휠) 과 /imu/data(자이로) 각각의 **누적 yaw** 를 적분해서,
둘의 비율(IMU/휠)을 출력한다. 비율이 1.0 에서 멀면 IMU yaw 스케일이 의심됨.

원리:
  - 휠 odom yaw: 90°=91° 검증된 적 있는 비교적 정확한 회전 기준(단, 슬립으로 장기 드리프트 있음).
  - IMU yaw: EKF 가 heading 에 쓰는 유일한 소스. 자이로 적분.
  - 두 소스가 같은 회전을 보고 얼마나 다르게 적분하는지 = 스케일 단서.
  주의: 휠도 완벽한 ground truth 는 아님(슬립). 비율이 1.0 근처면 'IMU 스케일은 범인 아님'
        쪽 증거, 크게(예: >3%) 벗어나면 IMU 스케일 의심 + 실물 360° 검증으로 확정.

사용법:
  # 터미널 A
  python3 scripts/yaw_scale_check.py --ros-args -p use_sim_time:=true
  # 터미널 B
  ros2 bag play /home/min/mapping_run5 --clock
  # 재생 끝나면 터미널 A 에서 Ctrl+C → 요약 출력
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class YawScaleCheck(Node):
    def __init__(self):
        super().__init__('yaw_scale_check')
        self.odom_cum = None      # 누적 yaw (rad, unwrap)
        self.odom_prev = None
        self.imu_cum = None       # /imu/data orientation (자이로 적분, 보정 X)
        self.imu_prev = None
        self.ekf_cum = None       # /odometry/filtered (EKF heading, 보정된 rate 반영) — 검증용
        self.ekf_prev = None
        self.create_subscription(Odometry, '/odom', self.on_odom, 50)
        self.create_subscription(Imu, '/imu/data', self.on_imu, 100)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_ekf, 50)
        self.get_logger().info('yaw_scale_check 시작 — 회전 시작하세요. 끝나면 Ctrl+C.')

    @staticmethod
    def _accumulate(cum, prev, yaw):
        if prev is None:
            return 0.0, yaw
        d = yaw - prev
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return cum + d, yaw

    def on_odom(self, msg):
        y = quat_to_yaw(msg.pose.pose.orientation)
        if self.odom_cum is None:
            self.odom_cum, self.odom_prev = 0.0, y
        else:
            self.odom_cum, self.odom_prev = self._accumulate(self.odom_cum, self.odom_prev, y)

    def on_imu(self, msg):
        y = quat_to_yaw(msg.orientation)
        if self.imu_cum is None:
            self.imu_cum, self.imu_prev = 0.0, y
        else:
            self.imu_cum, self.imu_prev = self._accumulate(self.imu_cum, self.imu_prev, y)

    def on_ekf(self, msg):
        y = quat_to_yaw(msg.pose.pose.orientation)
        if self.ekf_cum is None:
            self.ekf_cum, self.ekf_prev = 0.0, y
        else:
            self.ekf_cum, self.ekf_prev = self._accumulate(self.ekf_cum, self.ekf_prev, y)

    def report(self):
        od = math.degrees(self.odom_cum) if self.odom_cum is not None else 0.0
        im = math.degrees(self.imu_cum) if self.imu_cum is not None else 0.0
        ek = math.degrees(self.ekf_cum) if self.ekf_cum is not None else None
        print('\n══════════════ IMU yaw 스케일 점검 결과 ══════════════')
        print(f'  휠 odom 누적 회전        : {od:8.1f}°  (기준)')
        print(f'  IMU orientation 누적     : {im:8.1f}°  (보정 안 됨 — 참고용)')
        if ek is not None:
            print(f'  EKF heading(/odometry/filtered): {ek:8.1f}°  ← 보정 검증용. 휠과 같아지면 OK')
        if abs(od) > 1.0:
            ratio = im / od
            print(f'  비율 (IMU/휠)     : {ratio:.4f}   (1.0 이면 일치)')
            off = (ratio - 1.0) * 100.0
            print(f'  IMU 편차          : {off:+.1f}%')
            if abs(off) < 3.0:
                print('  → IMU yaw 스케일은 범인 아닐 가능성 (휠과 ~일치).')
            else:
                print('  → ⚠️ IMU yaw 스케일 의심. 실물 360° 검증으로 확정 권장.')
        else:
            print('  (누적 회전이 너무 작아 비율 계산 생략 — 회전 많은 구간 bag 필요)')
        print('═══════════════════════════════════════════════════════\n')


def main():
    rclpy.init()
    node = YawScaleCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
