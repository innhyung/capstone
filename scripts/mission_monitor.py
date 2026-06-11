#!/usr/bin/env python3
"""
mission_monitor.py — 미션 핵심 상태만 한 터미널에 깔끔히 보여주는 대시보드.

grab_mission/navigation 은 로그가 폭포처럼 쏟아져서 지켜보기 불편 → 확인할 것들
(로봇 위치·센서 살아있나·목표·도착신호·현재 속도)만 1초마다 화면 갱신해서 표시.

별도 터미널에서:
  cd ~/capstone && source install/setup.bash
  python3 scripts/mission_monitor.py
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import Bool, String
import tf2_ros


class Monitor(Node):
    def __init__(self):
        super().__init__('mission_monitor')
        self.last = {}   # 토픽 -> 마지막 수신 시각(s)
        self.val = {}    # 토픽 -> 최신 값 요약
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(LaserScan, '/scan', lambda m: self._mark('scan'), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', lambda m: self._mark('odom'), 10)
        self.create_subscription(Imu, '/imu/data', lambda m: self._mark('imu'), qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', lambda m: self._mark('imu'), qos_profile_sensor_data)  # 시뮬 토픽명
        self.create_subscription(OccupancyGrid, '/map', lambda m: self._mark('map'), latched)
        self.create_subscription(PointStamped, '/target_bottle_position', self._on_target, 10)
        self.create_subscription(String, '/target_object', self._on_obj, 10)
        self.create_subscription(Bool, '/approach_done', self._on_done, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_timer(5.0, self.show)

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _mark(self, k):
        self.last[k] = self._now()

    def _on_target(self, m):
        self._mark('target')
        self.val['target'] = f'({m.point.x:.2f}, {m.point.y:.2f})'

    def _on_obj(self, m):
        self._mark('obj')
        self.val['obj'] = m.data

    def _on_done(self, m):
        self._mark('done')
        self.val['done'] = '✅ True' if m.data else 'False'

    def _on_cmd(self, m):
        self._mark('cmd')
        self.val['cmd'] = f'v={m.linear.x:+.2f} m/s  w={m.angular.z:+.2f} rad/s'

    def _ok(self, k, sec=2.0):
        t = self.last.get(k)
        return '✅' if (t and self._now() - t < sec) else '❌'

    def _ever(self, k):
        # latched 토픽(/map 등)용 — 한 번이라도 받았으면 ✅ (계속 안 와도 정상)
        return '✅' if k in self.last else '❌(아직)'

    def pose(self):
        try:
            tr = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            q = tr.transform.rotation
            yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                          1 - 2 * (q.y * q.y + q.z * q.z)))
            return (f'x={tr.transform.translation.x:+.2f}  '
                    f'y={tr.transform.translation.y:+.2f}  yaw={yaw:+.0f}°')
        except tf2_ros.TransformException:
            return '❌ TF(map→base) 없음 — Nav2/AMCL 활성화 안 됐거나 localize 안 됨'

    def show(self):
        print('\033[2J\033[H', end='')   # 화면 지우고 맨 위로
        print('╔═══════════════ 미션 모니터 (5초 갱신) ═══════════════')
        print(f'║ 로봇 위치(map) : {self.pose()}')
        print('║')
        print(f'║ 센서   : /scan {self._ok("scan")}   /odom {self._ok("odom")}   '
              f'/imu {self._ok("imu")}   /map {self._ever("map")}')
        print(f'║ 목표   : 이름 {self.val.get("obj", "— (아직 안 받음)")}   '
              f'좌표 {self.val.get("target", "— (아직 안 받음)")}')
        print(f'║ 도착   : /approach_done = {self.val.get("done", "— (아직)")}')
        moving = self._ok("cmd", 1.0) == "✅" and self.val.get("cmd", "").find("v=+0.00") < 0
        print(f'║ 속도   : {self.val.get("cmd", "— (정지)")}   '
              f'{"🟢 움직임" if moving else "⚪ 정지(목표 대기중이면 정상)"}')
        print('╚══════════════════════════════════════════════════════')
        print('  (센서 ❌ = 그 토픽 안 들어옴 / 위치 ❌ = localize 문제)')


def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
