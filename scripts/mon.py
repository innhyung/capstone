#!/usr/bin/env python3
"""mon.py — grab 미션 실시간 모니터. /rosout 의 핵심 노드 로그 + 도착신호 + 목표를 한 줄씩 시각과 함께 출력."""
import time
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped

WATCH = ('lidar_approach', 'controller_server', 'behavior_server',
         'bt_navigator', 'planner_server', 'amcl')


class Mon(Node):
    def __init__(self):
        super().__init__('mon')
        self.t0 = time.time()
        self.create_subscription(Log, '/rosout', self.on_log, 50)
        self.create_subscription(Bool, '/approach_done', self.on_done, 10)
        self.create_subscription(PointStamped, '/target_bottle_position', self.on_tgt, 10)
        print('=== 모니터 시작 — 목표 발행하고 미션 돌리세요 ===', flush=True)

    def rel(self):
        return time.time() - self.t0

    def on_log(self, m):
        if any(w in m.name for w in WATCH):
            lvl = {10:'D',20:'I',30:'W',40:'E',50:'F'}.get(m.level, '?')
            print(f'{self.rel():6.1f} [{lvl}] {m.name}: {m.msg}', flush=True)

    def on_done(self, m):
        print(f'{self.rel():6.1f} >>> /approach_done = {m.data} <<<', flush=True)

    def on_tgt(self, m):
        print(f'{self.rel():6.1f} >>> 목표수신 ({m.point.x:.2f}, {m.point.y:.2f}) <<<', flush=True)


def main():
    rclpy.init()
    n = Mon()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
