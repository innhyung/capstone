#!/usr/bin/env python3
"""arm_domain_bridge.py — nav/조원(도메인 28) ↔ 로봇팔(도메인 77) '핸드셰이크 토픽만' 중계.

왜: 팔코드는 자기 RSP로 base_link(UR3 받침)을 발행하는데, 같은 도메인이면 nav의
    base_link(모바일베이스)와 이름이 겹쳐 nav의 map->base TF 가 깨진다(목표 '보류').
    → 팔을 도메인 77로 분리하면 TF가 완전 격리돼 충돌이 사라진다. 단 그러면 도착/파지
      신호도 서로 안 보이므로, 이 브릿지가 필요한 토픽만 골라 양 도메인으로 복사한다.
    TF/base_link 는 절대 중계하지 않음 → 그래서 충돌이 안 생김.

중계:
  /grasp_command (std_msgs/String) : 28 -> 77   (조원 CCTV -> 팔)
  /approach_done (std_msgs/Bool)   : 28 -> 77   (nav lidar_approach -> 팔)
  /grasp_done    (std_msgs/Bool)   : 77 -> 28   (팔 -> nav lidar_approach)

실행:
  source /opt/ros/jazzy/setup.bash
  python3 scripts/arm_domain_bridge.py
"""
import threading
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String, Bool

NAV_DOMAIN = 28   # nav + 조원 CCTV
ARM_DOMAIN = 77   # 로봇팔 (run_arm.sh 와 일치시킬 것)


def main():
    ctx_nav = Context(); rclpy.init(context=ctx_nav, domain_id=NAV_DOMAIN)
    ctx_arm = Context(); rclpy.init(context=ctx_arm, domain_id=ARM_DOMAIN)

    nav = Node('arm_bridge_nav', context=ctx_nav)
    arm = Node('arm_bridge_arm', context=ctx_arm)

    # 28 -> 77 (팔 도메인으로 내보낼 publisher)
    pub_cmd_arm = arm.create_publisher(String, '/grasp_command', 10)
    pub_app_arm = arm.create_publisher(Bool, '/approach_done', 10)
    # 77 -> 28 (nav 도메인으로 내보낼 publisher)
    pub_done_nav = nav.create_publisher(Bool, '/grasp_done', 10)

    def on_cmd(msg):
        pub_cmd_arm.publish(msg)
        nav.get_logger().info(f'[28->77] /grasp_command: {msg.data}')

    def on_app(msg):
        pub_app_arm.publish(msg)
        nav.get_logger().info(f'[28->77] /approach_done: {msg.data}')

    def on_done(msg):
        pub_done_nav.publish(msg)
        arm.get_logger().info(f'[77->28] /grasp_done: {msg.data}')

    nav.create_subscription(String, '/grasp_command', on_cmd, 10)
    nav.create_subscription(Bool, '/approach_done', on_app, 10)
    arm.create_subscription(Bool, '/grasp_done', on_done, 10)

    exec_nav = SingleThreadedExecutor(context=ctx_nav); exec_nav.add_node(nav)
    exec_arm = SingleThreadedExecutor(context=ctx_arm); exec_arm.add_node(arm)

    t_arm = threading.Thread(target=exec_arm.spin, daemon=True); t_arm.start()
    print(f'[arm_domain_bridge] nav(domain {NAV_DOMAIN}) <-> arm(domain {ARM_DOMAIN}) 중계 시작')
    print('  /grasp_command 28->77 | /approach_done 28->77 | /grasp_done 77->28')
    print('  (TF/base_link 은 중계 안 함 — 격리 유지)')
    try:
        exec_nav.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown(context=ctx_nav)
        rclpy.shutdown(context=ctx_arm)


if __name__ == '__main__':
    main()
