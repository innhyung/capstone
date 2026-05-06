#!/usr/bin/env python3
"""
Nav2 navigation goal relay 노드.

/simple_goal 토픽(PoseStamped)을 받으면 nav2 의 navigate_to_pose action 으로 전달.
진행 거리/결과를 logger 로 출력 + /navigation_result 토픽(String) 으로 publish.

새 goal 이 들어오면 진행 중인 goal 은 자동 cancel 후 새로 시작 (preemption).

Usage:
  # 1) launch 또는 직접 실행
  ros2 run ur3_4040_urdf goal_relay_node.py

  # 2) 다른 터미널에서 goal 발행 (예: map(5,2), yaw 90도)
  ros2 topic pub --once /simple_goal geometry_msgs/PoseStamped \\
    "{header: {frame_id: 'map'},
      pose: {position: {x: 5.0, y: 2.0, z: 0.0},
             orientation: {x: 0, y: 0, z: 0.707, w: 0.707}}}"

  # 3) 결과 모니터
  ros2 topic echo /navigation_result
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose


STATUS_STR = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
}


class GoalRelay(Node):
    def __init__(self):
        super().__init__('goal_relay')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(PoseStamped, '/simple_goal', self.on_goal, 10)
        self.result_pub = self.create_publisher(String, '/navigation_result', 10)
        self.current_handle = None
        self.last_print = self.get_clock().now()
        self.get_logger().info(
            'goal_relay 시작. /simple_goal (geometry_msgs/PoseStamped) 대기 중...'
        )

    def publish_result(self, status_str: str):
        msg = String()
        msg.data = status_str
        self.result_pub.publish(msg)

    def on_goal(self, msg: PoseStamped):
        # 진행 중 goal 있으면 cancel
        if self.current_handle is not None:
            try:
                self.get_logger().info('진행 중 goal cancel 후 새 goal 처리')
                self.current_handle.cancel_goal_async()
            except Exception:
                pass
            self.current_handle = None

        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                'navigate_to_pose action server 없음 — nav2 실행 중인지 확인'
            )
            self.publish_result('REJECTED')
            return

        if not msg.header.frame_id:
            msg.header.frame_id = 'map'

        goal = NavigateToPose.Goal()
        goal.pose = msg
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.get_logger().info(
            f'Goal 전송: ({x:.2f}, {y:.2f}) frame={msg.header.frame_id}'
        )

        send_future = self.action_client.send_goal_async(
            goal, feedback_callback=self.on_feedback
        )
        send_future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal REJECTED by nav2')
            self.publish_result('REJECTED')
            return
        self.get_logger().info('Goal accepted by nav2')
        self.current_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        # 1초에 한 번만 출력 (스팸 방지)
        now = self.get_clock().now()
        if (now - self.last_print).nanoseconds > 1_000_000_000:
            eta = fb.estimated_time_remaining.sec
            self.get_logger().info(
                f'  ... 남은 거리 {fb.distance_remaining:.2f}m, '
                f'ETA {eta}s, recoveries {fb.number_of_recoveries}'
            )
            self.last_print = now

    def on_result(self, future):
        result = future.result()
        status = result.status
        status_str = STATUS_STR.get(status, f'STATUS_{status}')
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'결과: {status_str}')
        else:
            self.get_logger().warn(f'결과: {status_str}')
        self.publish_result(status_str)
        self.current_handle = None


def main():
    rclpy.init()
    node = GoalRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
