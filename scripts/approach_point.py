#!/usr/bin/env python3
"""
approach_point.py — 목표 좌표를 받으면 그 점에서 반경 R(기본 0.7m) 떨어진 지점으로 가서
목표를 정면으로 바라보게 정렬한다.

기하:
  - 목표점 P 를 중심으로 반경 R 인 원을 그림.
  - 로봇 현재 위치 쪽의 원 위의 점 G 로 접근 (가장 가까운 접근점).
  - 도착 방향 = G 에서 P 를 바라보는 방향 = 반지름 방향 = 원의 접선에 수직.

입력: /clicked_point (geometry_msgs/PointStamped)
      → RViz "Publish Point" 로 클릭해 테스트하거나, 다른 노드가 이 토픽으로 좌표를 쏘면 됨.
출력: /navigate_to_pose 액션으로 Nav2 에 목표 전송 (Nav2 가 경로계획·주행).

전제: navigation.launch.py 로 Nav2(AMCL+planner+controller) 가 떠 있고 map->base_link TF 가 살아있을 것.

사용:
  python3 scripts/approach_point.py                         # 반경 0.7m
  python3 scripts/approach_point.py --ros-args -p radius:=1.0
  # 그다음 RViz Publish Point 로 목표 클릭 → 로봇이 0.7m 앞에서 그 점을 바라보며 정지.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
import tf2_ros


class ApproachPoint(Node):
    def __init__(self):
        super().__init__('approach_point')
        self.declare_parameter('radius', 0.7)            # 목표에서 떨어질 거리(m)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('face_target', True)      # True=목표 바라봄, False=목표 등짐
        self.radius = float(self.get_parameter('radius').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.face_target = bool(self.get_parameter('face_target').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.create_subscription(PointStamped, '/clicked_point', self.on_target, 10)
        self.get_logger().info(
            f'approach_point 시작 — /clicked_point 수신 시 반경 {self.radius}m 접근. '
            f'(RViz Publish Point 로 테스트)')

    def robot_xy(self):
        """map 프레임에서 base_link 의 (x, y) 를 TF 로 조회. 실패 시 None."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'TF 조회 실패 ({self.map_frame}->{self.base_frame}): {e}')
            return None

    def on_target(self, msg):
        px, py = msg.point.x, msg.point.y
        rob = self.robot_xy()
        if rob is None:
            return
        rx, ry = rob

        # 목표 P 에서 로봇 R 쪽으로 향하는 단위벡터 (로봇 쪽 접근점을 고르려고)
        dx, dy = rx - px, ry - py
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            self.get_logger().warn('로봇이 목표점과 거의 같은 위치 — 접근 방향 결정 불가, 무시')
            return
        ux, uy = dx / dist, dy / dist

        # 접근점 G = 목표에서 반경만큼 로봇 쪽으로 떨어진 원 위의 점
        gx = px + self.radius * ux
        gy = py + self.radius * uy

        # 방향: 목표 바라봄(face_target) = G→P 방향(반지름, 접선에 수직). 반대면 P→G.
        if self.face_target:
            yaw = math.atan2(py - gy, px - gx)
        else:
            yaw = math.atan2(gy - py, gx - px)

        self.send_goal(gx, gy, yaw, px, py, dist)

    def send_goal(self, x, y, yaw, px, py, cur_dist):
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose 액션 서버 없음 — Nav2(navigation.launch.py) 떠 있나 확인')
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.get_logger().info(
            f'목표 P({px:.2f},{py:.2f}) [현재거리 {cur_dist:.2f}m] '
            f'→ 접근점 G({x:.2f},{y:.2f}) yaw={math.degrees(yaw):.0f}° 전송')
        self.nav_client.send_goal_async(
            goal, feedback_callback=self._on_feedback).add_done_callback(self._on_response)

    def _on_feedback(self, msg):
        # Nav2 가 주는 진행상황 — 남은 거리를 1초마다 표시
        self.get_logger().info(
            f'남은 거리: {msg.feedback.distance_remaining:.2f} m',
            throttle_duration_sec=1.0)

    def _on_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn('목표 거부됨 (도달 불가 위치일 수 있음)')
            return
        gh.get_result_async().add_done_callback(
            lambda f: self.get_logger().info('주행 종료 (도착 또는 중단)'))


def main():
    rclpy.init()
    node = ApproachPoint()
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
