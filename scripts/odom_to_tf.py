#!/usr/bin/env python3
"""
odom_to_tf.py — /odom (nav_msgs/Odometry) 의 pose 를 odom->base_footprint TF 로 발행.

EKF 를 끄고 "wheel odom + scan 만으로" slam_toolbox 를 돌릴 때 쓴다 (EKF on/off 비교용).
EKF 가 하던 odom->base_footprint TF 역할을, wheel odom 그대로 TF 로 내보내는 것.
(주의: serial_bridge 의 wheel yaw 는 좌우 엔코더 비대칭이 있어 회전 정확도는 IMU 보다 낮음.
 그래서 평소엔 EKF(IMU vyaw)를 쓰지만, 비교 진단을 위해 이 노드로 wheel-only 를 본다.)

사용 (offline_slam.launch.py use_ekf:=false 가 자동 실행):
  python3 odom_to_tf.py --ros-args -p use_sim_time:=true
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self._cb, 10)
        self.get_logger().info('odom_to_tf: /odom → odom->base_footprint TF (EKF 대체)')

    def _cb(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp           # bag 의 원래 타임스탬프 유지
        t.header.frame_id = msg.header.frame_id or 'odom'
        t.child_frame_id = msg.child_frame_id or 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTf()
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
