import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry


class PositionController(Node):
    def __init__(self):
        super().__init__('position_controller')
        self.declare_parameter('max_v', 0.35)
        self.declare_parameter('max_w', 1.8)
        self.declare_parameter('kp_v', 1.2)
        self.declare_parameter('kp_w', 3.0)
        self.declare_parameter('goal_tol', 0.05)
        self.declare_parameter('heading_tol', 0.05)

        self._max_v = self.get_parameter('max_v').value
        self._max_w = self.get_parameter('max_w').value
        self._kp_v = self.get_parameter('kp_v').value
        self._kp_w = self.get_parameter('kp_w').value
        self._goal_tol = self.get_parameter('goal_tol').value
        self._head_tol = self.get_parameter('heading_tol').value

        self._goal = None
        self._pose = None

        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.05, self._control_loop)

    def _on_goal(self, msg: PoseStamped):
        self._goal = msg.pose
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        self.get_logger().info(f'New goal → x={gx:.3f}, y={gy:.3f}')

    def _on_odom(self, msg: Odometry):
        self._pose = msg.pose.pose

    def _control_loop(self):
        if self._goal is None or self._pose is None:
            return

        cx = self._pose.position.x
        cy = self._pose.position.y
        q = self._pose.orientation
        cth = 2.0 * math.atan2(q.z, q.w)

        gx = self._goal.position.x
        gy = self._goal.position.y
        gq = self._goal.orientation
        gth = 2.0 * math.atan2(gq.z, gq.w)

        dx = gx - cx
        dy = gy - cy
        dist = math.hypot(dx, dy)

        cmd = Twist()
        if dist > self._goal_tol:
            target_heading = math.atan2(dy, dx)
            herr = self._wrap(target_heading - cth)
            float_speed = self._kp_v * dist * max(0.0, math.cos(herr))
            float_turn = self._kp_w * herr
            cmd.linear.x = self._clamp(float_speed, self._max_v)
            cmd.angular.z = self._clamp(float_turn, self._max_w)
        else:
            herr = self._wrap(gth - cth)
            if abs(herr) > self._head_tol:
                cmd.angular.z = self._clamp(self._kp_w * herr, self._max_w)
            else:
                self._goal = None
                self.get_logger().info('Goal reached!')

        self._cmd_pub.publish(cmd)

    @staticmethod
    def _wrap(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))


def main(args=None):
    rclpy.init(args=args)
    node = PositionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
