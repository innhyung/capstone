import math
import threading
import serial

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_radius', 0.0625)
        self.declare_parameter('wheelbase', 0.542)
        self.declare_parameter('ticks_per_rev', 1200.0)
        self.declare_parameter('odom_frame', 'odom')
        # base_frame: URDF 루트가 base_footprint 이므로 odom 을 여기에 연결한다.
        # (base_link 에 붙이면 robot_state_publisher 의 base_footprint->base_link 와
        #  부모가 충돌해 TF 트리가 깨진다.)
        self.declare_parameter('base_frame', 'base_footprint')
        # EKF 도입 시 EKF 가 odom→base_footprint TF 를 대신 발행하므로
        # serial_bridge 는 토픽만 발행하도록 끄기.
        self.declare_parameter('publish_tf', True)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self._r = self.get_parameter('wheel_radius').value
        self._L = self.get_parameter('wheelbase').value
        tpr = self.get_parameter('ticks_per_rev').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._publish_tf_flag = self.get_parameter('publish_tf').value
        self._mpt = (2.0 * math.pi * self._r) / tpr

        try:
            self._ser = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f'Serial connected: {port} @ {baud}')
        except serial.SerialException as exc:
            self.get_logger().fatal(f'Cannot open serial port: {exc}')
            raise SystemExit(1)

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._encL = None
        self._encR = None
        self._lock = threading.Lock()
        self._buffer = ''

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf_bc = TransformBroadcaster(self) if self._publish_tf_flag else None
        if not self._publish_tf_flag:
            self.get_logger().info('publish_tf=false — TF 발행 비활성 (EKF 위임)')
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, qos)
        # PID 게인 런타임 튜닝: [Kp, Ki, Kd] 또는 [Kp, Ki, Kd, integral_limit]
        #   ros2 topic pub --once /pid_gain std_msgs/Float32MultiArray "data: [50.0, 8.0, 0.5]"
        self.create_subscription(Float32MultiArray, '/pid_gain', self._on_pid_gain, 10)

        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _on_cmd_vel(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        line = f'CMD_VEL:{int(v * 1000)},{int(w * 1000)}\n'
        try:
            self._ser.write(line.encode('ascii'))
        except serial.SerialException as exc:
            self.get_logger().warn(f'Serial write error: {exc}')

    def _on_pid_gain(self, msg: Float32MultiArray):
        data = list(msg.data)
        if len(data) == 3:
            line = f'GAIN:{data[0]:.4f},{data[1]:.4f},{data[2]:.4f}\n'
        elif len(data) == 4:
            line = f'GAIN:{data[0]:.4f},{data[1]:.4f},{data[2]:.4f},{data[3]:.4f}\n'
        else:
            self.get_logger().warn(
                f'/pid_gain expects 3 ([Kp,Ki,Kd]) or 4 ([Kp,Ki,Kd,ilim]) values, got {len(data)}'
            )
            return
        try:
            self._ser.write(line.encode('ascii'))
            self.get_logger().info(f'PID gain sent: {line.strip()}')
        except serial.SerialException as exc:
            self.get_logger().warn(f'Serial write error: {exc}')

    def _read_loop(self):
        while self._running and rclpy.ok():
            try:
                raw = self._ser.readline()
                if not raw:
                    continue
                line = raw.decode('ascii', errors='ignore').strip()
                if line.startswith('FB:'):
                    self._handle_fb(line[3:])
                elif line.startswith('GAIN_SET:'):
                    self.get_logger().info(f'ESP32 PID gains now: {line[9:]}')
            except serial.SerialException as exc:
                self.get_logger().error(f'Serial read error: {exc}')
                break

    # 물리적 한계 — 깨진 시리얼 프레임이 "쉼표4개+전부숫자"면 ValueError 없이 통과한다.
    # 그 터무니없는 속도/엔코더점프가 EKF 로 들어가면 odom 이 맵 밖으로 순간이동한다 → 여기서 차단.
    V_MAX = 1.2          # m/s. 실제 최대 ~0.5 라 여유. 이 이상이면 글리치로 보고 프레임 버림.
    W_MAX = 5.0          # rad/s. 제자리 회전 최대보다 충분히 큼.
    JUMP_MAX = 0.25      # m. 한 프레임(30~50Hz) 엔코더 이동 한계. 정상은 <0.02m.

    def _handle_fb(self, payload: str):
        parts = payload.split(',')
        if len(parts) != 4:
            return
        try:
            enc_l = int(parts[0])
            enc_r = int(parts[1])
            vel_l = int(parts[2]) / 1000.0
            vel_r = int(parts[3]) / 1000.0
        except ValueError:
            return

        # FB의 vel_l/vel_r은 좌·우 바퀴 각속도(rad/s)다. 몸체 속도로 변환.
        v_left = vel_l * self._r          # 왼쪽 바퀴 선속도 (m/s)
        v_right = vel_r * self._r         # 오른쪽 바퀴 선속도 (m/s)
        v_body = (v_left + v_right) / 2.0          # 직진 속도 (m/s)
        w_body = (v_right - v_left) / self._L      # 회전 속도 (rad/s)

        # ★ 속도 글리치 차단: 비정상 속도는 EKF 가 적분해 odom 을 날려버림 → 프레임 통째 버림.
        #    (EKF 는 빠진 한 프레임을 예측으로 메우므로 무해.) 엔코더 동기도 안 깨게 통째 스킵.
        if abs(v_body) > self.V_MAX or abs(w_body) > self.W_MAX:
            self.get_logger().warn(
                f'비정상 속도 무시 v={v_body:+.2f} w={w_body:+.2f} (시리얼 글리치?) — /odom 발행 건너뜀',
                throttle_duration_sec=1.0)
            return

        now = self.get_clock().now()

        with self._lock:
            if self._encL is not None and self._encR is not None:
                dl = (enc_l - self._encL) * self._mpt
                dr = (enc_r - self._encR) * self._mpt
                # ★ 엔코더 카운트 점프 차단: 한 프레임에 비현실적 이동이면 적분 스킵(엔코더는 재동기).
                if abs(dl) > self.JUMP_MAX or abs(dr) > self.JUMP_MAX:
                    self.get_logger().warn(
                        f'비정상 엔코더 점프 무시 dl={dl:+.2f} dr={dr:+.2f}m (글리치?) — pose 적분 스킵',
                        throttle_duration_sec=1.0)
                else:
                    d = (dl + dr) / 2.0
                    dth = (dr - dl) / self._L
                    self._x += d * math.cos(self._yaw + dth / 2.0)
                    self._y += d * math.sin(self._yaw + dth / 2.0)
                    self._yaw += dth
                    self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))

            self._encL = enc_l           # 글리치여도 재동기 — 다음 프레임 delta 가 또 커지지 않게
            self._encR = enc_r
            x, y, yaw = self._x, self._y, self._yaw

        self._publish_odom(now, x, y, yaw, v_body, w_body)
        if self._tf_bc is not None:
            self._publish_tf(now, x, y, yaw)

    def _publish_odom(self, stamp, x, y, yaw, v, w):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w
        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.05
        msg.twist.covariance[0] = 0.01
        msg.twist.covariance[35] = 0.05
        self._odom_pub.publish(msg)

    def _publish_tf(self, stamp, x, y, yaw):
        tf = TransformStamped()
        tf.header.stamp = stamp.to_msg()
        tf.header.frame_id = self._odom_frame
        tf.child_frame_id = self._base_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.z = math.sin(yaw / 2.0)
        tf.transform.rotation.w = math.cos(yaw / 2.0)
        self._tf_bc.sendTransform(tf)

    def destroy_node(self):
        self._running = False
        self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
