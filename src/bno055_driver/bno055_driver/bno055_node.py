import board
import adafruit_bno055
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField


class BNO055Node(Node):
    def __init__(self):
        super().__init__('bno055_node')

        self.declare_parameter('i2c_address', 0x29)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate_hz', 100.0)   # 자이로만 읽어 빨라졌으니 상한↑(실제는 I2C 한계로 settle)
        # 자이로 yaw rate 스케일 보정. 2026-06-04 실물 3바퀴 테스트.
        # ⚠️ 1차로 IMU orientation(쿼터니언) 5.6% under 만 보고 1.059 적용 → 과보정이었음.
        # EKF 가 실제 쓰는 건 orientation 이 아니라 rate(angular_velocity.z)인데,
        # rate 는 원래 거의 정확(±1%)했음. scale 1.059 일 때 EKF heading 1156°(truth 1080°, +7%).
        # → 1.059×1078.7/1156.4 = 0.988 로 내려 EKF heading 이 휠/실제와 일치하게 함.
        # 2026-06-13(저녁) rosbag 정량 재측정: 7바퀴 물리 +2520° 회전 시 자이로(scale 1.0935)는 +2854°(×1.133)로
        # 13% 과대, 휠은 +2530°(×1.004)로 정확 → 1.0935는 눈대중 오측이었음. raw 자이로 3.6% 과대 → 0.965 로 보정
        # (휠/실제와 일치). 1.0935 가 회전마다 +z 헤딩 주입 → AMCL 누적 틀어짐의 진짜 원인이었음. 상세 [[imu-gyro-yaw-scale]]
        self.declare_parameter('gyro_scale', 0.965)

        address = self.get_parameter('i2c_address').value
        self.frame_id = self.get_parameter('frame_id').value
        rate = self.get_parameter('publish_rate_hz').value
        self.gyro_scale = float(self.get_parameter('gyro_scale').value)
        # ★ 런타임 튜닝: ros2 param set /bno055_node gyro_scale X 하면 재시작 없이 즉시 반영
        self.add_on_set_parameters_callback(self._on_param)

        i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(i2c, address=address)
        # IMU 모드(IMUPLUS): 자이로+가속도만 융합, 지자기 미사용.
        # 모바일 로봇은 모터/금속 차체 때문에 지자기가 오염되므로
        # 기본 NDOF 대신 이 모드를 쓴다 → 지자기 캘리브레이션 불필요.
        self.sensor.mode = adafruit_bno055.IMUPLUS_MODE

        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_data)

        self.get_logger().info(
            f'BNO055 노드 시작 (addr=0x{address:02X}, frame={self.frame_id}, '
            f'{rate}Hz, mode=0x{self.sensor.mode:02X} IMUPLUS)'
        )

    def _on_param(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'gyro_scale':
                self.gyro_scale = float(p.value)
                self.get_logger().info(f'gyro_scale 런타임 변경 → {self.gyro_scale}')
        return SetParametersResult(successful=True)

    def publish_data(self):
        now = self.get_clock().now().to_msg()
        # ★ EKF 는 IMU 에서 angular_velocity.z(yaw rate) 만 쓴다(imu0_config 12번째만 true).
        #   그래서 quaternion·accel·mag 를 안 읽고 '자이로만' 읽는다 → I2C 5읽기→1읽기 → 발행률↑
        #   (BNO055 I2C 읽기당 ~10ms 라 19Hz 천장이었음. 자이로만이면 40~50Hz 기대).
        try:
            gyro = self.sensor.gyro
        except Exception as e:
            self.get_logger().error(f'gyro read error: {e}', throttle_duration_sec=2.0)
            return
        if not gyro or None in gyro:
            return

        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = self.frame_id
        imu_msg.orientation_covariance[0] = -1.0   # orientation 없음(REP-145) — EKF 가 무시
        imu_msg.angular_velocity.x = float(gyro[0]) * self.gyro_scale
        imu_msg.angular_velocity.y = float(gyro[1]) * self.gyro_scale
        imu_msg.angular_velocity.z = float(gyro[2]) * self.gyro_scale
        imu_msg.angular_velocity_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.02,
        ]
        self.imu_pub.publish(imu_msg)

        # 캘리브레이션은 매 사이클이 아니라 가끔만 읽어 로그(I2C 절약). ~1초마다.
        self._cal_count = getattr(self, '_cal_count', 0) + 1
        if self._cal_count >= 25:
            self._cal_count = 0
            try:
                sys_cal, gyro_cal, accel_cal, mag_cal = self.sensor.calibration_status
                if gyro_cal < 3:
                    self.get_logger().warn(
                        f'자이로 캘리브 미완 gyro_cal={gyro_cal}/3 (3 돼야 yaw 정확)',
                        throttle_duration_sec=10.0)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = BNO055Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
