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
        self.declare_parameter('publish_rate_hz', 50.0)
        # 자이로 yaw rate 스케일 보정. 2026-06-04 실물 3바퀴 테스트.
        # ⚠️ 1차로 IMU orientation(쿼터니언) 5.6% under 만 보고 1.059 적용 → 과보정이었음.
        # EKF 가 실제 쓰는 건 orientation 이 아니라 rate(angular_velocity.z)인데,
        # rate 는 원래 거의 정확(±1%)했음. scale 1.059 일 때 EKF heading 1156°(truth 1080°, +7%).
        # → 1.059×1078.7/1156.4 = 0.988 로 내려 EKF heading 이 휠/실제와 일치하게 함.
        self.declare_parameter('gyro_scale', 0.988)

        address = self.get_parameter('i2c_address').value
        self.frame_id = self.get_parameter('frame_id').value
        rate = self.get_parameter('publish_rate_hz').value
        self.gyro_scale = float(self.get_parameter('gyro_scale').value)

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

    def publish_data(self):
        now = self.get_clock().now().to_msg()

        try:
            quat = self.sensor.quaternion
            gyro = self.sensor.gyro
            accel = self.sensor.linear_acceleration
            mag = self.sensor.magnetic
            cal = self.sensor.calibration_status

            imu_msg = Imu()
            imu_msg.header.stamp = now
            imu_msg.header.frame_id = self.frame_id

            if quat and None not in quat:
                imu_msg.orientation.w = float(quat[0])
                imu_msg.orientation.x = float(quat[1])
                imu_msg.orientation.y = float(quat[2])
                imu_msg.orientation.z = float(quat[3])

            if gyro and None not in gyro:
                # gyro_scale: 자이로 스케일 보정 (5.6% under-read → ×1.059). 위 선언부 참고.
                imu_msg.angular_velocity.x = float(gyro[0]) * self.gyro_scale
                imu_msg.angular_velocity.y = float(gyro[1]) * self.gyro_scale
                imu_msg.angular_velocity.z = float(gyro[2]) * self.gyro_scale

            if accel and None not in accel:
                imu_msg.linear_acceleration.x = float(accel[0])
                imu_msg.linear_acceleration.y = float(accel[1])
                imu_msg.linear_acceleration.z = float(accel[2])

            imu_msg.orientation_covariance = [
                0.0025, 0.0, 0.0,
                0.0, 0.0025, 0.0,
                0.0, 0.0, 0.0025,
            ]
            imu_msg.angular_velocity_covariance = [
                0.02, 0.0, 0.0,
                0.0, 0.02, 0.0,
                0.0, 0.0, 0.02,
            ]
            imu_msg.linear_acceleration_covariance = [
                0.04, 0.0, 0.0,
                0.0, 0.04, 0.0,
                0.0, 0.0, 0.04,
            ]
            self.imu_pub.publish(imu_msg)

            if mag and None not in mag:
                mag_msg = MagneticField()
                mag_msg.header.stamp = now
                mag_msg.header.frame_id = self.frame_id
                # μT → Tesla
                mag_msg.magnetic_field.x = float(mag[0]) * 1e-6
                mag_msg.magnetic_field.y = float(mag[1]) * 1e-6
                mag_msg.magnetic_field.z = float(mag[2]) * 1e-6
                self.mag_pub.publish(mag_msg)

            sys_cal, gyro_cal, accel_cal, mag_cal = cal
            if mag_cal < 3 or sys_cal < 3:
                self.get_logger().warn(
                    f'Cal: sys={sys_cal} gyro={gyro_cal} accel={accel_cal} mag={mag_cal}',
                    throttle_duration_sec=5.0,
                )

        except Exception as e:
            self.get_logger().error(f'sensor read error: {e}', throttle_duration_sec=2.0)


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
