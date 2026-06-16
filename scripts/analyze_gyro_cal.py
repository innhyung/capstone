#!/usr/bin/env python3
"""gyro_cal_turns bag 적분 분석 — 자이로/휠 yaw vs 물리 N×360° 비교."""
import math
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

BAG = '/home/min/capstone/bags/gyro_cal_turns'
PHYS_TURNS = 7
PHYS_DEG = PHYS_TURNS * 360.0   # 반시계(+)

reader = SequentialReader()
reader.open(StorageOptions(uri=BAG, storage_id='mcap'), ConverterOptions('', ''))
typemap = {'/imu/data': Imu, '/odom': Odometry}

imu_t, imu_w = [], []      # (stamp_s, omega_z)
odom_t, odom_w = [], []
odom_yaw = []              # pose orientation yaw (for unwrap cross-check)

def yaw_of(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

while reader.has_next():
    topic, data, t = reader.read_next()
    ts = t / 1e9
    if topic == '/imu/data':
        m = deserialize_message(data, Imu)
        imu_t.append(ts); imu_w.append(m.angular_velocity.z)
    elif topic == '/odom':
        m = deserialize_message(data, Odometry)
        odom_t.append(ts); odom_w.append(m.twist.twist.angular.z)
        odom_yaw.append(yaw_of(m.pose.pose.orientation))

def integrate(ts, w, bias=0.0):
    total = 0.0
    for i in range(1, len(ts)):
        dt = ts[i] - ts[i-1]
        if dt <= 0 or dt > 1.0:   # 갭 무시
            continue
        total += (w[i] - bias) * dt
    return total

def est_bias(ts, w, thr=0.03):
    """저운동 구간(|w|<thr)에서 평균 = 정지 bias 추정."""
    vals = [w[i] for i in range(len(w)) if abs(w[i]) < thr]
    return sum(vals)/len(vals) if vals else 0.0

def unwrap_total(yaw):
    total = 0.0
    for i in range(1, len(yaw)):
        d = yaw[i] - yaw[i-1]
        while d > math.pi: d -= 2*math.pi
        while d < -math.pi: d += 2*math.pi
        total += d
    return total

imu_bias = est_bias(imu_t, imu_w)
odom_bias = est_bias(odom_t, odom_w)

gyro_raw = math.degrees(integrate(imu_t, imu_w))
gyro_bc  = math.degrees(integrate(imu_t, imu_w, imu_bias))
wheel_raw = math.degrees(integrate(odom_t, odom_w))
wheel_bc  = math.degrees(integrate(odom_t, odom_w, odom_bias))
wheel_pose = math.degrees(unwrap_total(odom_yaw))

print(f"=== 물리 진실값: {PHYS_TURNS}바퀴 반시계 = +{PHYS_DEG:.0f}° ===\n")
print(f"[자이로 /imu/data] (gyro_scale=1.0935 이미 반영됨)")
print(f"  적분 yaw        : {gyro_raw:+.1f}°")
print(f"  bias보정({math.degrees(imu_bias):+.4f}°/s) : {gyro_bc:+.1f}°")
print(f"  vs 물리 비율     : {gyro_bc/PHYS_DEG:.4f}  (1.0=정확, >1=과대)")
print(f"  → 올바른 gyro_scale = 1.0935 × {PHYS_DEG:.0f}/{abs(gyro_bc):.1f} = {1.0935*PHYS_DEG/abs(gyro_bc):.4f}\n")
print(f"[휠 /odom twist.angular.z] (자이로와 독립)")
print(f"  적분 yaw        : {wheel_raw:+.1f}°")
print(f"  bias보정({math.degrees(odom_bias):+.4f}°/s) : {wheel_bc:+.1f}°")
print(f"  pose unwrap 교차 : {wheel_pose:+.1f}°")
print(f"  vs 물리 비율     : {wheel_bc/PHYS_DEG:.4f}\n")
print(f"[참고] imu {len(imu_t)}msg, odom {len(odom_t)}msg, 측정시간 {imu_t[-1]-imu_t[0]:.1f}s")
