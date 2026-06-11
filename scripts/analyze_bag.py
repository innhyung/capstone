#!/usr/bin/env python3
"""
analyze_bag.py — grab 미션 rosbag 에서 "회전할 때 odom/AMCL 이 왜 안 따라가나" 디버깅.

ALIGNING(제자리 회전) 순간에:
  - cmd_vel.angular.z (회전 명령이 실제로 나갔나)
  - imu/data.angular.z (자이로가 회전을 봤나)
  - TF odom->base_footprint yaw (EKF 가 회전을 반영했나)   ← 핵심
  - TF map->odom yaw (AMCL 이 상쇄/발산했나)               ← 핵심
를 시간순으로 정렬해 보여준다. 이걸 보면 범인이 EKF 냐 AMCL 이냐가 숫자로 확정됨.

사용:
  ros2 run ... 말고 그냥:
  source /opt/ros/jazzy/setup.bash   # rosbag2_py 필요
  python3 scripts/analyze_bag.py ~/grab_debug         # bag 폴더 경로
  python3 scripts/analyze_bag.py ~/grab_debug 120 140 # 120~140초 구간만 자세히
"""
import sys
import math

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw_of(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                   1 - 2 * (q.y * q.y + q.z * q.z)))


def main():
    if len(sys.argv) < 2:
        print('사용: python3 analyze_bag.py <bag_dir> [t_start t_end]')
        return
    bag = sys.argv[1]
    win = None
    if len(sys.argv) >= 4:
        win = (float(sys.argv[2]), float(sys.argv[3]))

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id=''),
                ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msgcls = {n: get_message(t) for n, t in types.items()}
    print(f'토픽: {sorted(types)}')

    rows = []          # (t, key, value...) 시계열 통합
    t0 = None
    # 누적 추적용 최신값
    last = {'cmd_w': 0.0, 'cmd_v': 0.0, 'imu_z': 0.0,
            'odom_base_yaw': None, 'map_odom_yaw': None, 'ekf_yaw': None}

    while reader.has_next():
        topic, data, tns = reader.read_next()
        t = tns / 1e9
        if t0 is None:
            t0 = t
        rel = t - t0
        if win and not (win[0] <= rel <= win[1]):
            continue
        try:
            msg = deserialize_message(data, msgcls[topic])
        except Exception:
            continue

        if topic in ('/cmd_vel', '/cmd_vel_smoothed', '/cmd_vel_nav'):
            tw = msg.twist if hasattr(msg, 'twist') else msg
            rows.append((rel, topic, tw.linear.x, tw.angular.z))
        elif topic in ('/imu/data', '/imu'):
            rows.append((rel, 'imu', msg.angular_velocity.z))
        elif topic in ('/odometry/filtered',):
            rows.append((rel, 'ekf', yaw_of(msg.pose.pose.orientation)))
        elif topic == '/tf':
            for tr in msg.transforms:
                pc = (tr.header.frame_id, tr.child_frame_id)
                if pc == ('odom', 'base_footprint') or pc == ('odom', 'base_link'):
                    rows.append((rel, 'odom->base', yaw_of(tr.transform.rotation)))
                elif pc == ('map', 'odom'):
                    rows.append((rel, 'map->odom', yaw_of(tr.transform.rotation)))

    rows.sort(key=lambda r: r[0])

    # 1) cmd_vel 로 회전 구간(ALIGNING 추정) 찾기: |angular.z|>0.05 & |linear.x|<0.05 가 지속
    print('\n=== /cmd_vel 회전 명령 구간 (|w|>0.05, |v|<0.05) ===')
    rotating = [r for r in rows if r[1] == '/cmd_vel' and abs(r[3]) > 0.05 and abs(r[2]) < 0.05]
    if rotating:
        print(f'  {rotating[0][0]:.1f}s ~ {rotating[-1][0]:.1f}s, 샘플 {len(rotating)}개, '
              f'w 평균 {sum(abs(r[3]) for r in rotating)/len(rotating):+.2f} rad/s')
    else:
        print('  없음 — /cmd_vel 에 회전 명령이 거의 안 나감 (명령 자체가 문제일 수 있음)')

    # 2) 회전 구간 동안 각 신호의 누적 변화 비교 (핵심 진단)
    if rotating:
        a, b = rotating[0][0], rotating[-1][0]
        dur = max(b - a, 1e-3)
        seg = [r for r in rows if a <= r[0] <= b]

        def yaw_change(key):
            vals = [r[2] for r in seg if r[1] == key]
            if len(vals) < 2:
                return None
            # unwrap
            tot, prev = 0.0, vals[0]
            for v in vals[1:]:
                d = v - prev
                while d > 180:  d -= 360
                while d < -180: d += 360
                tot += d
                prev = v
            return tot

        cmd_int = sum(abs(r[3]) for r in seg if r[1] == '/cmd_vel') * 0.1  # ~10Hz 가정
        imu_int = 0.0
        imu_vals = [(r[0], r[2]) for r in seg if r[1] == 'imu']
        for i in range(1, len(imu_vals)):
            dt = imu_vals[i][0] - imu_vals[i-1][0]
            imu_int += math.degrees(imu_vals[i][1]) * dt
        print(f'\n=== 회전 구간 {a:.1f}~{b:.1f}s ({dur:.1f}s) 동안 누적 회전(deg) ===')
        print(f'  명령(cmd_vel ∫|w|, 대략) : {math.degrees(cmd_int):+8.1f}°')
        print(f'  IMU 자이로 ∫z            : {imu_int:+8.1f}°   ← 실제 물리 회전(자이로 기준)')
        ob = yaw_change('odom->base')
        mo = yaw_change('map->odom')
        ek = yaw_change('ekf')
        print(f'  odom->base yaw 변화      : {("%+8.1f°" % ob) if ob is not None else "   (없음)"}   ← EKF 가 회전 반영했나')
        print(f'  EKF(/odometry/filtered)  : {("%+8.1f°" % ek) if ek is not None else "   (없음)"}')
        print(f'  map->odom yaw 변화       : {("%+8.1f°" % mo) if mo is not None else "   (없음)"}   ← AMCL 보정/상쇄량')
        print('\n해석:')
        print('  · IMU 큰데 odom->base ~0  → EKF 가 자이로 yaw 적분 못함 (EKF/입력 문제)')
        print('  · IMU≈odom->base 인데 map(=odom->base + map->odom)에서 안 돎')
        print('    → map->odom 이 반대로 변함 = AMCL 이 회전 상쇄/발산 (AMCL 문제)')
        print('  · 명령은 큰데 IMU≈0       → 로봇이 실제로 안 돎 (cmd_vel 충돌/모터)')

    # 3) cmd_vel 타임라인 일부 (충돌·끊김 확인용): 회전 구간 앞뒤로 샘플
    if win:
        print('\n=== /cmd_vel & /cmd_vel_smoothed 타임라인 (구간 내) ===')
        for r in rows:
            if r[1] in ('/cmd_vel', '/cmd_vel_smoothed'):
                print(f'  {r[0]:7.2f}s {r[1]:18s} v={r[2]:+.2f} w={r[3]:+.2f}')


if __name__ == '__main__':
    main()
