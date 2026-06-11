#!/usr/bin/env python3
"""
lidar_approach.py — 목표(물건)까지 Nav2 로 벽 피해 다가간 뒤, 마지막만 라이다로 40cm 접근.

문제: 벽에 붙은 물건은 ① Nav2 플래너가 벽 근처를 거부하고 ② 단순 직선접근은 중간 벽에
      걸림. 그래서 2단계로 나눈다.

2단계 동작:
  [NAVIGATING] 목표에서 살짝 물러난 **자유공간 staging 지점**을 맵에서 찾아 Nav2 로 이동
               (Nav2 가 중간 벽을 피해 경로계획). staging = 목표 바라보는 자세.
  [APPROACHING] staging 도착(=목표 근처) 후에만 반응형 전진. 앞 라이다가 stop_distance
               (기본 0.40m) 이하가 되면 정지 → 그 자리서 팔로 잡음.

즉 40cm 정지는 **목표 근처(staging 도착 후)에서만** 조건적으로 적용된다.

미션 흐름 (상태머신):
  목표수신 → NAVIGATING → APPROACHING → [도착] ARRIVED
    → /approach_done(Bool) 발행, 멈춰서 파지 신호 대기
    → /grasp_done(Bool) 수신 → RETURNING (시작 시 기록한 초기위치로 Nav2 복귀) → HOME
  초기위치 = 노드 시작 시점의 로봇 위치(자동 기록).

전제: realmap_sim.launch.py (Gazebo + AMCL + Nav2 + /map) 가 떠 있을 것.
목표 입력: 기본 토픽 /target_bottle_position (geometry_msgs/PointStamped, frame_id: map).
          RViz "Publish Point"(/clicked_point) 로도 테스트 가능. 토픽 바꾸려면 -p target_topic:=...

사용:
  python3 scripts/lidar_approach.py
  # 비전 노드가 /target_bottle_position 으로 물건 좌표 발행, 또는 수동 테스트:
  ros2 topic pub --once /target_bottle_position geometry_msgs/msg/PointStamped \
    "{header: {frame_id: 'map'}, point: {x: -2.836, y: 1.31, z: 0.0}}"
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
import tf2_ros


def normalize(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class LidarApproach(Node):
    def __init__(self):
        super().__init__('lidar_approach')
        self.declare_parameter('stop_distance', 0.45)   # 라이다 정지 거리(m) — 앞 물체 이 거리에서 정지
        self.declare_parameter('clearance', 0.55)       # staging 이 벽에서 떨어질 거리(m). stop_distance 보다 커야 그 거리까지 다가가 멈춤. (너무 크면 좁은 곳서 staging 못 찾음)
        self.declare_parameter('wall_scan_radius', 0.55) # 목표 주변 이 반경(m)의 점유셀로 책상/벽 방향(법선)을 추정 → 면과 수직 접근. 0이면 끔. ★물건이 책상 위 깊이 찍히면 면에서 멀어져 작아선 안 됨(0.40이면 셀<6 폴백). 면까지 닿게 0.55+.
        self.declare_parameter('standoff', 0.50)         # 책상 앞면에서 로봇 중심까지 정지 거리(m). 책상 위 물건은 여기서 멈추고 팔로 집음. 회전반경보다 커야 회전 시 안 부딪힘.
        self.declare_parameter('rotate_radius', 0.45)    # 제자리 회전 시 footprint 외접 반경(m). staging 이 실제 장애물에서 이만큼 떨어졌는지 검사 — 회전 충돌 방지.
        self.declare_parameter('align_yaw_deg', 3.0)     # 수직접근 도착 후 제자리 정밀정렬 허용오차(도). 회전속도 높여서 오실레이션 줄이려 2→3°.
        self.declare_parameter('align_min_w', 0.40)      # 정밀정렬 최소 각속도(rad/s). 50kg 정지마찰 극복용. 0.15는 약해서 거의 안 돎 → 0.40. 오실레이션 나면 낮추기.
        self.declare_parameter('align_fov_deg', 60.0)    # 정밀정렬 시 앞 벽 직선을 맞출 라이다 시야각(도). 눈앞 벽만 보게 좁게.
        self.declare_parameter('align_max_range', 1.2)   # 정밀정렬 시 이 거리(m) 안의 스캔점만 사용. 멀리 다른 벽 무시.
        self.declare_parameter('align_timeout', 15.0)    # 정밀정렬 최대 시간(s). 넘으면 그냥 도착 처리(무한 회전 방지).
        self.declare_parameter('approach_run', 0.30)     # 수직접근: Nav2 가 최종 standoff 보다 이만큼 더 뒤에 멈춤 → 그 거리만큼 마지막에 곧장 직진. 직진 구간 길이.
        self.declare_parameter('pause_sec', 1.5)         # 수직접근: 정렬(회전) 끝나고 직진 시작 전 멈춰 쉬는 시간(s). odom 드리프트 안정화 — 회전·직진 분리.
        self.declare_parameter('forward_timeout', 8.0)   # 수직접근 마지막 직진 최대 시간(s). 넘으면 정지(라이다 정지 못 잡을 때 안전).
        self.declare_parameter('front_fov_deg', 50.0)
        self.declare_parameter('linear_speed', 0.12)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('align_tol', 0.10)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('target_topic', '/target_bottle_position')  # 목표(물건) 위치 토픽
        self.declare_parameter('done_topic', '/approach_done')   # 도착(파지 준비) 알림 발행 토픽
        self.declare_parameter('grasp_topic', '/grasp_done')     # 파지 완료 수신 → 초기위치 복귀
        # 속도명령 토픽. /cmd_vel 직접 = Nav2 체인(velocity_smoother→collision_monitor) 우회하고
        #   lidar_approach 가 직접 소유. Nav2 액션 완료 후엔 체인이 곧 조용해지므로(stop_pub_timeout~2s)
        #   접근 단계엔 단독 발행이 됨. (체인과 섞여 회전이 느리면 이게 맞음.)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        # 복귀 도착 시 시작 자세 대비 추가 회전(도). 180=처음과 반대(-y, 사용자요청). 0=뒤집기 끔
        # (집 도착 후 큰 제자리 회전이 AMCL 흔들면 0 으로 테스트).
        self.declare_parameter('return_flip_deg', 180.0)
        # 파지 후 복귀 전, 책상에서 회전 없이 곧장 뒤로 빠질 거리(m). 코앞 제자리회전→AMCL 발산을
        # 회피(열린 공간 확보 후 돌게). 0이면 끔(바로 복귀).
        self.declare_parameter('back_away_dist', 0.40)
        self.stop_dist = float(self.get_parameter('stop_distance').value)
        self.clearance = float(self.get_parameter('clearance').value)
        self.wall_scan_radius = float(self.get_parameter('wall_scan_radius').value)
        self.standoff = float(self.get_parameter('standoff').value)
        self.rot_radius = float(self.get_parameter('rotate_radius').value)
        self.align_yaw_tol = math.radians(float(self.get_parameter('align_yaw_deg').value))
        self.align_min_w = float(self.get_parameter('align_min_w').value)
        self.align_fov = math.radians(float(self.get_parameter('align_fov_deg').value))
        self.align_max_range = float(self.get_parameter('align_max_range').value)
        self.align_timeout = float(self.get_parameter('align_timeout').value)
        self.approach_run = float(self.get_parameter('approach_run').value)
        self.pause_sec = float(self.get_parameter('pause_sec').value)
        self.forward_timeout = float(self.get_parameter('forward_timeout').value)
        self.fov = math.radians(float(self.get_parameter('front_fov_deg').value))
        self.v = float(self.get_parameter('linear_speed').value)
        self.w = float(self.get_parameter('angular_speed').value)
        self.align_tol = float(self.get_parameter('align_tol').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.return_flip = math.radians(float(self.get_parameter('return_flip_deg').value))
        self.back_away_dist = float(self.get_parameter('back_away_dist').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.marker_pub = self.create_publisher(Marker, '/target_marker', 10)  # RViz 시각화
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        target_topic = self.get_parameter('target_topic').value
        # ★ 라이다는 BEST_EFFORT(센서 QoS) 발행 — RELIABLE 로 구독하면 메시지 0개라 inf 됨
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PointStamped, target_topic, self.on_target, 10)
        # RViz "Publish Point" 로도 테스트 가능하게 /clicked_point 도 함께 구독
        if target_topic != '/clicked_point':
            self.create_subscription(PointStamped, '/clicked_point', self.on_target, 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map,
                                 QoSProfile(depth=1,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # 도착 알림 발행 + 파지 완료 수신(→ 초기위치 복귀)
        self.done_topic = self.get_parameter('done_topic').value
        self.grasp_topic = self.get_parameter('grasp_topic').value
        self.done_pub = self.create_publisher(Bool, self.done_topic, 10)
        self.create_subscription(Bool, self.grasp_topic, self.on_grasp, 10)

        self.grid = None
        self.scan = None              # 최신 LaserScan (수직정렬용)
        self.align_t0 = None          # 정밀정렬 시작 시각(s) — 타임아웃용
        self.pause_t0 = None          # 정렬 후 정지(쉼) 시작 시각(s)
        self.forward_t0 = None        # 마지막 직진 시작 시각(s) — 타임아웃용
        self.front_min = float('inf')
        self.target = None
        self.home = None              # 초기 위치 (시작 시 1회 기록) — 복귀 목표
        self.perp_mode = False        # 이번 목표가 책상/벽 면 수직접근인지(=라이다 추가전진 생략, staging 도착 후 정밀정렬)
        self.goal_yaw = None          # 도착 후 맞출 목표 방향(수직접근 시 책상 면 법선의 반대 = 면 정면)
        self.back_start = None        # 후진 시작 위치(복귀 전 책상에서 빠질 때 거리 측정용)
        self.return_goal = None       # 후진 끝나고 보낼 복귀 목표 (hx, hy, yaw)
        self.state = 'IDLE'           # IDLE/NAVIGATING/APPROACHING/ALIGNING/PAUSING/FORWARD/ARRIVED/BACKING/RETURNING/HOME
        self.create_timer(0.1, self.control)
        self.get_logger().info(
            f'lidar_approach 시작 — 목표[{target_topic}] → Nav2+라이다 {self.stop_dist}m 접근, '
            f'도착시 [{self.done_topic}] 발행, [{self.grasp_topic}] 수신시 초기위치 복귀.')

    # ---------- 입력 ----------
    def on_map(self, msg):
        self.grid = msg
        self.gw = msg.info.width
        self.gh = msg.info.height
        self.res = msg.info.resolution
        self.ox = msg.info.origin.position.x
        self.oy = msg.info.origin.position.y

    def on_scan(self, msg):
        self.scan = msg                  # 원본 스캔 보관 (수직정렬에 사용)
        half = self.fov / 2.0
        best = float('inf')
        for i, r in enumerate(msg.ranges):
            a = msg.angle_min + i * msg.angle_increment
            if -half <= a <= half and msg.range_min < r < msg.range_max and math.isfinite(r):
                best = min(best, r)
        self.front_min = best

    def publish_marker(self, x, y):
        """목표 위치를 RViz 에 빨간 구 Marker 로 표시 (/target_marker)."""
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'target'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = 0.1
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.2
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.1, 0.1, 1.0
        self.marker_pub.publish(m)

    def on_target(self, msg):
        if self.grid is None:
            self.get_logger().warn('/map 아직 수신 안 됨 — 잠시 후 다시 목표 발행')
            return
        self.target = (msg.point.x, msg.point.y)
        self.publish_marker(msg.point.x, msg.point.y)   # RViz 에 목표 표시
        pose = self.robot_pose()
        if pose is None:
            self.get_logger().warn('TF(map->base) 없음 — 목표 무시')
            return
        stage = self.find_staging(self.target[0], self.target[1], pose[0], pose[1])
        if stage is None:
            self.get_logger().error('목표 근처에 접근 가능한 staging 지점 없음 (벽으로 둘러싸임?)')
            return
        sx, sy, yaw = stage
        self.send_nav_goal(sx, sy, yaw)

    # ---------- 맵 충돌 검사 ----------
    def occupied(self, x, y):
        """world (x,y) 가 점유/미지면 True (= 못 감)."""
        mx = int((x - self.ox) / self.res)
        my = int((y - self.oy) / self.res)
        if not (0 <= mx < self.gw and 0 <= my < self.gh):
            return True
        v = self.grid.data[my * self.gw + mx]
        return v < 0 or v >= 50      # 미지(-1) 또는 점유

    def is_free(self, x, y):
        """(x,y) 가 확실한 자유공간(0)인지. 미지·점유는 False."""
        mx = int((x - self.ox) / self.res)
        my = int((y - self.oy) / self.res)
        if not (0 <= mx < self.gw and 0 <= my < self.gh):
            return False
        v = self.grid.data[my * self.gw + mx]
        return 0 <= v < 50

    def footprint_clear(self, x, y, radius):
        """(x,y) 중심 radius 안에 '실제 장애물(점유셀)' 이 없는지.
        미지(-1)는 통과시킴 — 책상 위처럼 벽 바깥이 미지로 찍힌 좁은 구석에서 과도하게 거부하지 않게.
        충돌 방지는 점유셀 기준으로만 (실제 벽/책상에만 안 부딪히면 됨)."""
        steps = max(1, int(radius / self.res))
        for i in range(-steps, steps + 1):
            for j in range(-steps, steps + 1):
                if math.hypot(i, j) * self.res <= radius:
                    mx = int((x + i * self.res - self.ox) / self.res)
                    my = int((y + j * self.res - self.oy) / self.res)
                    if 0 <= mx < self.gw and 0 <= my < self.gh:
                        if self.grid.data[my * self.gw + mx] >= 50:   # 점유셀만 충돌로 봄
                            return False
        return True

    def wall_normal(self, tx, ty):
        """목표 주변 점유셀(책상/벽)에 직선을 PCA 로 맞춰, 자유공간 쪽 법선(단위벡터) 반환.
        벽을 신뢰성 있게 못 찾으면 None → 호출부가 로봇쪽 접근으로 폴백."""
        if self.wall_scan_radius <= 0.0:
            return None
        R = self.wall_scan_radius
        steps = max(2, int(R / self.res))
        xs, ys = [], []
        for i in range(-steps, steps + 1):
            for j in range(-steps, steps + 1):
                if math.hypot(i, j) * self.res > R:
                    continue
                x, y = tx + i * self.res, ty + j * self.res
                mx = int((x - self.ox) / self.res)
                my = int((y - self.oy) / self.res)
                if 0 <= mx < self.gw and 0 <= my < self.gh:
                    if self.grid.data[my * self.gw + mx] >= 50:   # 점유셀만(미지 제외)
                        xs.append(x); ys.append(y)
        n = len(xs)
        if n < 6:                       # 점유셀 너무 적음 → 벽 아님(허공의 목표)
            return None
        mx = sum(xs) / n; my = sum(ys) / n
        sxx = sum((x - mx) ** 2 for x in xs) / n
        syy = sum((y - my) ** 2 for y in ys) / n
        sxy = sum((xs[k] - mx) * (ys[k] - my) for k in range(n)) / n
        # 2x2 대칭행렬 최소고유값 = 벽 법선 방향(점들이 퍼진 선의 수직)
        tr = sxx + syy
        disc = math.sqrt(max(0.0, (tr / 2) ** 2 - (sxx * syy - sxy * sxy)))
        l_small = tr / 2 - disc
        l_large = tr / 2 + disc
        nx, ny = sxy, l_small - sxx
        if math.hypot(nx, ny) < 1e-9:
            nx, ny = l_small - syy, sxy
        norm = math.hypot(nx, ny)
        if norm < 1e-9:
            return None
        nx, ny = nx / norm, ny / norm
        # 직선성: 큰고유값/작은고유값. 1 에 가까우면 점들이 뭉쳐있어(코너·블록) 법선이 못 믿을 값.
        ratio = l_large / l_small if l_small > 1e-9 else float('inf')
        self.get_logger().info(
            f'벽 법선 추정: n=({nx:+.2f},{ny:+.2f}) 면방향 {math.degrees(math.atan2(-nx, ny)):+.0f}°, '
            f'점유셀 {n}개, 직선성 {ratio:.1f}{"  ⚠️낮음(면 아님?)" if ratio < 3.0 else ""}')
        if ratio < 2.0:
            self.get_logger().warn(
                '직선성 너무 낮음 → 깔끔한 벽면이 아님(코너/뭉친 점). 법선 신뢰 불가 → 개방공간 접근으로 폴백')
            return None
        # 법선의 ± 중 '자유공간 쪽' 부호 선택 (책상 반대편)
        for sign in (1.0, -1.0):
            if not self.occupied(tx + sign * self.clearance * nx,
                                 ty + sign * self.clearance * ny):
                return sign * nx, sign * ny
        return None

    def find_staging(self, tx, ty, rx, ry):
        """staging(로봇이 설 자리)과 도착 자세(yaw)를 (sx, sy, yaw) 로 반환. 없으면 None.

        책상/벽 면을 찾으면(perp_mode): 면의 '앞면'에서 standoff 만큼 앞에, 면과 수직으로 정렬.
        좁으면 면을 따라 좌우로 옮겨 들어갈 자리를 찾음. → 책상 위 물건을 정면·안전거리에서 집음.
        면을 못 찾으면(개방공간 목표): 예전처럼 로봇 쪽 방향으로 물러나며 자리 탐색."""
        n = self.wall_normal(tx, ty)
        if n is None:
            # 폴백: 개방공간 목표 — 로봇이 있는 쪽으로 물러나며 자리 찾기
            self.perp_mode = False
            d = math.hypot(rx - tx, ry - ty)
            if d < 1e-3:
                return None
            ux, uy = (rx - tx) / d, (ry - ty) / d
            self.get_logger().warn('벽 법선 추정 실패 → 개방공간 접근(로봇 방향, 비스듬할 수 있음)')
            back = self.clearance
            while back <= 2.5:
                sx, sy = tx + back * ux, ty + back * uy
                if self.footprint_clear(sx, sy, self.rot_radius):
                    yaw = math.atan2(ty - sy, tx - sx)   # 목표 바라봄
                    return sx, sy, yaw
                back += 0.05
            return None

        # 수직 접근 모드
        self.perp_mode = True
        nx, ny = n                     # 책상 면의 법선(자유공간 쪽)
        # 1) 책상 '앞면' 찾기: 목표에서 법선 따라 나가며 첫 자유셀
        fx = fy = None
        s = 0.0
        while s <= 1.5:
            x, y = tx + s * nx, ty + s * ny
            if self.is_free(x, y):
                fx, fy = x, y
                break
            s += self.res
        if fx is None:
            self.get_logger().error('책상 앞 자유공간을 못 찾음(법선 방향이 막힘)')
            return None
        yaw = math.atan2(-ny, -nx)     # 면을 정면으로(법선 반대) 바라봄 → 항상 수직
        # 2) Nav2 정지 지점: 앞면에서 (standoff+approach_run) 뒤. 그만큼 마지막에 곧장 직진(라이다 정지).
        #    좁으면 면(접선) 따라 좌우로 옮겨 가장 가까운 자리.
        nav_dist = self.standoff + self.approach_run
        tgx, tgy = -ny, nx             # 접선(벽을 따라가는 방향)
        lat = 0.0
        while lat <= 0.50:
            for sgn in ((0.0,) if lat == 0.0 else (1.0, -1.0)):
                sx = fx + nav_dist * nx + sgn * lat * tgx
                sy = fy + nav_dist * ny + sgn * lat * tgy
                if self.footprint_clear(sx, sy, self.rot_radius):
                    if lat > 0.01:
                        self.get_logger().info(f'정면 자리 좁아 면 따라 {sgn*lat:+.2f}m 이동해 정렬')
                    self.get_logger().info(
                        f'벽 법선 수직접근 — 앞면({fx:.2f},{fy:.2f})에서 {nav_dist:.2f}m 뒤로 Nav2 이동 '
                        f'→ 정렬·{self.pause_sec:.1f}s쉼·직진({self.stop_dist:.2f}m 라이다정지)')
                    return sx, sy, yaw
            lat += 0.05
        self.get_logger().error('책상 앞 Nav2 정지자리가 회전반경만큼 안 비어있음(주변 너무 좁음)')
        return None

    # ---------- Nav2 ----------
    def send_nav_goal(self, x, y, yaw):
        if not self.nav.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose 서버 없음 — Nav2 떠 있나 확인')
            return
        p = PoseStamped()
        p.header.frame_id = self.map_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = x, y
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        goal = NavigateToPose.Goal()
        goal.pose = p
        self.goal_yaw = yaw          # 도착 후 정밀정렬용 목표 방향 기억
        self.state = 'NAVIGATING'
        self.get_logger().info(f'[1단계] staging({x:.2f},{y:.2f}) 로 Nav2 이동 (벽 회피)')
        self.nav.send_goal_async(goal).add_done_callback(self._nav_accepted)

    def _nav_accepted(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().warn('staging 목표 거부됨')
            self.state = 'IDLE'
            return
        gh.get_result_async().add_done_callback(self._nav_done)

    def _nav_done(self, fut):
        # ★ Nav2 가 실제로 staging 에 '성공' 도착했을 때만 접근 단계로. 실패/취소/선점(preempt)이면
        #   여기서 접근 시작하면 엉뚱한 데(멀리)서 전진함 → SUCCEEDED 아닌 경우 중단.
        if fut.result().status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                f'staging 도착 실패(Nav2 status={fut.result().status}) — 접근 안 함. '
                f'(Nav2 가 목표를 못 갔거나 목표가 다시 발행됨) 목표 다시 주거나 Nav2 확인.')
            self.state = 'IDLE'
            return
        # 수직모드든 개방공간이든 동일하게 반응형 전진 접근.
        # 수직성은 staging 위치(벽 법선 위)로 이미 보장됨 → 거기서 목표 향해 직진하면 법선 따라가 수직 유지.
        # ★ 제자리 회전(ALIGNING)은 코앞 회전 시 AMCL 흔들려 프레임 멈춤/발산 유발 → 안 씀.
        self.state = 'APPROACHING'
        mode = '수직(법선 위에서 정면 전진)' if self.perp_mode else '개방공간'
        self.get_logger().info(f'[2단계] staging 도착({mode}) → 라이다 보며 {self.stop_dist}m 까지 전진')

    # ---------- 파지 완료 → 초기 위치 복귀 ----------
    def on_grasp(self, msg):
        if not msg.data:
            return
        if self.state != 'ARRIVED':
            self.get_logger().warn(
                f'파지 완료 신호 받았으나 상태가 {self.state} — 무시 (도착 후에만 복귀)')
            return
        if self.home is None:
            self.get_logger().error('초기 위치 기록이 없어 복귀 불가')
            return
        hx, hy, hyaw = self.home
        # 복귀 도착 자세 = 시작자세 + return_flip(기본 180°=-y). 0 이면 시작과 같은 방향(큰 회전 없음).
        return_yaw = normalize(hyaw + self.return_flip)
        self.return_goal = (hx, hy, return_yaw)
        # ★ 바로 복귀(=책상 코앞 180° 회전)하면 라이다가 책상에 꽉 차 AMCL 발산. 먼저 회전 없이
        #   곧장 후진해 책상에서 벗어난 뒤(열린 공간) 복귀. back_away_dist=0 이면 바로 복귀.
        if self.back_away_dist > 0.0:
            p = self.robot_pose()
            self.back_start = (p[0], p[1]) if p is not None else None
            self.state = 'BACKING'
            self.get_logger().info(
                f'✋ 파지 완료 → 책상에서 {self.back_away_dist:.2f}m 곧장 후진 후 복귀 (코앞 회전 회피)')
        else:
            self.send_return_goal(hx, hy, return_yaw)

    def send_return_goal(self, x, y, yaw):
        if not self.nav.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose 서버 없음 — 복귀 실패')
            return
        p = PoseStamped()
        p.header.frame_id = self.map_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = x, y
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        goal = NavigateToPose.Goal()
        goal.pose = p
        self.target = None            # 접근 목표 해제 (반응형 정지)
        self.state = 'RETURNING'
        self.nav.send_goal_async(goal).add_done_callback(self._return_accepted)

    def _return_accepted(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().warn('복귀 목표 거부됨')
            return
        gh.get_result_async().add_done_callback(self._return_done)

    def _return_done(self, fut):
        if fut.result().status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'복귀 실패(Nav2 status={fut.result().status}).')
            return
        self.state = 'HOME'
        self.get_logger().info('🏠 초기 위치 복귀 완료. 다음 목표 대기.')

    def front_wall_yaw_err(self):
        """앞쪽 라이다 점들에 직선(벽)을 맞춰, '로봇이 벽과 수직이 되려면 얼마나 더 돌아야 하는지'(rad) 반환.
        0 이면 이미 벽 정면. AMCL 과 무관하게 '진짜 눈앞 벽' 기준이라 위치추정 오차에 안 흔들림.
        깔끔한 벽을 못 읽으면 None (→ 호출부가 맵 yaw 로 폴백)."""
        msg = self.scan
        if msg is None:
            return None
        half = self.align_fov / 2.0
        xs, ys = [], []
        for i, r in enumerate(msg.ranges):
            a = msg.angle_min + i * msg.angle_increment
            if (-half <= a <= half and msg.range_min < r < msg.range_max
                    and math.isfinite(r) and r <= self.align_max_range):
                xs.append(r * math.cos(a)); ys.append(r * math.sin(a))   # 로봇좌표(앞=+x, 좌=+y)
        n = len(xs)
        if n < 8:
            return None
        mx = sum(xs) / n; my = sum(ys) / n
        sxx = sum((x - mx) ** 2 for x in xs) / n
        syy = sum((y - my) ** 2 for y in ys) / n
        sxy = sum((xs[k] - mx) * (ys[k] - my) for k in range(n)) / n
        tr = sxx + syy
        disc = math.sqrt(max(0.0, (tr / 2) ** 2 - (sxx * syy - sxy * sxy)))
        l_large = tr / 2 + disc; l_small = tr / 2 - disc
        if l_small > 1e-9 and l_large / l_small < 3.0:   # 직선이 아님(벽 아님) → 신뢰 불가
            return None
        # 벽 방향(최대고유벡터) → 법선
        dx, dy = sxy, l_large - sxx
        if math.hypot(dx, dy) < 1e-9:
            dx, dy = l_large - syy, sxy
        dn = math.hypot(dx, dy)
        if dn < 1e-9:
            return None
        dx, dy = dx / dn, dy / dn
        nx, ny = -dy, dx                      # 벽 법선
        if nx * mx + ny * my < 0:             # 벽(앞쪽) 향하도록 부호
            nx, ny = -nx, -ny
        # 로봇 정면(+x)이 법선과 일치할 때 수직. 법선의 로봇좌표 각도 = 더 돌아야 할 양.
        return math.atan2(ny, nx)

    # ---------- 제어 ----------
    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            return t.transform.translation.x, t.transform.translation.y, yaw
        except tf2_ros.TransformException:
            return None

    def control(self):
        if self.home is None:                       # 초기 위치 1회 기록 (복귀용)
            p = self.robot_pose()
            if p is not None:
                self.home = p
                self.get_logger().info(
                    f'초기 위치 기록: ({p[0]:.2f}, {p[1]:.2f}, {math.degrees(p[2]):.0f}°)')
        # 수직접근 도착 후 정밀정렬: 라이다로 '진짜 앞 벽'에 수직이 되게 제자리 회전 (AMCL 무관).
        if self.state == 'ALIGNING':
            src = '라이다'
            err = self.front_wall_yaw_err()
            if err is None:                     # 라이다로 벽 못 읽음 → 맵 yaw 로 폴백
                pose = self.robot_pose()
                if pose is None or self.goal_yaw is None:
                    self.cmd_pub.publish(Twist())
                    return
                err = normalize(self.goal_yaw - pose[2])
                src = '맵yaw(폴백)'
            now = self.get_clock().now().nanoseconds / 1e9
            timed_out = self.align_t0 is not None and (now - self.align_t0) > self.align_timeout
            if abs(err) < self.align_yaw_tol or timed_out:
                self.cmd_pub.publish(Twist())
                tag = ' ⏱️타임아웃' if (timed_out and abs(err) >= self.align_yaw_tol) else ''
                self.get_logger().info(
                    f'✅ 앞 벽 수직정렬 완료({src}, 잔여 {math.degrees(err):+.1f}°){tag} '
                    f'→ {self.pause_sec:.1f}s 정지 후 직진 (회전·직진 분리)')
                self.pause_t0 = now
                self.state = 'PAUSING'
                return
            cmd = Twist()                       # 제자리 회전만 (전진 0)
            wz = max(-self.w, min(self.w, 1.2 * err))
            if abs(wz) < self.align_min_w:      # 최소 각속도 보장(정지마찰 극복)
                wz = math.copysign(self.align_min_w, err)
            cmd.angular.z = wz
            self.get_logger().info(f'정밀정렬 중({src}) — 잔여 {math.degrees(err):+.1f}°',
                                   throttle_duration_sec=0.5)
            self.cmd_pub.publish(cmd)
            return

        # 정렬 후 쉼: 회전 멈추고 odom 안정화될 때까지 정지 (회전→직진 사이 텀)
        if self.state == 'PAUSING':
            self.cmd_pub.publish(Twist())
            now = self.get_clock().now().nanoseconds / 1e9
            if self.pause_t0 is not None and (now - self.pause_t0) >= self.pause_sec:
                self.forward_t0 = now
                self.state = 'FORWARD'
                self.get_logger().info(f'직진 시작 → 앞 라이다 {self.stop_dist}m 까지 곧장 접근')
            return

        # 마지막 직진: 회전 없이 곧장 전진(드리프트 최소), 앞 라이다 ≤ stop_distance 면 정지(맵오차 무관).
        if self.state == 'FORWARD':
            now = self.get_clock().now().nanoseconds / 1e9
            timed_out = self.forward_t0 is not None and (now - self.forward_t0) > self.forward_timeout
            if self.front_min <= self.stop_dist or timed_out:
                self.cmd_pub.publish(Twist())
                tag = ' ⏱️타임아웃' if timed_out and self.front_min > self.stop_dist else ''
                self.get_logger().info(
                    f'🎯 도착 — 앞 라이다 {self.front_min:.2f}m{tag}. '
                    f'[{self.done_topic}] 발행 → 파지 대기')
                self.done_pub.publish(Bool(data=True))
                self.state = 'ARRIVED'
                return
            cmd = Twist()
            cmd.linear.x = self.v               # 회전 0, 곧장 전진만
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f'직진 중 — 앞 라이다 {self.front_min:.2f}m',
                                   throttle_duration_sec=0.5)
            return

        # 파지 후 복귀 전: 회전 없이 곧장 후진해 책상에서 벗어남(코앞 제자리회전 AMCL발산 회피).
        if self.state == 'BACKING':
            pose = self.robot_pose()
            if pose is None or self.back_start is None:
                self.cmd_pub.publish(Twist())   # 위치 못 읽으면 후진 생략하고 바로 복귀
                if self.return_goal is not None:
                    self.send_return_goal(*self.return_goal)
                return
            dist = math.hypot(pose[0] - self.back_start[0], pose[1] - self.back_start[1])
            if dist >= self.back_away_dist:
                self.cmd_pub.publish(Twist())
                self.get_logger().info(f'후진 {dist:.2f}m 완료 → 열린 공간에서 Nav2 복귀 시작')
                self.send_return_goal(*self.return_goal)
                return
            cmd = Twist()
            cmd.linear.x = -self.v              # 회전 0, 곧장 후진만
            self.cmd_pub.publish(cmd)
            return

        if self.state != 'APPROACHING' or self.target is None:
            return
        # 목표 근처(2단계)에서만 라이다 정지 적용
        if self.front_min <= self.stop_dist:
            self.cmd_pub.publish(Twist())
            self.get_logger().info(
                f'🎯 도착 — 라이다 {self.front_min:.2f}m. [{self.done_topic}] 발행 → 파지 대기')
            self.done_pub.publish(Bool(data=True))   # 도착 알림 (팔이 이걸 받고 파지)
            self.state = 'ARRIVED'
            return
        pose = self.robot_pose()
        if pose is None:
            self.cmd_pub.publish(Twist())
            return
        rx, ry, yaw = pose
        tx, ty = self.target
        heading_err = normalize(math.atan2(ty - ry, tx - rx) - yaw)
        cmd = Twist()
        # 부드러운 추종: 회전·전진 동시. 제자리 회전으로 안 빠져서 떨림 방지.
        if abs(heading_err) < 0.05:          # 데드밴드(~3°): 미세오차엔 회전 안 함
            cmd.angular.z = 0.0
        else:
            cmd.angular.z = max(-self.w, min(self.w, 0.8 * heading_err))  # 낮은 gain
        if abs(heading_err) < 0.7:           # ~40° 안으로 향하면 전진(정렬되며 따라감)
            cmd.linear.x = self.v * max(0.3, math.cos(heading_err))
        self.get_logger().info(
            f'접근 중 — 목표까지 {math.hypot(tx-rx,ty-ry):.2f}m, 앞 라이다 {self.front_min:.2f}m',
            throttle_duration_sec=1.0)
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = LidarApproach()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
