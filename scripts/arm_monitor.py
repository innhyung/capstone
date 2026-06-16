#!/usr/bin/env python3
"""
arm_monitor.py — 로봇팔 grasp 미션 상태를 한 터미널에 깔끔히 보여주는 대시보드.

왜: pointcloud_all.py(팔 노드)는 로그가 폭포처럼 쏟아져 눈으로 따라갈 수 없음.
    이 모니터는 팔 노드가 도메인 77에 뿌리는 /rosout 로그를 "파싱"해서 핵심 상태만
    뽑아 0.5초마다 화면을 갱신한다. 거기에 /joint_states 를 직접 구독해 특이점(singularity)
    근접도를 실시간 계산한다.

무엇을 보여주나:
  - X/Y 보정 상태 (error_x, error_y, 허용오차 안인지)
  - 경로 생성 후 전진해야 할 Z 거리 (raw=병중심까지 / actual=ready까지 전진)
  - READY(=물병 7cm 앞, 직진 파지 직전) 분해:
      · ready 까지 전진값      (actual)
      · ready→물병 남은 거리   (grasp_margin = raw - actual, 보통 7cm)
      · 실제 ready 이동값       (Cartesian fraction × actual = 진짜 움직인 양)
  - 어느 번호 joint 가 특이점에 빠지는지 (elbow=J3, wrist=J5 근접 각도)

실행 (별도 터미널, 도메인 77 자동 설정):
  source /opt/ros/jazzy/setup.bash && python3 ~/capstone/scripts/arm_monitor.py

  (팔 노드와 같은 도메인 77 로 강제. 다른 도메인을 보려면 ARM_DOMAIN=NN 환경변수.)
"""
import os
# 팔 노드(run_arm.sh)는 도메인 77. .bashrc 의 28 을 덮어써 같은 도메인에서 본다.
os.environ['ROS_DOMAIN_ID'] = os.environ.get('ARM_DOMAIN', '77')

import re
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import Log
from sensor_msgs.msg import JointState

ARM_NODE = 'pc_ransac_bottle_table_test'   # 팔 노드 이름 (로그 name 필드로 필터)

# UR3 joint 이름 → 사람이 보는 번호(1~6)
JOINT_NUM = {
    'shoulder_pan_joint': 1,
    'shoulder_lift_joint': 2,
    'elbow_joint': 3,
    'wrist_1_joint': 4,
    'wrist_2_joint': 5,
    'wrist_3_joint': 6,
}

# 각 로그 줄에서 값을 뽑는 정규식 (팔 노드 로그 문자열과 1:1)
RE = {
    'xy_corr':   re.compile(r'XY 보정: error_x=(-?\d+), error_y=(-?\d+)'),
    'xy_track':  re.compile(r'YOLO XY TRACK: bbox=\((-?\d+),(-?\d+)\) img=\((-?\d+),(-?\d+)\) '
                            r'error_x=(-?\d+) error_y=(-?\d+) \(tolX=(\d+) tolY=(\d+)\)'),
    'fwd_calc':  re.compile(r'전진거리 재계산: raw=([\d.]+)cm, actual=([\d.]+)cm'),
    'fwd_goal':  re.compile(r'FORWARD .*목표: x=(-?[\d.]+), y=(-?[\d.]+), z=(-?[\d.]+)'),
    'cart_run':  re.compile(r'Cartesian 직선 전진 실행: fraction=([\d.]+)'),
    'cart_part': re.compile(r'Cartesian 부분 전진: fraction=([\d.]+)'),
    're_fwd':    re.compile(r'남은 거리 재전진 \((\d+)/(\d+)\)'),
    'lock_ok':   re.compile(r'물병 좌표 LOCK 완료: x=(-?[\d.]+), y=(-?[\d.]+), z=(-?[\d.]+)'),
    'diag':      re.compile(r'LOCKED\(gripper\)= X=(-?[\d.]+) Y=(-?[\d.]+) Z=(-?[\d.]+) .*'
                            r'FRESH\(now\)= X=(-?[\d.]+) Y=(-?[\d.]+) Z=(-?[\d.]+).*dist=([\d.]+)cm'),
    'mg_fail':   re.compile(r'MoveGroup failed\. error_code=(-?\d+) \(연속 (\d+)/(\d+)\)'),
    'ctrl':      re.compile(r'CONTROL LOOP: motion_done=(\w+), is_planning=(\w+), goal_active=(\w+)'),
    'btype':     re.compile(r'BOTTLE TYPE = (\w+)'),
    'step1':     re.compile(r'\[STEP 1\] 변환 전 \(Camera Frame\): X=(-?[\d.]+), Y=(-?[\d.]+), Z\(실거리\)=([\d.]+)'),
    'step2':     re.compile(r'\[STEP 2\] 변환 후 \(Robot Base Frame\): X=(-?[\d.]+), Y=(-?[\d.]+), Z\(높이\)=([\d.]+)'),
    'ready':     re.compile(r'\[READY\] init->bottle=([\d.]+)cm init->ready=(-?[\d.]+)cm ready->bottle=([\d.]+)cm'),
}


def wrap(a):
    """각도를 [-pi, pi] 로."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def dist_to_singular(val, singular_angles):
    """val(rad) 에서 가장 가까운 특이 각도까지 거리(도)."""
    return min(abs(math.degrees(wrap(val - s))) for s in singular_angles)


class ArmMonitor(Node):
    def __init__(self):
        super().__init__('arm_monitor')
        self.now0 = time.time()
        self.s = {}          # 상태 저장
        self.last_t = {}     # 키별 마지막 갱신 시각
        self.event = '-'     # 최근 주요 이벤트
        self.event_t = 0.0
        self.joints = {}     # joint 이름 -> 위치(rad)
        self.joints_t = 0.0

        # /rosout 은 양 많음 → depth 크게, reliable
        rosout_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
        )
        self.create_subscription(Log, '/rosout', self.on_log, rosout_qos)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_timer(0.5, self.show)

    # ---- 데이터 수집 ----
    def _set(self, key, val, ev=None):
        self.s[key] = val
        self.last_t[key] = time.time()
        if ev:
            self.event = ev
            self.event_t = time.time()

    def on_joints(self, m):
        for name, pos in zip(m.name, m.position):
            if name in JOINT_NUM:
                self.joints[name] = pos
        self.joints_t = time.time()

    def on_log(self, m):
        if m.name != ARM_NODE:
            return
        msg = m.msg

        mt = RE['xy_track'].search(msg)
        if mt:
            self._set('xy', {
                'bbox': (int(mt.group(1)), int(mt.group(2))),
                'img': (int(mt.group(3)), int(mt.group(4))),
                'ex': int(mt.group(5)), 'ey': int(mt.group(6)),
                'tolx': int(mt.group(7)), 'toly': int(mt.group(8)),
            })
            return

        mc = RE['xy_corr'].search(msg)
        if mc:
            x = self.s.get('xy', {})
            x.update({'ex': int(mc.group(1)), 'ey': int(mc.group(2))})
            self._set('xy', x)
            return

        mf = RE['fwd_calc'].search(msg)
        if mf:
            raw = float(mf.group(1)); actual = float(mf.group(2))
            self._set('fwd', {'raw': raw, 'actual': actual, 'margin': raw - actual},
                      ev='전진거리 계산')
            return

        mg = RE['fwd_goal'].search(msg)
        if mg:
            self._set('fwd_goal', (float(mg.group(1)), float(mg.group(2)), float(mg.group(3))))
            return

        mr = RE['cart_run'].search(msg)
        if mr:
            self._set('cart', {'frac': float(mr.group(1)), 'full': float(mr.group(1)) >= 0.9},
                      ev='Cartesian 전진 실행')
            return

        mp = RE['cart_part'].search(msg)
        if mp:
            self._set('cart', {'frac': float(mp.group(1)), 'full': False}, ev='부분 전진(재전진 예정)')
            return

        mrf = RE['re_fwd'].search(msg)
        if mrf:
            self._set('refwd', (int(mrf.group(1)), int(mrf.group(2))),
                      ev=f"남은거리 재전진 {mrf.group(1)}/{mrf.group(2)}")
            return

        ml = RE['lock_ok'].search(msg)
        if ml:
            self._set('lock', (float(ml.group(1)), float(ml.group(2)), float(ml.group(3))),
                      ev='물병 좌표 LOCK')
            return

        md = RE['diag'].search(msg)
        if md:
            self._set('diag', {
                'lk': (float(md.group(1)), float(md.group(2)), float(md.group(3))),
                'fr': (float(md.group(4)), float(md.group(5)), float(md.group(6))),
                'dist': float(md.group(7)),
            })
            return

        mm = RE['mg_fail'].search(msg)
        if mm:
            self._set('mgfail', (int(mm.group(1)), int(mm.group(2)), int(mm.group(3))),
                      ev=f"MoveGroup 실패 {mm.group(2)}/{mm.group(3)}")
            return

        mctrl = RE['ctrl'].search(msg)
        if mctrl:
            self._set('ctrl', {'motion_done': mctrl.group(1),
                               'is_planning': mctrl.group(2),
                               'goal_active': mctrl.group(3)})
            return

        mb = RE['btype'].search(msg)
        if mb:
            self._set('btype', mb.group(1))
            return

        m1 = RE['step1'].search(msg)
        if m1:
            self._set('cam', (float(m1.group(1)), float(m1.group(2)), float(m1.group(3))))
            return

        m2 = RE['step2'].search(msg)
        if m2:
            self._set('base', (float(m2.group(1)), float(m2.group(2)), float(m2.group(3))))
            return

        mrd = RE['ready'].search(msg)
        if mrd:
            self._set('ready', {'ib': float(mrd.group(1)),
                                'ir': float(mrd.group(2)),
                                'rb': float(mrd.group(3))}, ev='접근 계획(READY 계산)')
            return

        # 그 외 주요 이벤트(상태 전이) — 이벤트 줄로만 표시
        for kw in ('X 보정 완료', 'WRIST 완료', '직선 전진 완료', 'Gripper CLOSE',
                   '리프트 시작', 'Return fixed pose', '전진 실패(데드락)',
                   'MoveGroup motion success', '잘못된 lock 폐기', '그립 보류'):
            if kw in msg:
                self.event = msg.strip()[:60]
                self.event_t = time.time()
                self._set('mgfail', (0, 0, self.s.get('mgfail', (0, 0, 6))[2])) \
                    if kw == 'MoveGroup motion success' else None
                return

    # ---- 특이점 계산 ----
    def singularity(self):
        """현재 joint 각도로 elbow(J3)/wrist(J5) 특이점 근접도 계산.
        UR: elbow≈0/±180° (팔 곧게 펴짐), wrist_2≈0/±180° (손목 정렬) 에서 특이.
        """
        out = []
        worst = None
        if 'elbow_joint' in self.joints:
            d = dist_to_singular(self.joints['elbow_joint'], [0.0, math.pi, -math.pi])
            out.append(('J3 elbow', d))
        if 'wrist_2_joint' in self.joints:
            d = dist_to_singular(self.joints['wrist_2_joint'], [0.0, math.pi, -math.pi])
            out.append(('J5 wrist', d))
        # manipulability 대용 스칼라 (0 에 가까울수록 특이)
        w = 1.0
        if 'elbow_joint' in self.joints:
            w *= abs(math.sin(self.joints['elbow_joint']))
        if 'wrist_2_joint' in self.joints:
            w *= abs(math.sin(self.joints['wrist_2_joint']))
        for name, d in out:
            if worst is None or d < worst[1]:
                worst = (name, d)
        return out, w, worst

    # ---- 렌더링 ----
    def age(self, key):
        t = self.last_t.get(key)
        return '' if t is None else f"{time.time()-t:4.1f}s전"

    def show(self):
        L = []
        L.append('╔══════════ 🤖 ARM GRASP MONITOR (domain 77) ══════════')
        bt = self.s.get('btype', '-')
        ctrl = self.s.get('ctrl', {})
        phase = self._phase(ctrl)
        L.append(f'║ 단계   : {phase}')
        L.append(f'║ 병자세 : {bt}        최근이벤트: {self.event}  ({self._ev_age()})')

        # lock 상태
        lk = self.s.get('lock')
        diag = self.s.get('diag')
        if lk:
            L.append(f'║ LOCK   : ✅ ({lk[0]:.3f}, {lk[1]:.3f}, {lk[2]:.3f})')
        else:
            L.append('║ LOCK   : ❌ 아직 없음')
        if diag:
            warn = '⚠️ 흔들림 큼' if diag['dist'] > 5.0 else '✅'
            L.append(f'║ lock↔now: dist={diag["dist"]:.1f}cm {warn}  '
                     f'(now {diag["fr"][0]:.2f},{diag["fr"][1]:.2f},{diag["fr"][2]:.2f})')

        L.append('╠── 검출 좌표 (depth↔좌표 진단) ───────────────────')
        cam = self.s.get('cam')
        base = self.s.get('base')
        if cam:
            L.append(f'║ 카메라 실거리(depth Z): {cam[2]:.3f} m  ← 센서 raw 깊이  {self.age("cam")}')
            L.append(f'║ 카메라 X,Y           : {cam[0]:+.3f}, {cam[1]:+.3f} m')
        else:
            L.append('║ 카메라 depth: (대기)')
        if base:
            L.append(f'║ base 좌표 X,Y,Z      : {base[0]:.3f}, {base[1]:.3f}, {base[2]:.3f}')
        L.append('║ 판정: depth가 실제거리와 다르면→세그/near_pts 문제')
        L.append('║       depth맞고 base/raw틀리면→TF/extrinsic 문제')

        L.append('╠── XY 보정 ───────────────────────────────────────')
        xy = self.s.get('xy')
        if xy and 'ex' in xy:
            tolx = xy.get('tolx', 20); toly = xy.get('toly', 20)
            okx = '✅' if abs(xy['ex']) < tolx else '❌'
            oky = '✅' if abs(xy['ey']) < toly else '❌'
            L.append(f'║ error_x : {xy["ex"]:>5}  {okx} (±{tolx})')
            L.append(f'║ error_y : {xy["ey"]:>5}  {oky} (±{toly})   {self.age("xy")}')
        else:
            L.append('║ (보정 단계 대기)')

        L.append('╠── Z 전진 / READY 분해 (READY = 물병 7cm 앞) ──────')
        ready = self.s.get('ready')
        fwd = self.s.get('fwd')
        cart = self.s.get('cart')
        if ready:
            L.append(f'║ 초기자세→물병 거리 : {ready["ib"]:5.1f} cm   ← 초기자세에서 감지한 물병까지 거리  {self.age("ready")}')
            L.append(f'║ 초기자세→ready 전진: {ready["ir"]:5.1f} cm   ← 초기자세에서 ready까지 계획 전진값')
            L.append(f'║ ready→병 남은거리  : {ready["rb"]:5.1f} cm   ← grasp_margin')
        else:
            L.append('║ (접근 계획 대기 — READY 값 미수신)')
        if fwd and cart:
            executed = fwd['actual'] * cart['frac']
            full = '✅완전' if cart['full'] else f'⚠️부분 {cart["frac"]:.0%}'
            L.append(f'║ 실제 이동값(전진)  : {executed:5.1f} cm   (Cartesian {cart["frac"]:.2f} {full})')
        refwd = self.s.get('refwd')
        if refwd:
            L.append(f'║ 청크 재전진        : {refwd[0]}/{refwd[1]} 회')
        fg = self.s.get('fwd_goal')
        if fg:
            L.append(f'║ 전진 목표 pose    : x={fg[0]:.3f} y={fg[1]:.3f} z={fg[2]:.3f}')

        L.append('╠── 계획 상태 ─────────────────────────────────────')
        if ctrl:
            L.append(f'║ motion_done={ctrl["motion_done"]}  is_planning={ctrl["is_planning"]}  '
                     f'goal_active={ctrl["goal_active"]}')
        mgf = self.s.get('mgfail')
        if mgf:
            warn = '⚠️' if mgf[1] >= 3 else ''
            L.append(f'║ MoveGroup 연속실패: {mgf[1]}/{mgf[2]}  (error_code={mgf[0]}) {warn}')

        L.append('╠── Singularity (joint) ───────────────────────────')
        if self.joints:
            out, w, worst = self.singularity()
            for name, d in out:
                flag = '🛑 특이점!' if d < 8 else ('⚠️ 근접' if d < 15 else '✅')
                L.append(f'║ {name:9}: {d:5.1f}° 여유   {flag}')
            L.append(f'║ manip(0=특이): {w:.3f}   '
                     f'{("🛑 위험: " + worst[0]) if worst and worst[1] < 8 else ""}')
            L.append(f'║ joint 갱신: {time.time()-self.joints_t:.1f}s전')
        else:
            L.append('║ (/joint_states 없음 — 팔 드라이버/도메인 확인)')

        L.append('╚══════════════════════════════════════════════════')

        print('\033[2J\033[H', end='')
        print('\n'.join(L))

    def _phase(self, ctrl):
        ev = self.event
        if 'Return' in ev or '리프트' in ev:
            return '🔵 복귀/리프트'
        if 'Gripper' in ev or '전진 완료' in ev:
            return '🟢 파지(grip)'
        if '재전진' in ev or 'Cartesian' in ev or '전진' in ev:
            return '🟡 직진 전진'
        if 'WRIST' in ev or '보정' in ev:
            return '🟡 XY 보정'
        if 'LOCK' in ev:
            return '🟠 좌표 LOCK'
        if ctrl and ctrl.get('goal_active') == 'True':
            return '🟠 경로 계획중'
        return '⚪ 대기'

    def _ev_age(self):
        return f'{time.time()-self.event_t:.1f}s전' if self.event_t else '-'


def main():
    from rclpy.executors import ExternalShutdownException
    rclpy.init()
    node = ArmMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
