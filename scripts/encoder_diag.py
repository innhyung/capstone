#!/usr/bin/env python3
"""
encoder_diag.py — 왼쪽 엔코더(GPIO34/35) floating/×2 디코딩 진단 도구.

배경:
  ESP32 GPIO34/35 는 입력 전용 + 내부 풀업 없음 → 왼쪽 엔코더 신호가
  floating 일 수 있음. 증상: 카운트 절반(×2), PID 떨림, 우편향, 맵 ghost wall.
  이 스크립트로 "정말 왼쪽 엔코더가 범인인지" 를 데이터로 가른다.

ROS 가 아니라 시리얼로 ESP32 와 직접 대화한다 (PWM 개루프 = PID 우회).
  → 펌웨어 명령: PWM:left,right  (개루프), RAW (raw 핀 출력 켜기)
  → 펌웨어 피드백: FB:L_count,R_count,L_vel,R_vel  /  RAW:R_A,R_B,L_A,L_B

⚠️ 전제 (꼭!):
  1. ROS serial_bridge(또는 mapping/navigation launch) 를 **끄기**.
     안 끄면 이 스크립트가 /dev/esp32 를 못 열거나 명령이 섞인다.
  2. counts / spin 테스트는 **바퀴를 공중에 띄운 상태**(받침대)에서.
     안 그러면 로봇이 굴러간다.
  3. 모터 전원 ON.

사용 순서 (내일 이대로):
  1) python3 encoder_diag.py monitor          # 통신 되는지 + FB 흐르는지 확인
  2) python3 encoder_diag.py rest             # [정지 상태] 핀이 floating 으로 떨리나 → 회전 불필요!
  3) python3 encoder_diag.py counts           # [바퀴 띄움] 좌/우 카운트 비율 = ×2 인지 + 노이즈인지
  4) python3 encoder_diag.py spin --pwm 120   # 임의 개루프 구동 + FB 로깅 (우편향 육안확인용)
"""
import argparse
import statistics
import sys
import threading
import time

try:
    import serial
except ImportError:
    sys.exit('pyserial 필요: pip install pyserial')


class ESP32Link:
    """ESP32 시리얼 연결 + 백그라운드 리더 + 페일세이프 비팅 송신기."""

    def __init__(self, port, baud):
        try:
            self._ser = serial.Serial(port, baud, timeout=0.2)
        except serial.SerialException as exc:
            sys.exit(f'시리얼 열기 실패: {exc}\n'
                     f'  → ROS serial_bridge 가 떠서 포트를 잡고 있지 않은지 확인하세요.')
        self._lock = threading.Lock()
        self._running = True

        self.latest_fb = None          # (L_count, R_count, L_vel, R_vel, t)
        self.recording = False
        self.fb_log = []               # 기록 중 FB 샘플 누적
        self.raw_log = []              # 기록 중 RAW 샘플 누적

        self._drive = None             # (l, r) 이면 워치독이 계속 PWM 송신
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._writer = threading.Thread(target=self._beat_loop, daemon=True)
        self._reader.start()
        self._writer.start()

    # --- 송신 ---
    def send(self, line):
        with self._lock:
            self._ser.write((line + '\n').encode('ascii'))

    def drive(self, left, right):
        """개루프 PWM 구동 시작 (워치독이 0.1s 마다 재송신해 페일세이프 회피)."""
        self._drive = (int(left), int(right))

    def stop(self):
        """구동 중단 + 모터 정지 명령 여러 번."""
        self._drive = None
        for _ in range(5):
            self.send('PWM:0,0')
            time.sleep(0.05)

    def _beat_loop(self):
        while self._running:
            d = self._drive
            if d is not None:
                self.send(f'PWM:{d[0]},{d[1]}')
            time.sleep(0.1)            # 펌웨어 500ms 페일세이프보다 충분히 빠르게

    # --- 수신 ---
    def _read_loop(self):
        while self._running:
            try:
                raw = self._ser.readline()
            except serial.SerialException:
                break
            if not raw:
                continue
            line = raw.decode('ascii', errors='ignore').strip()
            t = time.time()
            if line.startswith('FB:'):
                self._handle_fb(line[3:], t)
            elif line.startswith('RAW:'):
                self._handle_raw(line[4:], t)

    def _handle_fb(self, payload, t):
        p = payload.split(',')
        if len(p) != 4:
            return
        try:
            lc, rc, lv, rv = int(p[0]), int(p[1]), int(p[2]), int(p[3])
        except ValueError:
            return
        fb = (lc, rc, lv, rv, t)
        self.latest_fb = fb
        if self.recording:
            self.fb_log.append(fb)

    def _handle_raw(self, payload, t):
        p = payload.split(',')
        if len(p) != 4:
            return
        try:
            ra, rb, la, lb = (int(x) for x in p)
        except ValueError:
            return
        if self.recording:
            self.raw_log.append((ra, rb, la, lb, t))

    def start_recording(self):
        self.fb_log = []
        self.raw_log = []
        self.recording = True

    def stop_recording(self):
        self.recording = False

    def close(self):
        self._running = False
        time.sleep(0.2)
        try:
            self._ser.close()
        except Exception:
            pass


def wait_fb(link, timeout=3.0):
    """첫 FB 수신 대기 — 통신 확인."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        if link.latest_fb is not None:
            return True
        time.sleep(0.05)
    return False


# ============================================================
# 1) monitor — 통신/FB 흐름 확인
# ============================================================
def cmd_monitor(link, args):
    print('FB 실시간 출력 (Ctrl+C 종료). 모터 명령 없음.')
    if not wait_fb(link):
        print('❌ FB 안 들어옴 — 배선/전원/serial_bridge 종료 여부 확인.')
        return
    print(f'{"L_count":>10} {"R_count":>10} {"L_vel":>7} {"R_vel":>7}')
    try:
        while True:
            fb = link.latest_fb
            if fb:
                print(f'{fb[0]:>10} {fb[1]:>10} {fb[2]:>7} {fb[3]:>7}', end='\r')
            time.sleep(0.1)
    except KeyboardInterrupt:
        print('\n종료.')


# ============================================================
# 2) rest — [정지] floating 노이즈 검출 (회전 불필요)
# ============================================================
def cmd_rest(link, args):
    print('=== 정지 floating 테스트 ===')
    print('바퀴를 움직이지 마세요. RAW 핀 상태를 정지 상태에서 샘플링합니다.')
    print('원리: 정지 중엔 어떤 핀도 안 바뀌어야 정상. 왼쪽(34/35) 핀만 마구\n'
          '      토글하면 → floating 노이즈 확정 (오른쪽 32/33 은 내부 풀업이라 안정).')
    print('⚠️ RAW 모드는 끄는 명령이 없어요 — 테스트 끝나면 ESP32 EN(리셋) 누르세요.\n')

    link.send('RAW')                  # rawDebug 켜기
    time.sleep(0.3)
    link.start_recording()
    dur = args.secs
    print(f'{dur}s 동안 정지 상태 샘플링...')
    time.sleep(dur)
    link.stop_recording()

    samples = link.raw_log
    if len(samples) < 5:
        print(f'❌ RAW 샘플 부족({len(samples)}). 펌웨어가 RAW 출력하는지 확인.')
        return

    # 핀별 연속 샘플 간 전이(토글) 횟수
    names = ['R_A(32)', 'R_B(33)', 'L_A(34)', 'L_B(35)']
    transitions = [0, 0, 0, 0]
    for a, b in zip(samples, samples[1:]):
        for i in range(4):
            if a[i] != b[i]:
                transitions[i] += 1

    n = len(samples)
    print(f'\n샘플 {n}개 ({dur}s) — 정지 상태 핀별 토글 횟수:')
    for i in range(4):
        rate = transitions[i] / n * 100
        flag = ''
        if i >= 2 and transitions[i] > max(3, n * 0.05):
            flag = '  ← floating 노이즈 의심!'
        print(f'  {names[i]:>8}: {transitions[i]:>4} 회 ({rate:4.1f}%){flag}')

    left_noise = transitions[2] + transitions[3]
    right_noise = transitions[0] + transitions[1]
    print('\n--- 판정 ---')
    if left_noise > max(5, right_noise * 3) and right_noise <= max(3, n * 0.05):
        print('🔴 왼쪽 핀이 정지 중에도 떨림 → floating 확정. 풀업 저항(4.7k~10k) 답.')
    elif left_noise <= max(3, n * 0.05) and right_noise <= max(3, n * 0.05):
        print('🟢 정지 중 양쪽 다 안정. floating 노이즈는 아님.\n'
              '   → counts 테스트로 ×2 여부(해상도)만 더 확인.')
    else:
        print('🟡 애매함. counts 테스트(구동)로 추가 판별 필요.')


# ============================================================
# 3) counts — [바퀴 띄움] 좌/우 카운트 비율 + 노이즈
# ============================================================
def cmd_counts(link, args):
    print('=== 좌/우 카운트 비교 테스트 (개루프) ===')
    print('⚠️ 바퀴를 공중에 띄웠는지 확인! (안 띄우면 로봇이 굴러감)')
    try:
        input('   준비됐으면 Enter, 취소는 Ctrl+C ... ')
    except KeyboardInterrupt:
        print('\n취소.')
        return

    if not wait_fb(link):
        print('❌ FB 안 들어옴 — 통신 확인.')
        return

    pwm = args.pwm
    print(f'개루프 PWM {pwm},{pwm} 로 {args.secs}s 구동 (양쪽 동일).')
    link.drive(pwm, pwm)
    time.sleep(0.7)                   # 모터 가속 안정화 대기
    link.start_recording()
    time.sleep(args.secs)
    link.stop_recording()
    link.stop()

    log = link.fb_log
    if len(log) < 5:
        print(f'❌ FB 샘플 부족({len(log)}). PWM 이 약해 안 돌았을 수도 → --pwm 키우기.')
        return

    # 전체 델타 + 구간별 증분
    dL_total = log[-1][0] - log[0][0]
    dR_total = log[-1][1] - log[0][1]
    incs_L, incs_R, ratios = [], [], []
    zero_L_while_R = 0
    for a, b in zip(log, log[1:]):
        dl, dr = b[0] - a[0], b[1] - a[1]
        incs_L.append(dl)
        incs_R.append(dr)
        if abs(dr) > 3:
            ratios.append(abs(dl) / abs(dr))
            if abs(dl) == 0:
                zero_L_while_R += 1

    ratio = abs(dL_total) / abs(dR_total) if dR_total != 0 else float('inf')
    print(f'\n구동 {args.secs}s 결과:')
    print(f'  왼쪽 총 카운트 변화 : {dL_total}')
    print(f'  오른쪽 총 카운트 변화: {dR_total}')
    print(f'  비율 (왼/오)        : {ratio:.3f}')
    if ratios:
        cv = statistics.pstdev(ratios) / statistics.mean(ratios) if statistics.mean(ratios) else 0
        print(f'  구간별 비율 변동(CV): {cv:.2f}  (작을수록 일관됨)')
        print(f'  왼쪽이 0인데 오른쪽 도는 구간: {zero_L_while_R}/{len(ratios)} (드롭아웃)')

    print('\n--- 판정 ---')
    # 0) 한쪽 카운트가 사실상 0 → 신호 단절 (floating 아님!)
    NEAR_ZERO = 50
    if abs(dL_total) < NEAR_ZERO and abs(dR_total) >= NEAR_ZERO:
        print('🔴 왼쪽 카운트 ≈ 0 → 왼쪽 엔코더 신호 단절(사망).')
        print('   floating 아님(풀업 무관). 모터가 돌았다면 → 엔코더/컨버터/배선.')
        print('   → 위상 판별: spin --left 120 --right 0 --raw')
        return
    if abs(dR_total) < NEAR_ZERO and abs(dL_total) >= NEAR_ZERO:
        print('🔴 오른쪽 카운트 ≈ 0 → 오른쪽 엔코더 신호 단절(사망).')
        return
    # 1) 스케일
    if 0.40 <= ratio <= 0.60:
        print('• 비율 ≈ 0.5 → 왼쪽이 절반(×2 디코딩) 맞음.')
    elif 0.85 <= ratio <= 1.15:
        print('• 비율 ≈ 1.0 → 양쪽 ×4 정상 (왼쪽도 멀쩡). 다른 원인 의심.')
    else:
        print(f'• 비율 {ratio:.2f} → 비정상. 모터 한쪽이 안 돌거나 배선 문제 가능.')
    # 2) 노이즈/일관성
    if ratios:
        if cv > 0.25 or zero_L_while_R > len(ratios) * 0.1:
            print('• 변동 큼/드롭아웃 있음 → floating 노이즈. 풀업 저항 필요.')
        else:
            print('• 변동 작음 → 깨끗한 ×2. 해상도만 절반인 거라 5700 그대로 써도 OK.\n'
                  '  (떨림/우편향이 남으면 그건 PID·stiction 쪽)')


# ============================================================
# 4) spin — 임의 개루프 구동 + FB 로깅 (우편향 육안 + 단일바퀴 가능)
# ============================================================
def cmd_spin(link, args):
    l = args.pwm if args.left is None else args.left
    r = args.pwm if args.right is None else args.right
    print(f'=== 개루프 구동: 왼 PWM={l}, 오 PWM={r}, {args.secs}s ===')
    print('바닥에서 하면 우편향 육안 확인용 / 띄우면 단일바퀴 테스트용.')
    print('  (한쪽만 돌리려면 --left 120 --right 0 처럼)')
    try:
        input('   Enter 시작, Ctrl+C 취소 ... ')
    except KeyboardInterrupt:
        print('\n취소.')
        return

    wait_fb(link)
    if args.raw:
        print('RAW 위상 분석 모드: 구동 중 각 핀 토글을 셉니다.')
        print('⚠️ 끝나면 ESP32 EN(리셋) 누르세요.')
        link.send('RAW')
        time.sleep(0.3)
        link.start_recording()
    link.drive(l, r)
    infinite = args.secs <= 0
    if infinite:
        print('무한 구동 — 멈추려면 Ctrl+C (테스터로 찍는 동안 계속 돕니다).')
    t0 = time.time()
    try:
        while infinite or time.time() - t0 < args.secs:
            fb = link.latest_fb
            if fb:
                print(f'  L_vel={fb[2]:>6} R_vel={fb[3]:>6} '
                      f'L_cnt={fb[0]:>10} R_cnt={fb[1]:>10}', end='\r')
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        link.stop()
        print('\n정지.')

    if not args.raw:
        return
    link.stop_recording()
    samples = link.raw_log
    if len(samples) < 5:
        print(f'❌ RAW 샘플 부족({len(samples)}). 펌웨어 RAW 출력 확인.')
        return
    names = ['R_A(32)', 'R_B(33)', 'L_A(34)', 'L_B(35)']
    trans = [0, 0, 0, 0]
    for a, b in zip(samples, samples[1:]):
        for i in range(4):
            if a[i] != b[i]:
                trans[i] += 1
    n = len(samples)
    print(f'\n구동 중 핀별 토글 (샘플 {n}개):')
    for i in range(4):
        # 회전 중이면 살아있는 위상은 자주 토글, 죽은 위상은 ~0
        dead = ' ← 죽음(고정)' if trans[i] <= max(2, n * 0.02) else ''
        print(f'  {names[i]:>8}: {trans[i]:>4} 회 ({trans[i]/n*100:4.1f}%){dead}')
    la, lb = trans[2], trans[3]
    print('\n--- 왼쪽 위상 판정 ---')
    live = max(la, lb)
    dead_v = min(la, lb)
    if live > n * 0.05 and dead_v <= max(2, n * 0.02):
        dead_name = 'L_A(34)' if la < lb else 'L_B(35)'
        live_name = 'L_B(35)' if la < lb else 'L_A(34)'
        print(f'🔴 {dead_name} 위상이 죽음 (고정), {live_name} 는 살아있음.')
        print(f'   → 한 위상만 끊김. 그 채널의 [엔코더 출력 / 레벨컨버터 채널 / 배선] 중 하나.')
        print(f'   살아있는 채널은 ESP32까지 신호가 오므로 그 채널 컨버터·배선은 정상.')
    elif la <= max(2, n*0.02) and lb <= max(2, n*0.02):
        print('🔴 L_A·L_B 둘 다 고정 → 왼쪽 전체 신호 단절 (공통 전원/GND/컨버터칩).')
    else:
        print('🟡 양쪽 다 토글 → ESP32까지는 신호 옴. 카운트 0이면 디코딩/타이밍 의심.')


def main():
    ap = argparse.ArgumentParser(
        description='ESP32 왼쪽 엔코더 floating/×2 진단',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='순서: monitor → rest → counts → spin')
    ap.add_argument('--port', default='/dev/esp32', help='시리얼 포트 (기본 /dev/esp32)')
    ap.add_argument('--baud', type=int, default=115200, help='보레이트 (기본 115200)')
    sub = ap.add_subparsers(dest='cmd', required=True)

    sub.add_parser('monitor', help='FB 실시간 출력 (통신 확인)')

    p_rest = sub.add_parser('rest', help='[정지] floating 노이즈 검출')
    p_rest.add_argument('--secs', type=float, default=5.0, help='샘플링 시간 (기본 5s)')

    p_cnt = sub.add_parser('counts', help='[바퀴띄움] 좌/우 카운트 비율')
    p_cnt.add_argument('--pwm', type=int, default=120, help='개루프 PWM (기본 120)')
    p_cnt.add_argument('--secs', type=float, default=4.0, help='구동 시간 (기본 4s)')

    p_spin = sub.add_parser('spin', help='[개루프] 임의 구동 + 로깅')
    p_spin.add_argument('--pwm', type=int, default=120, help='양쪽 공통 PWM')
    p_spin.add_argument('--left', type=int, default=None, help='왼쪽 PWM 개별 지정')
    p_spin.add_argument('--right', type=int, default=None, help='오른쪽 PWM 개별 지정')
    p_spin.add_argument('--secs', type=float, default=3.0,
                        help='구동 시간 (기본 3s, 0 이하면 Ctrl+C 까지 무한)')
    p_spin.add_argument('--raw', action='store_true',
                        help='구동 중 RAW 핀 토글 분석 (어느 위상이 죽었는지)')

    args = ap.parse_args()
    link = ESP32Link(args.port, args.baud)
    try:
        {'monitor': cmd_monitor, 'rest': cmd_rest,
         'counts': cmd_counts, 'spin': cmd_spin}[args.cmd](link, args)
    finally:
        link.stop()
        link.close()


if __name__ == '__main__':
    main()
