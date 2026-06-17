"""UR3 해석적 IK + 정확 URDF FK 자가검증 (오프라인, 로봇 불필요).
pointcloud_all_test.py 에 넣기 전 수학 검증용."""
import numpy as np
from scipy.spatial.transform import Rotation

PI = np.pi
# UR3e DH (실제 로봇팔, 2026-06-17). UR3 아님 — d4/d6 가 다름.
D1, A2, A3, D4, D5, D6 = 0.15185, -0.24355, -0.2132, 0.13105, 0.08535, 0.0921


def rpy_T(x, y, z, roll, pitch, yaw):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()  # 'xyz'=extrinsic=URDF rpy (버그수정 2026-06-18)
    T[:3, 3] = [x, y, z]
    return T


def Rz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])


def dh_A(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0, sa, ca, d],
        [0, 0, 0, 1.0]])


# --- 정확 URDF 상수 변환 ---
O_SH = rpy_T(0, 0, 0.15185, 0, 0, 0)
O_UA = rpy_T(0, 0, 0, 1.570796327, 0, 0)
O_FA = rpy_T(-0.24355, 0, 0, 0, 0, 0)
O_W1 = rpy_T(-0.2132, 0, 0.13105, 0, 0, 0)
O_W2 = rpy_T(0, -0.08535, 0, 1.570796327, 0, 0)
O_W3 = rpy_T(0, 0.0921, 0, 1.570796326589793, 3.141592653589793, 3.141592653589793)
T_W3_FLANGE = rpy_T(0, 0, 0, 0, -PI / 2.0, -PI / 2.0)
T_FLANGE_TOOL0 = rpy_T(0, 0, 0, PI / 2.0, 0, PI / 2.0)
T_TOOL0_GRLINK = rpy_T(0, 0, 0.0678750000000002, 0, -1.5707963267949, 3.14159265358979)
T_GRLINK_TCP = rpy_T(0.107175, 0, 0, 0, 0, 0)
T_TOOL0_TCP = T_TOOL0_GRLINK @ T_GRLINK_TCP


def fk_urdf_tool0(q):
    return (O_SH @ Rz(q[0]) @ O_UA @ Rz(q[1]) @ O_FA @ Rz(q[2]) @
            O_W1 @ Rz(q[3]) @ O_W2 @ Rz(q[4]) @ O_W3 @ Rz(q[5]) @
            T_W3_FLANGE @ T_FLANGE_TOOL0)


def fk_urdf_tcp(q):
    return fk_urdf_tool0(q) @ T_TOOL0_TCP


def fk_dh(theta):
    A1 = dh_A(theta[0], D1, 0, PI / 2)
    A2_ = dh_A(theta[1], 0, A2, 0)
    A3_ = dh_A(theta[2], 0, A3, 0)
    A4 = dh_A(theta[3], D4, 0, PI / 2)
    A5 = dh_A(theta[4], D5, 0, -PI / 2)
    A6 = dh_A(theta[5], D6, 0, 0)
    return A1 @ A2_ @ A3_ @ A4 @ A5 @ A6


# --- 캘리브레이션: U_tool0(q) == fk_dh(q) @ B 인가? (A=I, theta=q 가정) ---
def calibrate():
    B0 = np.linalg.inv(fk_dh(np.zeros(6))) @ fk_urdf_tool0(np.zeros(6))
    maxdiff = 0.0
    rng = np.random.default_rng(0)
    for _ in range(200):
        q = rng.uniform(-PI, PI, 6)
        B = np.linalg.inv(fk_dh(q)) @ fk_urdf_tool0(q)
        maxdiff = max(maxdiff, np.abs(B - B0).max())
    return B0, maxdiff


def ur_ik(T):
    """T: DH0 -> DH6. up to 8 solutions."""
    sols = []
    T = np.asarray(T, float)
    T02, T00, T01, T03 = T[0, 2], T[0, 0], T[0, 1], T[0, 3]
    T12, T10, T11, T13 = T[1, 2], T[1, 0], T[1, 1], T[1, 3]
    # theta1
    p05 = T @ np.array([0, 0, -D6, 1.0])
    psi = np.arctan2(p05[1], p05[0])
    L = np.hypot(p05[0], p05[1])
    if D4 > L + 1e-9:
        return []
    phi = np.arccos(np.clip(D4 / L, -1, 1))
    for t1 in (psi + phi + PI / 2, psi - phi + PI / 2):
        s1, c1 = np.sin(t1), np.cos(t1)
        a5 = np.clip((T03 * s1 - T13 * c1 - D4) / D6, -1, 1)
        for t5 in (np.arccos(a5), -np.arccos(a5)):
            s5 = np.sin(t5)
            if abs(s5) < 1e-7:
                t6 = 0.0
            else:
                t6 = np.arctan2((-T01 * s1 + T11 * c1) / s5,
                                (T00 * s1 - T10 * c1) / s5)
            A1 = dh_A(t1, D1, 0, PI / 2)
            A6 = dh_A(t6, D6, 0, 0)
            A5 = dh_A(t5, D5, 0, -PI / 2)
            T16 = np.linalg.inv(A1) @ T
            T14 = T16 @ np.linalg.inv(A6) @ np.linalg.inv(A5)
            p13 = (T14 @ np.array([0, -D4, 0, 1.0]))[:3]
            r = np.hypot(p13[0], p13[1])
            c3 = np.clip((r**2 - A2**2 - A3**2) / (2 * A2 * A3), -1, 1)
            for t3 in (np.arccos(c3), -np.arccos(c3)):
                s3 = np.sin(t3)
                t2 = np.arctan2(p13[1], p13[0]) - np.arctan2(A3 * s3, A2 + A3 * np.cos(t3))
                A2m = dh_A(t2, 0, A2, 0)
                A3m = dh_A(t3, 0, A3, 0)
                T34 = np.linalg.inv(A3m) @ np.linalg.inv(A2m) @ T14
                t4 = np.arctan2(T34[1, 0], T34[0, 0])
                sols.append(np.array([t1, t2, t3, t4, t5, t6]))
    return sols


def rot_err_deg(Ra, Rb):
    R = Ra[:3, :3].T @ Rb[:3, :3]
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


if __name__ == "__main__":
    B0, maxdiff = calibrate()
    print(f"[CALIB] A=I, theta=q 가정 → B 상수성 maxdiff={maxdiff:.2e}  "
          f"({'PASS' if maxdiff < 1e-6 else 'FAIL'})")
    B_inv = np.linalg.inv(B0)

    rng = np.random.default_rng(42)
    n_ok = n_total = 0
    worst_pos = worst_rot = 0.0
    for _ in range(500):
        q_true = rng.uniform(-2.5, 2.5, 6)
        T_tcp = fk_urdf_tcp(q_true)          # 목표 (base->gripper_tcp)
        T_tool0 = T_tcp @ np.linalg.inv(T_TOOL0_TCP)
        T_dh = T_tool0 @ B_inv
        sols = ur_ik(T_dh)
        n_total += 1
        # 해 중 q_true 를 FK 로 재현하는 게 있나 + 정확도
        best = None
        for s in sols:
            Tc = fk_urdf_tcp(s)
            pe = np.linalg.norm(Tc[:3, 3] - T_tcp[:3, 3])
            re = rot_err_deg(Tc, T_tcp)
            if best is None or pe + re < best[0]:
                best = (pe + re, pe, re)
        if best and best[1] < 1e-3 and best[2] < 0.1:
            n_ok += 1
            worst_pos = max(worst_pos, best[1])
            worst_rot = max(worst_rot, best[2])
    print(f"[IK<->FK] {n_ok}/{n_total} 왕복 성공  "
          f"worst_pos={worst_pos*1000:.3f}mm worst_rot={worst_rot:.4f}deg")
