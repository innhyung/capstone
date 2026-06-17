"""Prove the fix: change rpy_T to 'xyz' (extrinsic) AND keep Rz(pi) prepend.
Verify base_link FK matches MoveIt at ground truth AND that the buggy 'XYZ'
chain is wrong by >13cm even with Rz(pi). Also random-joint cross-check the two
correct paths (full-chain vs Rz(pi)@no-step1) agree everywhere."""
import numpy as np
from scipy.spatial.transform import Rotation

PI = np.pi
q_gt = np.array([-0.313572, -1.269609, -0.636822, -3.763643, 1.893224, -2.977382])
gt = np.array([0.166652, -0.005264, 0.370135])
Rz_pi = np.diag([-1.0, -1.0, 1.0, 1.0])


def Rz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])


def make_rpy_T(conv):
    def f(x, y, z, roll, pitch, yaw):
        T = np.eye(4)
        T[:3, :3] = Rotation.from_euler(conv, [roll, pitch, yaw]).as_matrix()
        T[:3, 3] = [x, y, z]
        return T
    return f


def fk_tool0_no_step1(q, rpy_T):
    T = rpy_T(0, 0, 0.15185, 0, 0, 0) @ Rz(q[0])
    T = T @ rpy_T(0, 0, 0, 1.570796327, 0, 0) @ Rz(q[1])
    T = T @ rpy_T(-0.24355, 0, 0, 0, 0, 0) @ Rz(q[2])
    T = T @ rpy_T(-0.2132, 0, 0.13105, 0, 0, 0) @ Rz(q[3])
    T = T @ rpy_T(0, -0.08535, 0, 1.570796327, 0, 0) @ Rz(q[4])
    T = T @ rpy_T(0, 0.0921, 0, 1.570796326589793, 3.141592653589793, 3.141592653589793) @ Rz(q[5])
    T = T @ rpy_T(0, 0, 0, 0, -PI / 2.0, -PI / 2.0)
    T = T @ rpy_T(0, 0, 0, PI / 2.0, 0, PI / 2.0)
    return T


def fk_tcp_base(q, conv):
    """FIXED: Rz(pi) prepend (step1) + correct conv rpy, base_link -> gripper_tcp."""
    rpy_T = make_rpy_T(conv)
    T_tool0_tcp = (rpy_T(0, 0, 0.0678750000000002, 0, -1.5707963267949, 3.14159265358979) @
                   rpy_T(0.107175, 0, 0, 0, 0, 0))
    T_inertia = fk_tool0_no_step1(q, rpy_T) @ T_tool0_tcp     # base_link_inertia frame
    return Rz_pi @ T_inertia                                  # -> base_link


# --- at ground truth ---
p_fix = fk_tcp_base(q_gt, 'xyz')[:3, 3]
p_old = fk_tcp_base(q_gt, 'XYZ')[:3, 3]
print("FIXED  (Rz(pi) + 'xyz'):", np.round(p_fix, 6), " err=%.5fm" % np.linalg.norm(p_fix - gt))
print("BUGGY  (Rz(pi) + 'XYZ'):", np.round(p_old, 6), " err=%.5fm" % np.linalg.norm(p_old - gt))

# --- cross-check fixed path vs the verified full-chain FK over 500 random joints ---
def fk_full(q):
    rpy_T = make_rpy_T('xyz')
    T = rpy_T(0, 0, 0, 0, 0, PI)
    T = T @ fk_tool0_no_step1(q, rpy_T)
    T = T @ (rpy_T(0, 0, 0.0678750000000002, 0, -1.5707963267949, 3.14159265358979) @
             rpy_T(0.107175, 0, 0, 0, 0, 0))
    return T

rng = np.random.default_rng(1)
worst = 0.0
for _ in range(500):
    q = rng.uniform(-PI, PI, 6)
    worst = max(worst, np.linalg.norm(fk_tcp_base(q, 'xyz')[:3, 3] - fk_full(q)[:3, 3]))
print("Rz(pi)@no-step1  vs  full-chain  worst pos diff over 500 random q:", f"{worst:.3e} m")
