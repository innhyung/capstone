"""Diff the CORRECT from-scratch FK vs the buggy code's FK to localize the bug.
Tests: (a) buggy fk_tcp (omits step1), (b) Rz(pi) prepend vs postmultiply,
(c) the 'XYZ' (intrinsic) vs 'xyz' (extrinsic) rpy convention in the buggy code."""
import numpy as np
from scipy.spatial.transform import Rotation

PI = np.pi
q = np.array([-0.313572, -1.269609, -0.636822, -3.763643, 1.893224, -2.977382])
gt = np.array([0.166652, -0.005264, 0.370135])
Rz_pi = np.diag([-1.0, -1.0, 1.0, 1.0])


def Rz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])


# rpy_T as the BUGGY code writes it (Rotation.from_euler('XYZ', ...) = INTRINSIC)
def rpy_T_buggy(x, y, z, roll, pitch, yaw):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('XYZ', [roll, pitch, yaw]).as_matrix()
    T[:3, 3] = [x, y, z]
    return T


# rpy_T CORRECT (fixed-axis = extrinsic = lowercase 'xyz')
def rpy_T_correct(x, y, z, roll, pitch, yaw):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
    T[:3, 3] = [x, y, z]
    return T


def chain_no_step1(rpy_T):
    """Exactly the buggy _fk_urdf_tool0 + T_tool0_tcp (starts at shoulder, no Rz(pi))."""
    T = rpy_T(0, 0, 0.15185, 0, 0, 0) @ Rz(q[0])
    T = T @ rpy_T(0, 0, 0, 1.570796327, 0, 0) @ Rz(q[1])
    T = T @ rpy_T(-0.24355, 0, 0, 0, 0, 0) @ Rz(q[2])
    T = T @ rpy_T(-0.2132, 0, 0.13105, 0, 0, 0) @ Rz(q[3])
    T = T @ rpy_T(0, -0.08535, 0, 1.570796327, 0, 0) @ Rz(q[4])
    T = T @ rpy_T(0, 0.0921, 0, 1.570796326589793, 3.141592653589793, 3.141592653589793) @ Rz(q[5])
    T = T @ rpy_T(0, 0, 0, 0, -PI / 2.0, -PI / 2.0)
    T = T @ rpy_T(0, 0, 0, PI / 2.0, 0, PI / 2.0)
    T_tool0_tcp = (rpy_T(0, 0, 0.0678750000000002, 0, -1.5707963267949, 3.14159265358979) @
                   rpy_T(0.107175, 0, 0, 0, 0, 0))
    return T @ T_tool0_tcp


def report(name, T):
    p = T[:3, 3]
    print(f"{name:42s} {np.round(p,6)}  err={np.linalg.norm(p-gt):.4f}m")


print("ground truth:", gt, "\n")

# 1. buggy FK with buggy 'XYZ' rpy, no step1 (what code actually computes)
Tb = chain_no_step1(rpy_T_buggy)
report("buggy fk_tcp ('XYZ',no step1)", Tb)
report("  Rz(pi)@buggy fk_tcp", Rz_pi @ Tb)

# 2. buggy chain but with CORRECT 'xyz' rpy, still no step1
Tc = chain_no_step1(rpy_T_correct)
report("'xyz' fk_tcp (no step1)", Tc)
report("  Rz(pi)@['xyz' no step1]  <-- FIX?", Rz_pi @ Tc)

# 3. does 'XYZ' vs 'xyz' even matter for these rpy values?
print("\n'XYZ' vs 'xyz' rotation max-diff per joint origin:")
for nm, r, p_, y_ in [("UA", 1.570796327, 0, 0),
                      ("W2", 1.570796327, 0, 0),
                      ("W3", 1.570796326589793, 3.141592653589793, 3.141592653589793),
                      ("flange", 0, -PI/2, -PI/2),
                      ("tool0", PI/2, 0, PI/2),
                      ("grlink", 0, -1.5707963267949, 3.14159265358979)]:
    A = Rotation.from_euler('XYZ', [r, p_, y_]).as_matrix()
    B = Rotation.from_euler('xyz', [r, p_, y_]).as_matrix()
    print(f"  {nm:8s} maxdiff={np.abs(A-B).max():.3e}")
