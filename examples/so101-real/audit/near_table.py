"""Fixed-jaw tip height above the table at the real contact readings of REAL_OBSERVATIONS §5,
under the real calibrated model (reading + (0, 4.4, -10.8, 5.2, 0) deg) and under the sim's
hidden truth (calibration.ts: same offsets, roll +65, plus 10 deg on the shoulder ramped in
between lift readings 5 and 35). FK and tip point copied from evidence/truth.py."""
import math
import numpy as np

CHAIN = [
    ("shoulder_pan", [0.0388353, -8.97657e-9, 0.0624], [3.14159, 4.18253e-17, -3.14159]),
    ("shoulder_lift", [-0.0303992, -0.0182778, -0.0542], [-1.5708, -1.5708, 0.0]),
    ("elbow_flex", [-0.11257, -0.028, 1.73763e-16], [-3.63608e-16, 8.74301e-16, 1.5708]),
    ("wrist_flex", [-0.1349, 0.0052, 3.62355e-17], [4.02456e-15, 8.67362e-16, -1.5708]),
    ("wrist_roll", [5.55112e-17, -0.0611, 0.0181], [1.5708, 0.0486795, 3.14159]),
]
FIXED_TIP = np.array([-0.012, 0.0, -0.1044, 1.0])
TABLE = -0.003
NAMES = [c[0] for c in CHAIN]


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def tip_cm(deg):
    m = np.eye(4)
    for (name, xyz, r), a in zip(CHAIN, deg):
        t = np.eye(4); t[:3, :3] = rpy(*r); t[:3, 3] = xyz
        z = np.eye(4); c, s = math.cos(math.radians(a)), math.sin(math.radians(a))
        z[:2, :2] = [[c, -s], [s, c]]
        m = m @ t @ z
    return 100 * ((m @ FIXED_TIP)[2] - TABLE)


def real_model(r):
    return [a + o for a, o in zip(r, [0, 4.4, -10.8, 5.2, 0])]


def sim_truth(r):
    share = min(1, max(0, (r[1] - 5) / 30))
    return [a + o for a, o in zip(r, [0, 4.4 + 10 * share, -10.8, 5.2, 65])]


EVENTS = {  # name: (readings, real truth)
    "early": ((0, -30.7, 93.4, 0, 0), "pressed into table"),
    "B": ((-46.5, 53.3, -49.0, 83.2, -64.7), "on table (0)"),
    "C": ((-48.7, 48.8, -44.7, 83.0, -71.7), "on table (0)"),
    "D": ((-48.7, 34.8, -36.0, 83.2, -71.7), "~0.5 above"),
    "Q": ((-44.8, 37.5, -31.4, 82.5, -62.5), "on queen base (~2-3?)"),
}
print(f"{'event':6} {'real model':>10} {'sim truth':>10}  real truth")
for k, (r, truth) in EVENTS.items():
    print(f"{k:6} {tip_cm(real_model(r)):10.1f} {tip_cm(sim_truth(r)):10.1f}  {truth}")
