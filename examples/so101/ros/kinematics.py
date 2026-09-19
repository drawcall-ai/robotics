"""SO101 hardware transforms and bounded numerical inverse kinematics."""

import math

import numpy as np
from scipy.optimize import least_squares

NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
LIMITS = np.array(
    [
        [-1.91986, 1.91986],
        [-1.74533, 1.74533],
        [-1.69, 1.69],
        [-1.65806, 1.65806],
        [-2.74385, 2.84121],
    ]
)


def rotation(r, p, y):
    cr, sr, cp, sp, cy, sy = (
        math.cos(r),
        math.sin(r),
        math.cos(p),
        math.sin(p),
        math.cos(y),
        math.sin(y),
    )
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def transform(xyz, rpy):
    t = np.eye(4)
    t[:3, :3] = rotation(*rpy)
    t[:3, 3] = xyz
    return t


ORIGINS = [
    transform(p, r)
    for p, r in [
        ((0.0388353, 0, 0.0624), (math.pi, 0, -math.pi)),
        ((-0.0303992, -0.0182778, -0.0542), (-math.pi / 2, -math.pi / 2, 0)),
        ((-0.11257, -0.028, 0), (0, 0, math.pi / 2)),
        ((-0.1349, 0.0052, 0), (0, 0, -math.pi / 2)),
        ((0, -0.0611, 0.0181), (math.pi / 2, 0.0486795, math.pi)),
    ]
]
CAMERA = (
    transform((-0.015, 0.024, -0.0315), (-1.5708, -0.0008, -1.5708))
    @ transform((-0.04941, -0.03813, 0.0175), (0, -1.5708, -2.0076))
    @ transform((0, 0, 0.027), (0, 0, 0))
)
TCP = np.array([0.0031, 0, -0.090, 1.0])


def fk(q):
    t = np.eye(4)
    for origin, angle in zip(ORIGINS, q):
        t = t @ origin @ transform((0, 0, 0), (0, 0, float(angle)))
    return t


def ik(position, seed=None, roll=0):
    """Position and downward finger direction, with preferred wrist roll."""
    target = np.array(position)
    best = None
    seeds = [
        np.array(seed[:5]) if seed is not None else np.array([0, -0.4, 0.7, 0.9, roll]),
        np.array([0, -0.8, 1.2, 0.8, roll]),
    ]
    for q in seeds:
        q = q.copy()

        def residual(a):
            t = fk(a)
            return np.r_[
                (t @ TCP)[:3] - target,
                (0.10 if target[2] < 0.04 else 0.025)
                * (t[:3, 2] - np.array([0, 0, 1])),
                0.02 * (a[4] - roll),
            ]

        fit = least_squares(
            residual,
            np.clip(q, LIMITS[:, 0] + 1e-5, LIMITS[:, 1] - 1e-5),
            bounds=(LIMITS[:, 0], LIMITS[:, 1]),
            max_nfev=100,
            ftol=1e-8,
            gtol=1e-8,
            xtol=1e-8,
        )
        q = fit.x
        score = np.linalg.norm(residual(q))
        if best is None or score < best[0]:
            best = (score, q)
    return best[1], best[0]
