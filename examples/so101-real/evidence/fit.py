"""Builder check: fit the hidden scene and arm-error parameters to features measured by eye in
the real wrist-camera photos (pixel coordinates, 1280x960), by projecting them through the
calibrated camera (REAL_OBSERVATIONS §3) at the photos' joint readings."""
import json
import math
import sys

import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, "evidence")
from truth import gripper  # noqa: E402

T = np.array([[0.1523, -0.7021, -0.6956, 0.1006], [-0.9879, -0.0883, -0.1273, 0.0191],
              [0.028, 0.7065, -0.7071, -0.0294], [0, 0, 0, 1]])
J = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
OFF = np.array([0, 4.4, -10.8, 5.2, 0])
S, BORDER, TOP = 0.0314, 0.015, 0.027
PHOTOS = {
    "26-view": dict(q=(-46.6, 29.0, -34.4, 83.0, -0.2), Q=(722, 575), S=(1105, 478),
                    edge=[(400, 862), (800, 645), (1200, 415)]),
    "30-sB": dict(q=(-48.7, 19.7, -35.9, 83.3, 9.9), F=(727, 98), Q=(728, 518), S=(985, 507),
                  edge=[(400, 775), (800, 585), (1200, 395)]),
    "30-sA": dict(q=(-48.7, 19.7, -35.9, 83.3, -30.0), F=(535, 100), Q=(820, 445), S=(1125, 280),
                  edge=[(860, 640), (1020, 400), (1195, 160)]),
    "20-pre": dict(q=(-44.5, 31.5, -38.6, 82.5, -61.2), F=(410, 385), Q=(825, 490), S=(960, 240),
                   edge=[(950, 100), (960, 500), (965, 900)]),
}


def camera(q, lift_error, ramp):
    g = min(1, max(0, (q[1] - 5) / ramp))
    true = np.array(q) + OFF + g * np.array([0, lift_error, 0, 0, 0])
    return gripper({j: math.radians(a) for j, a in zip(J, true)}) @ T


def project(c, p):
    d = np.linalg.inv(c) @ np.r_[p, 1]
    return np.array([640 + 1061 * d[0] / d[2], 480 + 1061 * d[1] / d[2]])


def world(v):
    bx, by, byaw, qx, qy, qh, lift_error = v
    c, s = math.cos(byaw), math.sin(byaw)
    u = 4 * S + BORDER
    seam = np.array([bx - u * s, by + u * c, TOP])
    along = np.array([c, s, 0.0])  # the h-side edge runs along the ranks
    base = np.array([qx, qy, -0.003 + 0.0145])
    finial = base + 0.071 * np.array([math.cos(qh), math.sin(qh), 0]) + [0, 0, -0.008]
    return seam, along, base, finial


def residuals(v, ramp=30, report=False):
    seam, along, base, finial = world(v)
    out, rows = [], {}
    for name, ph in PHOTOS.items():
        c = camera(ph["q"], v[6], ramp)
        r = {}
        r["S"] = project(c, seam) - ph["S"]
        r["Q"] = project(c, base) - ph["Q"]
        if "F" in ph:
            r["F"] = project(c, finial) - ph["F"]
        a, b = project(c, seam - 0.1 * along), project(c, seam + 0.1 * along)
        n = np.array([-(b - a)[1], (b - a)[0]]) / np.linalg.norm(b - a)
        r["edge"] = np.array([(np.array(p) - a) @ n for p in ph["edge"]])
        out += [x for k in r for x in np.atleast_1d(r[k])]
        rows[name] = {k: np.round(x).astype(int).tolist() for k, x in r.items()}
    return rows if report else np.array(out)


if __name__ == "__main__":
    measured = [0.2935, 0.0165, math.radians(2), 0.23, 0.174, math.atan2(6.8, 3.3), 10.0]
    for ramp in (20, 30):
        print(f"ramp {ramp}: measured scene, residual px", json.dumps(residuals(measured, ramp, True)))
        fit = least_squares(residuals, measured, kwargs={"ramp": ramp}, loss="soft_l1", f_scale=20)
        v = fit.x
        print(f"  fit: board ({v[0]*100:.1f}, {v[1]*100:.1f}) cm yaw {math.degrees(v[2]):.1f}, queen base ({v[3]*100:.1f}, {v[4]*100:.1f}) "
              f"heading {math.degrees(v[5]):.0f}, lift error {v[6]:.1f} deg")
        print("  residual px", json.dumps(residuals(v, ramp, True)))


def tip_height(q, err, ramp):
    from truth import tip_height as h
    g = min(1, max(0, (q[1] - 5) / ramp))
    true = np.array(q) + OFF + g * np.array(err)
    return h({j: math.radians(a) for j, a in zip(J, true)})


def fixed_error(err, ramp):
    """Fit only the scene with the arm error fixed to `err` (degrees per joint at full share)."""
    global camera

    def cam(q, _lift, _ramp):
        g = min(1, max(0, (q[1] - 5) / ramp))
        true = np.array(q) + OFF + g * np.array(err)
        return gripper({j: math.radians(a) for j, a in zip(J, true)}) @ T

    saved, camera = camera, cam
    start = [0.2935, 0.0165, math.radians(2), 0.23, 0.174, math.atan2(6.8, 3.3), 0]
    fit = least_squares(lambda v: residuals(np.r_[v, 0]), start[:6], loss="soft_l1", f_scale=20)
    rows = residuals(np.r_[fit.x, 0], report=True)
    camera = saved
    return fit.x, rows


CONTACTS = {"B": (-46.5, 53.3, -49.0, 83.2, -64.7), "C": (-48.7, 48.8, -44.7, 83.0, -71.7),
            "D": (-48.7, 34.8, -36.0, 83.2, -71.7), "early": (0, -30.7, 93.4, 0, 0)}
if __name__ == "__main__":
    for label, err, ramp in [("lift 10, ramp 20", [0, 10, 0, 0, 0], 20), ("lift 10, ramp 30", [0, 10, 0, 0, 0], 30),
                             ("elbow 18.4, ramp 20", [0, 0, 18.4, 0, 0], 20),
                             ("lift 5 + elbow 9, ramp 20", [0, 5, 9, 0, 0], 20)]:
        v, rows = fixed_error(err, ramp)
        rms = np.sqrt(np.mean(np.concatenate([np.ravel(np.array(x, float)) for r in rows.values() for x in r.values()]) ** 2))
        tips = {k: round(tip_height(q, err, ramp) * 100, 1) for k, q in CONTACTS.items()}
        print(f"{label}: rms {rms:.0f} px, board ({v[0]*100:.1f}, {v[1]*100:.1f}) yaw {math.degrees(v[2]):.1f}, "
              f"queen ({v[3]*100:.1f}, {v[4]*100:.1f}) heading {math.degrees(v[5]):.0f}; tip above table cm {tips}")
        print("   ", json.dumps(rows))
    # The board as measured in the calibration region; only the queen fitted.
    board = [0.2935, 0.0165, math.radians(2)]

    def cam(q, _lift, _ramp):
        g = min(1, max(0, (q[1] - 5) / 30))
        true = np.array(q) + OFF + g * np.array([0, 10, 0, 0, 0])
        return gripper({j: math.radians(a) for j, a in zip(J, true)}) @ T

    camera = cam
    fit = least_squares(lambda v: residuals(np.r_[board, v, 0]), [0.23, 0.174, math.atan2(6.8, 3.3)], loss="soft_l1", f_scale=20)
    v = fit.x
    print(f"board fixed: queen base ({v[0]*100:.1f}, {v[1]*100:.1f}) heading {math.degrees(v[2]):.0f}")
    print("   ", json.dumps(residuals(np.r_[board, v, 0], report=True)))
    print("measured queen:", json.dumps(residuals(np.r_[board, 0.23, 0.174, math.atan2(6.8, 3.3), 0], report=True)))
