"""Builder check of a neck grasp in the policy's geometry (needs the queen standing at QX, QY on the
board; temporary scene edit): approach pitch PITCH° (90 = tool vertical), jaws closing CLOSE
(tangential or radial), the jaw tip H m above the board, squeeze SQ units past contact, a 5 cm lift,
then a HOLD s hold logging the gripper reading, load and true jaw angle. Goals come from the true
kinematics mapped through the calibration, as in grip.py."""
import json
import math
import os
import sys

import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, "evidence")
from drive import Driver  # noqa: E402
from truth import TIP, gripper  # noqa: E402

J = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
# calibration.ts: model = reading + FITTED (roll: ROLL_ERROR), the shoulder 2.2° back when folded.
OFF = np.array([0, -0.09, 1.55, -0.71, 100.0])
LIM = np.array([101.6, 95.4, 90.3, 85.9, 180])
BOARD_TOP = 0.02  # base_link z of the measured board's top

env = os.environ.get
QX, QY = float(env("QX", 0.32)), float(env("QY", 0.02))
PITCH, CLOSE = float(env("PITCH", 72)), env("CLOSE", "tangential")
H, SQ, HOLD = float(env("H", 0.035)), float(env("SQ", 3)), float(env("HOLD", 20))


def true_from_readings(r):
    folded = 2.2 * min(1, max(0, (r[2] - 47) / 9))
    return np.radians(np.array(r) + OFF - np.array([0, folded, 0, 0, 0]))


radial = np.array([QX - 0.0388, QY, 0]) / math.hypot(QX - 0.0388, QY)
tangential = np.array([-radial[1], radial[0], 0])
p = math.radians(PITCH)
axis = math.sin(p) * np.array([0, 0, 1]) - math.cos(p) * radial  # tip to wrist
closing = tangential if CLOSE == "tangential" else radial


def solve(point, r0):
    def f(r):
        m = gripper(dict(zip(J, true_from_readings(r))))
        return np.r_[(m @ TIP)[:3] - point, 0.03 * (m[:3, 2] - axis), 0.03 * (m[:3, 0] - closing)]

    best = None
    for roll in (r0[4], r0[4] + 180, r0[4] - 180):
        if abs(roll) >= LIM[4]:
            continue
        s = least_squares(f, np.clip([*r0[:4], roll], -LIM + 0.01, LIM - 0.01), bounds=(-LIM, LIM))
        if best is None or s.cost < best.cost:
            best = s
    m = gripper(dict(zip(J, true_from_readings(best.x))))
    return dict(zip(J, best.x)), float(np.linalg.norm((m @ TIP)[:3] - point))


d = Driver()
r = d.robot
target = np.array([QX, QY, BOARD_TOP + H])
above, e1 = solve(target + 0.06 * axis, np.array([0, 0, 0, 70, 0.0]))
at, e2 = solve(target, np.array(list(above.values())))
lift, e3 = solve(target + [0, 0, 0.05], np.array(list(at.values())))
d.log("RESULT " + json.dumps({"name": "pose-plan", "env": dict(QX=QX, QY=QY, PITCH=PITCH, CLOSE=CLOSE, H=H, SQ=SQ),
                              "at": {k: round(v, 1) for k, v in at.items()}, "err_mm": [round(e * 1000, 1) for e in (e1, e2, e3)]}))
d.robot.p_gain(shoulder_lift=32, elbow_flex=32, wrist_flex=32, shoulder_pan=32)
d.glide(**{**above, "shoulder_lift": -10.0, "elbow_flex": 0.0, "gripper": 60})
d.reach(**above)
d.reach(**at)
d.report("pose-at")
m = gripper(dict(d.truth.q))
d.log("RESULT " + json.dumps({"name": "pose-true", "tip_mm": [round(v * 1000, 1) for v in (m @ TIP)[:3] - target],
                              "x_dot_closing": round(float(m[:3, 0] @ closing), 3), "z_dot_axis": round(float(m[:3, 2] @ axis), 3)}))
# Close until the jaw stops (contact), then squeeze SQ units past it.
g = 60.0
while g > 0:
    g -= 2
    d.goal["gripper"] = g
    r.goal(**d.goal)
    r.wait(0.3)
    if r.joints["gripper"] - g > 3:
        break
contact = r.joints["gripper"]
d.goal["gripper"] = contact - SQ
r.goal(**d.goal)
r.wait(1.0)
d.shoot("pose-closed")
d.log("RESULT " + json.dumps({"name": "pose-squeeze", "contact": round(contact, 1), "goal": round(d.goal["gripper"], 1), "load": r.loads["gripper"]}))
d.glide(seconds=3, **lift)
rows = []
for k in range(int(HOLD / 2) + 1):
    jaw = math.degrees(d.truth.q.get("gripper", float("nan")))
    rows.append((round(r.time, 1), round(r.joints["gripper"], 2), int(r.loads["gripper"]), round(jaw, 2)))
    r.wait(2)
d.shoot("pose-held")
d.log("RESULT " + json.dumps({"name": "pose-hold", "cols": "t, gripper reading, load, true jaw deg", "rows": rows}))
