"""Builder check of REAL_OBSERVATIONS §4/§6.1 (needs the queen standing at (24.8, 20.8) cm; see
FIDELITY.md): a top-down grasp around the queen's neck at about 3.9 cm with a gentle squeeze
(contact + 3 units), then a 5 cm lift. Joint goals come from the true kinematics mapped through
the hidden calibration, i.e. the best a perfectly calibrated operator could command."""
import json
import math
import sys

import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, "evidence")
from drive import Driver  # noqa: E402
from truth import gripper  # noqa: E402

J = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
OFF = np.array([0, 4.4, -10.8, 5.2, 65.0])
LIM = np.array([101, 89, 86, 85.5, 179])


def true_from_readings(r):
    g = min(1, max(0, (r[1] - 5) / 30))
    return np.radians(np.array(r) + OFF + g * np.array([0, 10, 0, 0, 0]))


GRASP = np.array([-0.004, 0, -0.098, 1])  # between the jaw pads, by the fixed jaw


def solve(p, close_dir, r0):
    def f(r):
        m = gripper(dict(zip(J, true_from_readings(r))))
        return np.r_[(m @ GRASP)[:3] - p, 0.03 * (m[:3, 2] - [0, 0, 1]), 0.02 * (m[:3, 0] @ close_dir)]

    s = least_squares(f, r0, bounds=(-LIM, LIM))
    m = gripper(dict(zip(J, true_from_readings(s.x))))
    tilt = math.degrees(math.acos(np.clip(m[2, 2], -1, 1)))
    return dict(zip(J, s.x)), float(np.linalg.norm((m @ GRASP)[:3] - p)), tilt


d = Driver()
r = d.robot
target = np.array([0.248, 0.208, 0.039 - 0.003])
side = np.array([-0.208, 0.248, 0]) / np.hypot(0.248, 0.208)  # close across the radial direction
r0 = np.array([-40, 0, 0, 80, 0.0])
above, e1, t1 = solve(target + [0, 0, 0.06], side, r0)
at, e2, t2 = solve(target, side, np.array(list(above.values())))
d.log("RESULT " + json.dumps({"name": "grip-plan", "above": above, "at": at, "err_cm": [e1 * 100, e2 * 100], "tilt_deg": [t1, t2]}))
d.robot.p_gain(shoulder_lift=32, elbow_flex=32, wrist_flex=32, shoulder_pan=32)
d.glide(**{**above, "shoulder_lift": -10.0, "elbow_flex": 0.0, "gripper": 60})
d.reach(**above)
d.reach(**at)
d.report("grip-at")
d.shoot("grip-open")
# Close until the jaw stops (contact), then squeeze 3 units past it, as the real attempt did.
g = 60.0
while g > 0:
    g -= 2
    d.goal["gripper"] = g
    r.goal(**d.goal)
    r.wait(0.3)
    if r.joints["gripper"] - g > 3:
        break
contact = r.joints["gripper"]
d.goal["gripper"] = contact - 3
r.goal(**d.goal)
r.wait(1.0)
d.shoot("grip-closed")
d.log("RESULT " + json.dumps({"name": "grip-squeeze", "contact": contact, "goal": d.goal["gripper"], "load": r.loads["gripper"]}))
lift, _, _ = solve(target + [0, 0, 0.05], side, np.array(list(at.values())))
d.glide(seconds=3, **lift)
r.wait(2)
d.shoot("grip-lifted")
d.report("grip-lifted")
