"""Builder diagnostic (Round 4, parts 2 and 3) from the sim's truth mapping (reading + the real
calibration's offsets, wrist_roll + 100 deg; at given readings the sim's jaws sit exactly there).
2: the real neck-grasp pre-close readings (evidence/real4/): true jaw tips against the board and
   against the queen axis as the solver's model placed it (model tool point - (-1.5, -7.5) mm).
3: the board touch at pitch 72, jaws closed, y -0.05, x 0.33 / 0.215 / 0.19: the model tool-point
   height at which the true lowest jaw point meets the board top, and (true tip z - board z) when
   the model tool point is on the board."""
import glob
import json
import math
import os
import sys

import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, "evidence")
from neck_diag import BOARD, FIXED_TIP, TCP, describe, model_tcp, moving_tip, true_angles, GRIPPER_CLOSED, TRAVEL  # noqa: E402
from truth import gripper  # noqa: E402

J = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
OFF = [0, -0.09, 1.55, -0.71, 0]
out = open("evidence/neck_diag2.jsonl", "w")
print("== 2: real pre-close readings")
for path in sorted(glob.glob("evidence/real4/*.json")):
    readings = json.load(open(path))
    queen = model_tcp(readings)[:2] - np.array([-0.0015, -0.0075])
    row = describe(os.path.basename(path)[:-5], true_angles(readings), readings, queen)
    out.write(json.dumps(row) + "\n")
    g = readings
    print(row["name"], "| lift %.1f elbow %.1f wrist %.1f grip %.0f" % (g["shoulder_lift"], g["elbow_flex"], g["wrist_flex"], g["gripper"]))
    for k in ["model tcp", "tcp", "fixed tip", "moving tip"]:
        print("   %-10s" % k, "from model's queen axis", row[k]["from queen axis mm"], "above board", row[k]["above board mm"])


def lowest(readings):
    """Lowest true jaw point (pad corners, jaws closed), base frame z."""
    q = true_angles(readings)
    g = gripper(q)
    jaw = GRIPPER_CLOSED
    pts = [(g @ np.array([-0.0119 + a * 0.004, b * 0.006, -0.08 + c * 0.0244, 1]))[2]
           for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)]
    pts.append(moving_tip(g, jaw)[2])
    return min(pts)


def ik(x, y, z, pitch=72):
    """Readings putting the model tool point at (x, y, z), the jaws pointing down `pitch` degrees
    below horizontal and outwards, roll reading 0."""
    out_dir = np.array([x, y]) / np.hypot(x, y)
    d = np.r_[out_dir * math.cos(math.radians(pitch)), -math.sin(math.radians(pitch))]

    def f(v):
        r = dict(zip(J, list(v) + [0.0]))
        m = gripper({j: math.radians(r[j] + o) for j, o in zip(J, OFF)})
        return np.r_[(m @ TCP)[:3] - [x, y, z], 0.05 * (-m[:3, 2] - d)]

    best = None
    for seed in ([0, 0, 0, 70], [0, -40, 45, 70], [0, 30, -50, 80]):
        s = least_squares(f, seed, bounds=([-100, -95, -90, -85.9], [100, 95, 90, 85.9]))
        if best is None or s.cost < best.cost:
            best = s
    return dict(zip(J, list(best.x) + [0.0])), math.sqrt(2 * best.cost)


print("== 3: board touch, pitch 72, jaws closed, y -0.05")
for x in (0.33, 0.215, 0.19):
    zs = np.arange(0.06, -0.03, -0.001)
    contact = None
    for z in zs:
        r, err = ik(x, -0.05, z)
        if lowest(r) <= BOARD:
            contact = (z, r, err)
            break
    r0, err0 = ik(x, -0.05, BOARD)
    row = {"x": x, "model tcp z at true contact": round(contact[0], 4), "board z": BOARD,
           "true lowest jaw z - board when model tcp on board mm": round((lowest(r0) - BOARD) * 1000, 1),
           "readings at contact": {k: round(v, 1) for k, v in contact[1].items()}, "ik error mm": round(contact[2] * 1000, 1)}
    out.write(json.dumps(row) + "\n")
    print(json.dumps(row))
