"""Builder check against the real board touches D1/D2 (closed jaws, pitch 72; evidence/real5/,
readings at contact): at those readings the sim's true lowest jaw point minus the board top
(0 would match the real arm), without and with a shoulder_lift term that ramps in with the elbow
reading. Usage: touch_fit.py [fold_deg] [elbow_from] [elbow_to]"""
import glob
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, "evidence")
from neck_diag import GRIPPER_CLOSED, TRAVEL, moving_tip  # noqa: E402
from truth import gripper  # noqa: E402

J = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
OFF = [0, -0.09, 1.55, -0.71]
BOARD = 0.0196


def lowest(readings, fold, lo, hi):
    share = min(1, max(0, (readings["elbow_flex"] - lo) / (hi - lo)))
    q = {j: math.radians(readings[j] + o) for j, o in zip(J, OFF)}
    q["shoulder_lift"] -= math.radians(fold * share)
    q["wrist_roll"] = math.radians(readings["wrist_roll"] + 100)
    g = gripper(q)
    jaw = GRIPPER_CLOSED + readings["gripper"] / 100 * TRAVEL
    pts = [(g @ np.array([-0.0119 + a * 0.004, b * 0.006, -0.08 + c * 0.0244, 1]))[2]
           for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)]
    pts.append(moving_tip(g, jaw)[2])
    return min(pts) - BOARD


fold, lo, hi = (float(a) for a in (sys.argv[1:4] if len(sys.argv) > 3 else (0, 47, 56)))
for path in sorted(glob.glob("evidence/real5/*.json")):
    r = json.load(open(path))
    print(f"{os.path.basename(path)[:-5]:22s} lift {r['shoulder_lift']:6.1f} elbow {r['elbow_flex']:5.1f}"
          f" | sim jaw - board: {lowest(r, 0, lo, hi) * 1000:5.1f} mm, with term {lowest(r, fold, lo, hi) * 1000:5.1f} mm")
