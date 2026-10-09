"""Builder replay of a real pick-and-place joints log (policy/real/autologs/<run>/joints.csv: time, six
readings, six loads, ...) from the rest pose at real timing, with an integral correction like the
real harness. The gripper's goal is its reading minus load / 62 (the squeeze calibration: 3 units
past contact give load ≈186), so a held piece is squeezed as hard as the real one was. Logs the
jaw tip point and gripper load every second, to set against the queen's heartbeat."""
import csv
import json
import sys

import numpy as np

sys.path.insert(0, "evidence")
from drive import Driver  # noqa: E402
from truth import gripper  # noqa: E402

TIP = np.array([-0.004, 0, -0.1044 + 0.004, 1])  # between the jaw tips (JAW_SHIFT)
J = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
log, end = sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 and not sys.argv[2].startswith("-") else 1e9
rows = list(csv.reader(open(log)))
t0 = float(rows[0][0])
path = []
for r in rows:
    t = float(r[0]) - t0
    if t > end:
        break
    q = dict(zip(J, map(float, r[1:7])))
    q["gripper"] -= float(r[12]) / 62
    path.append((t, q))
d = Driver()
r = d.robot
bias = {j: 0.0 for j in J[:5]}
start = r.time
mark = 0.0
for t, q in path:
    while r.time - start < t:
        r.wait(0.02)
    for j in J[:5]:
        bias[j] = max(-8.0, min(8.0, bias[j] + 0.2 * (q[j] - r.joints[j])))
    d.goal.update({j: q[j] + bias.get(j, 0.0) for j in J})
    r.goal(**d.goal)
    if t >= mark:
        mark = t + 1.0
        tq = {k: v for k, v in d.truth.q.items() if k in J[:5]}
        d.log("RESULT " + json.dumps({"name": "pnp", "t": round(t, 1), "gripper": round(r.joints["gripper"], 1),
                                      "load": int(r.loads["gripper"]), "shoulder_load": int(r.loads["shoulder_lift"]), "loads": {k: int(v) for k, v in r.loads.items()},
                                      "readings": {k: round(r.joints[k], 1) for k in J[:4]}, "want": {k: round(q[k], 1) for k in J[:4]}, "bias": {k: round(v, 1) for k, v in bias.items() if k != "wrist_roll"},
                                      "tip": [round(v, 4) for v in (gripper(tq) @ TIP)[:3]] if tq else None}))
r.wait(3)
d.shoot("pnp-end")
d.report("pnp-end")
