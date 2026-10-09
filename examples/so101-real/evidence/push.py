"""Builder check of the real push (DIFFERENCES.md Round 5, evidence/real7/push_joints.csv): replay the
real readings from the rest pose (0 s) to the end of the push (168.1 s) at real timing, with an
integral correction like the real harness, the queen standing where the real one started,
(0.183, 0.009) (a temporary scene edit). Real: the queen slid ~4 cm outward and stayed upright."""
import csv
import sys
sys.path.insert(0, "evidence")
from drive import Driver

J = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
rows = list(csv.reader(open("evidence/real7/push_joints.csv")))
t0 = float(rows[0][0])
path = [(float(r[0]) - t0, dict(zip(J, map(float, r[1:7])))) for r in rows]
path = [(t, q) for t, q in path if t <= 168.1]
d = Driver()
r = d.robot
bias = {j: 0.0 for j in J[:5]}
start = r.time
for t, q in path:
    while r.time - start < t - path[0][0]:
        r.wait(0.02)
    for j in J[:5]:
        bias[j] += 0.2 * (q[j] - r.joints[j])
        bias[j] = max(-8.0, min(8.0, bias[j]))
    d.goal.update({j: q[j] + (bias[j] if j in bias else 0.0) for j in J})
    r.goal(**d.goal)
r.wait(2)
d.shoot("push-end")
d.report("push-end")
