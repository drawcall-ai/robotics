"""Builder check of audit 2's launch: attempt 2's goals at t = 208 s and its gripper closing
sequence, with the queen lying where it lay then (temporary scene edit). Logs the shoulder's
reading against its true angle and the queen's height."""
import json
import math
import sys
sys.path.insert(0, "evidence")
from drive import Driver

d = Driver()
r = d.robot
G = dict(shoulder_pan=-46.0, shoulder_lift=22.0, elbow_flex=1.9, wrist_flex=51.2, wrist_roll=103.0, gripper=20.0)
d.glide(seconds=3, **{**G, "shoulder_lift": 0.0})
d.glide(seconds=3, **G)
r.wait(2)
rows = []
for g in list(range(18, -1, -2)) + [0] * 10:
    d.goal["gripper"] = float(g)
    r.goal(**d.goal)
    r.wait(0.2)
    q = d.truth.q
    rows.append((round(r.time, 1), g, round(r.joints["shoulder_lift"], 1), round(math.degrees(q["shoulder_lift"]) - 4.4, 1),
                 round(r.joints["elbow_flex"], 1), round(math.degrees(q["elbow_flex"]) + 10.8, 1), int(r.loads["gripper"])))
d.log("RESULT " + json.dumps({"name": "launch", "cols": "t, gripper goal, lift reading, true lift - 4.4, elbow reading, true elbow + 10.8, gripper load", "rows": rows}))
d.report("launch-end")
