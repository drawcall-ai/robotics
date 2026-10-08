"""Builder check of the overheating trip (REAL_OBSERVATIONS §2): the sag correction pressing the
jaw into the table, as at B, holds the shoulder at full load until it trips and the arm drops."""
import json
import sys
sys.path.insert(0, "evidence")
from drive import Driver

d = Driver()
r = d.robot
B = dict(shoulder_pan=-46.5, shoulder_lift=53.3, elbow_flex=-49.0, wrist_flex=83.2, wrist_roll=-64.7, gripper=100)
d.glide(**{**B, "shoulder_lift": -10.0, "elbow_flex": 0.0})
d.glide(seconds=4, **B)
d.goal["shoulder_lift"] = 90.0  # what a sag correction does when the table blocks the arm
r.goal(**d.goal)
t0 = r.time
rows = []
while r.time - t0 < 400:
    r.wait(10)
    rows.append((round(r.time - t0), r.status["temperature"]["shoulder_lift"], r.status["torque"]["shoulder_lift"],
                 int(r.loads["shoulder_lift"]), round(r.joints["shoulder_lift"], 1)))
    if not r.status["torque"]["shoulder_lift"]:
        break
r.wait(3)
d.log("RESULT " + json.dumps({"name": "thermal", "rows": rows, "after_trip": dict(r.joints), "status": r.status}))
d.report("after-trip")
