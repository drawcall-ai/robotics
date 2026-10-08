"""Builder check of the real touch test (2026-10-08 evening, DIFFERENCES.md §3): replay its joint
goals, gripper closed, tool pitched ~85° down at (0.22, -0.17), stepping the model tool point down
3 mm at 3°/s. Real: the shoulder load fell from its +30 baseline to 0 at model z -0.006 (the
table is at -0.003), so the jaw tip sits 3 mm above the URDF gripper_frame point."""
import json
import sys
sys.path.insert(0, "evidence")
from drive import Driver
from truth import tip_height

steps = json.load(open("evidence/touch_steps.json"))
d = Driver()
r = d.robot
d.glide(seconds=4, **{"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": -13.0, "wrist_flex": 80.0, "wrist_roll": 0.0, "gripper": 3.4})
d.glide(seconds=4, **steps[0][1])
r.settle(timeout=3)
r.wait(1)
rows = []
for z, q in steps[1:]:
    seconds = max(abs(q[j] - d.goal[j]) for j in q) / 3.0 + 0.1
    d.glide(seconds=seconds, steps=max(2, int(seconds / 0.05)), **q)
    r.wait(0.5)
    rows.append((round(z, 4), int(r.loads["shoulder_lift"]), round(tip_height(d.truth.q) * 100, 2)))
    if r.loads["shoulder_lift"] <= 0:
        break
d.log("RESULT " + json.dumps({"name": "touch", "cols": "model tool-point z (m), shoulder load, true jaw tip above table (cm)", "rows": rows}))
