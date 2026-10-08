"""Builder regression check of the withdrawal rule (TASK=scatter, the queen standing off d8 by a
temporary scene edit, success held off for 40 s): close the jaws on the queen and hold, open them
wide around it, then lift away. Logs the time of each phase; success must come only after the lift."""
import json
import sys
sys.path.insert(0, "evidence")
from drive import Driver

d = Driver()
r = d.robot
S1 = json.load(open("evidence/real3/rr_S1.json"))
for op in json.load(open("evidence/real3/plan_S1.json")):
    if op["op"] == "move":
        d.glide(seconds=3, **op["q"])
        d.reach(tol=0.5, **op["q"])
    if op["op"] == "descend":
        for q in op["steps"]:
            d.glide(seconds=0.5, steps=10, **q)
            d.reach(tol=0.5, rounds=4, **q)
d.glide(seconds=3, gripper=5.0)
d.log(f"PHASE held {r.time:.1f} gripper {r.joints['gripper']:.1f} load {r.loads['gripper']:.0f}")
r.wait(max(0.0, 45 - r.time) + 5)
d.glide(seconds=2, gripper=100.0)
d.log(f"PHASE open {r.time:.1f}")
r.wait(5)
d.glide(seconds=3, shoulder_lift=d.goal["shoulder_lift"] - 25, elbow_flex=d.goal["elbow_flex"] + 15)
d.log(f"PHASE lifted {r.time:.1f}")
r.wait(5)
