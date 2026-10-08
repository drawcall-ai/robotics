"""Builder check of the failed real neck regrasp (DIFFERENCES.md §7): the queen standing on the
board where the real image rr_S1 shows it, (0.347, -0.005) (a temporary scene edit; the
solver's own estimate, (0.3405, 0.0134), assumed the old, larger sim queen), the arm at the readings
of rr_S1/rr_f*, then the gripper closing 45 -> 25 -> 15 -> 10 -> 5, stopping at load > 250."""
import json
import sys
sys.path.insert(0, "evidence")
from drive import Driver

d = Driver()
r = d.robot
S1 = {k: v for k, v in json.load(open("evidence/real3/rr_S1.json")).items()}
# The real plan's own joint path (plan_S1.json): survey, above the queen, then the descent.
for op in json.load(open("evidence/real3/plan_S1.json")):
    if op["op"] == "move":
        target = {**op["q"], "gripper": op["q"].get("gripper", d.goal["gripper"])}
        span = max(abs(target[j] - d.goal[j]) for j in op["q"])
        d.glide(seconds=max(1.0, span / op["speed"]), **target)
    if op["op"] == "descend":
        for q in op["steps"]:
            d.glide(seconds=0.5, steps=10, **q)
d.reach(**{**S1, "gripper": 45.0})
r.wait(1)
d.shoot("neck-S1")
rows = []
for g in (25.0, 15.0, 10.0, 5.0):
    while d.goal["gripper"] > g:
        d.goal["gripper"] = max(g, d.goal["gripper"] - 1.0)
        r.goal(**d.goal)
        r.wait(0.1)
        if abs(r.loads["gripper"]) > 250:
            break
    r.wait(0.5)
    rows.append((g, round(r.joints["gripper"], 1), int(r.loads["gripper"])))
    d.shoot(f"neck-f{int(g):02d}")
    if abs(r.loads["gripper"]) > 250:
        break
d.log("RESULT " + json.dumps({"name": "neck", "readings": {k: round(v, 1) for k, v in r.joints.items()}, "steps (goal, reading, load)": rows}))
