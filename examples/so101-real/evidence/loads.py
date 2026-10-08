"""Builder check of the gravity loads against the real arm: the survey pose of the 2026-10-08
evening session (DIFFERENCES.md §4) and REAL_OBSERVATIONS §2's extended pose (converted to
today's readings), at P=16. Logs loads, goal − present and the link torques (in /sim/audit)."""
import json
import sys
sys.path.insert(0, "evidence")
from drive import Driver, shifted

d = Driver()
r = d.robot
POSES = {
    "survey": dict(shoulder_pan=0.0, shoulder_lift=0.4, elbow_flex=-13.0, wrist_flex=80.0, wrist_roll=0.0, gripper=5.0),
    "extended": {**shifted(dict(shoulder_pan=60.0, shoulder_lift=27.0, elbow_flex=-31.0, wrist_flex=78.0, wrist_roll=0.0)), "gripper": 50.0},
}
J = ["shoulder_lift", "elbow_flex", "wrist_flex"]
for name, p in POSES.items():
    d.glide(seconds=4, **{**p, "shoulder_lift": p["shoulder_lift"] - 15})
    d.glide(seconds=3, **p)  # from above, as a descent
    r.settle(timeout=3)
    r.wait(2)
    plain = {j: (round(p[j] - r.joints[j], 1), int(r.loads[j])) for j in J}
    reached = d.reach(**p)  # integral nudges, as the real survey did
    r.wait(2)
    nudged = {j: (round(d.goal[j] - r.joints[j], 1), int(r.loads[j])) for j in J}
    # Goal = present, held 2 s: what friction carries.
    for j in J:
        d.goal[j] = r.joints[j]
    r.goal(**d.goal)
    start = dict(r.joints)
    r.wait(2)
    held = {j: (round(r.joints[j] - start[j], 2), int(r.loads[j])) for j in J}
    d.log("RESULT " + json.dumps({"name": name, "t": round(r.time, 1), "goal_minus_present_and_load": {"plain": plain, "nudged": nudged}, "goal_eq_present_drift_and_load": held}))
