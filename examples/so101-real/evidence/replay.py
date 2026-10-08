"""Builder replay of the real arm's plans (`evidence/real3/plan_*.json`, the solver's exec.py format):
move (joint goals at a speed in °/s, then nudged until the readings arrive, as the real
harness's integral did), descend (joint steps, stopping when the shoulder load drops by
`drop`), grip (closing in 1-unit steps, stopping above `until_load`), shot. Usage:
replay.py <tag> <plan> [<plan> ...]; logs RESULT lines and saves sim-<tag>-<shot>.jpg."""
import json
import sys
sys.path.insert(0, "evidence")
from drive import Driver

d = Driver()
r = d.robot
tag = sys.argv[1]
for path in [a for a in sys.argv[2:] if not a.startswith("--")]:
    for op in json.load(open(path)):
        kind = op["op"]
        if kind == "move":
            target = {**op["q"]}
            span = max(abs(target[j] - d.goal[j]) for j in target)
            d.glide(seconds=max(0.5, span / op.get("speed", 5)), **target)
            # The real harness nudges goals until the readings arrive (an integral term).
            d.reach(tol=0.5, **target)
        elif kind == "descend":
            start = r.loads["shoulder_lift"]
            for q in op["steps"]:
                d.glide(seconds=0.5, steps=10, **q)
                d.reach(tol=0.5, rounds=4, **q)
                # The real descent onto the lying queen ran to its end (model z 0.004); the sim's
                # loads sit higher, so its drop test only stops descents marked require_contact.
                if op.get("require_contact") and start - r.loads["shoulder_lift"] > op.get("drop", 60):
                    d.log(f"descend stopped by load {r.loads['shoulder_lift']} (from {start})")
                    break
        elif kind == "grip":
            while d.goal["gripper"] > op["g"]:
                d.goal["gripper"] = max(op["g"], d.goal["gripper"] - 1.0)
                r.goal(**d.goal)
                r.wait(0.1)
                if abs(r.loads["gripper"]) > op.get("until_load", 1000):
                    break
            r.wait(0.5)
            d.log("RESULT " + json.dumps({"name": f"{tag}-grip", "gripper": round(r.joints["gripper"], 1), "load": int(r.loads["gripper"])}))
        elif kind == "shot":
            d.shoot(f"{tag}-{op['name']}")
            d.report(f"{tag}-{op['name']}")
