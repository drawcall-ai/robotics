"""Hidden link bending in a run: true joint angle minus (reading + unloaded zero offset), for
shoulder_lift and elbow_flex, from the private /robot/joint_states and /follower/joint_states.
Usage: pixi run drawcall-robotics cat runs/<run> --topics /robot/joint_states,/follower/joint_states | python3 audit/hidden_bend.py"""
import json
import math
import sys

OFFSET = {1: 4.4 + 8 * -0.54, 2: -10.8 + 8 * -0.42}  # calibration.ts: FITTED + BENDING·REFERENCE
read = None
worst = []
for line in sys.stdin:
    d = json.loads(line)
    if d["topic"] == "/follower/joint_states":
        read = d["msg"]["position"]
        continue
    if read is None:
        continue
    true = [math.degrees(a) for a in d["msg"]["position"]]
    bend = {i: true[i] - (read[i] + OFFSET[i]) for i in OFFSET}
    worst.append((max(abs(b) for b in bend.values()), d["t"], bend))
worst.sort(reverse=True)
seen = []
for w, t, bend in worst:
    if all(abs(t - s) > 2 for s in seen):
        seen.append(t)
        print(f"t={t:7.2f} s  lift {bend[1]:+6.1f} deg  elbow {bend[2]:+6.1f} deg")
    if len(seen) == 10:
        break
big = sum(1 for w, _, _ in worst if w > 5) / max(1, len(worst))
print(f"samples with |bend| > 5 deg: {100 * big:.1f} %")
