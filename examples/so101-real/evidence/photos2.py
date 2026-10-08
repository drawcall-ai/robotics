"""Builder check against the real views of the 2026-10-08 evening session (`evidence/real2/`, readings
in today's calibration): drive to each view's readings and save the sim's view."""
import glob
import json
import os
import sys
sys.path.insert(0, "evidence")
from drive import Driver

d = Driver()
d.robot.p_gain(shoulder_pan=32, shoulder_lift=32, elbow_flex=32, wrist_flex=32, wrist_roll=32)
for path in sorted(glob.glob("evidence/real2/*.json")):
    name = os.path.basename(path)[:-5]
    p = {k: float(v) for k, v in json.load(open(path)).items()}
    d.glide(**{**p, "shoulder_lift": min(p["shoulder_lift"], 0.0)})
    d.reach(**p)
    d.report(name)
    d.shoot(name)
