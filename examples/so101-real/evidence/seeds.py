"""Builder check of the seed variation: the wrist view at rest and from the survey pose (pan 0,
lift 0.4, elbow -13, wrist 80) and 45° to the left, saved as sim-seed-<tag>-{rest,survey,left}.jpg."""
import sys
sys.path.insert(0, "evidence")
from drive import Driver

tag = sys.argv[1]
d = Driver()
d.robot.wait(1)
d.shoot(f"seed-{tag}-rest")
d.glide(seconds=4, shoulder_pan=0.0, shoulder_lift=0.4, elbow_flex=-13.0, wrist_flex=80.0, wrist_roll=0.0)
d.robot.wait(1)
d.shoot(f"seed-{tag}-survey")
d.glide(seconds=3, shoulder_pan=-45.0)
d.robot.wait(1)
d.shoot(f"seed-{tag}-left")
