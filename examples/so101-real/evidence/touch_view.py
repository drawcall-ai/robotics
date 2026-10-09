"""Builder check: the sim's wrist view at the readings of real board touches (evidence/real5/),
jaws closed, saved as sim-touch-<name>.jpg, to compare where the jaw tips appear."""
import json
import sys
sys.path.insert(0, "evidence")
from drive import Driver

d = Driver()
for name in sys.argv[1:]:
    if name.startswith("--"):
        continue
    q = json.load(open(f"evidence/real5/{name}.json"))
    d.glide(seconds=3, **{**q, "shoulder_lift": q["shoulder_lift"] - 10})
    d.reach(tol=0.3, **q)
    d.shoot(f"touch-{name}")
    d.glide(seconds=2, **{**q, "shoulder_lift": q["shoulder_lift"] - 10})
