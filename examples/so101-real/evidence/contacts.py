"""Builder check of REAL_OBSERVATIONS §5: command the real readings of the contact events, as the
real attempt reached them (descending from above with the gripper open), and record the true
fixed-jaw height and loads. Plain = goal set to the readings (P=16, no correction);
reached = goals nudged until the readings match, as the real controller's sag correction did."""
import sys
sys.path.insert(0, "evidence")
from drive import Driver, shifted

d = Driver()
POSES = {
    "B": (-46.5, 53.3, -49.0, 83.2, -64.7),
    "C": (-48.7, 48.8, -44.7, 83.0, -71.7),
    "D": (-48.7, 34.8, -36.0, 83.2, -71.7),
    "Q": (-44.8, 37.5, -31.4, 82.5, -62.5),
}
POSES = {name: shifted(p) for name, p in POSES.items()}
J = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def above(p):
    """Over the target, high: shoulder back, forearm level, wrist down."""
    d.glide(**{**dict(zip(J, p)), "shoulder_lift": -10.0, "elbow_flex": 0.0, "gripper": 100})


for mode in ("plain", "reached"):
    for name in ("B", "C", "D", "Q"):
        p = POSES[name]
        if name == "D":
            # D: "up 4 cm" from C, approached from below as the real arm was.
            d.glide(**dict(zip(J, POSES["C"])))
        else:
            above(p)
        target = dict(zip(J, p))
        if mode == "plain":
            d.glide(seconds=4, **target)
            d.robot.settle(timeout=3)
        else:
            d.reach(**target)
        d.report(f"{name}-{mode}")
        d.shoot(f"{name}-{mode}")
    above(POSES["Q"])
# The first motion test's pose: the arm folded down onto the table in front of the base.
d.glide(seconds=3, shoulder_pan=0, shoulder_lift=-30.7 + 5.36, elbow_flex=60, wrist_flex=0, wrist_roll=0)
d.glide(seconds=3, elbow_flex=93.4 - 3.52)
d.robot.settle(timeout=3)
d.report("early-plain")
