"""Builder check: drive to the poses of the real photos (readings from REAL_OBSERVATIONS) and
save the sim wrist camera's view as evidence/sim-<name>.jpg."""
import sys
sys.path.insert(0, "evidence")
from drive import Driver, shifted

d = Driver()
d.robot.p_gain(shoulder_pan=32, shoulder_lift=32, elbow_flex=32, wrist_flex=32, wrist_roll=32)
poses = {
    "01-wrist40": dict(shoulder_pan=-0.3, shoulder_lift=0.5, elbow_flex=3.4, wrist_flex=39.9, wrist_roll=0.2, gripper=8),
    "03-pan-30": dict(shoulder_pan=-29.8, shoulder_lift=0.5, elbow_flex=3.4, wrist_flex=39.9, wrist_roll=0.2, gripper=8),
    "26-view": dict(shoulder_pan=-46.6, shoulder_lift=29.0, elbow_flex=-34.4, wrist_flex=83.0, wrist_roll=-0.2, gripper=100),
    "30-sA": dict(shoulder_pan=-48.7, shoulder_lift=19.7, elbow_flex=-35.9, wrist_flex=83.3, wrist_roll=-30.0, gripper=100),
    "30-sB": dict(shoulder_pan=-48.7, shoulder_lift=19.7, elbow_flex=-35.9, wrist_flex=83.3, wrist_roll=9.9, gripper=100),
    "20-pre": dict(shoulder_pan=-44.5, shoulder_lift=31.5, elbow_flex=-38.6, wrist_flex=82.5, wrist_roll=-61.2, gripper=100),
}
for name, p in poses.items():
    p = shifted(p)
    if p["shoulder_lift"] > 10:
        d.glide(**{**p, "shoulder_lift": 0.0})
    d.reach(**p)
    d.report(name)
    d.shoot(name)
