"""Builder check of REAL_OBSERVATIONS §2: sag, dead band, sudden drop, tracking lag, present-position
interpolation, approach-direction hysteresis and overheating, in the extended pose of §2, at LeRobot's P=16."""
import json
import sys
sys.path.insert(0, "evidence")
from drive import Driver

d = Driver()
r = d.robot
EXT = dict(shoulder_pan=60.0, shoulder_lift=22.0, elbow_flex=-31.0, wrist_flex=78.0, wrist_roll=0.0, gripper=50)


def out(name, **kw):
    d.log("RESULT " + json.dumps({"name": name, **kw}))


def sample(seconds, every=0.1):
    rows = []
    t0 = r.time
    while r.time - t0 < seconds:
        rows.append((round(r.time - t0, 2), round(r.joints["shoulder_lift"], 2), int(r.loads["shoulder_lift"])))
        r.wait(every)
    return rows


# 1. Sag: glide to the extended pose, the shoulder coming up from below as the real arm's holding
# load (320-336, the top of its 104...344 hysteresis) shows it did; settle.
d.glide(**{**EXT, "shoulder_lift": 0.0, "elbow_flex": 0.0})
d.glide(seconds=3, **{**EXT, "shoulder_lift": EXT["shoulder_lift"] + 8})
d.glide(seconds=2, **EXT)
p = r.settle(timeout=4)
r.wait(2)
p = dict(r.joints)
out("sag", goal_minus_present={j: round(EXT[j] - p[j], 1) for j in ("shoulder_lift", "elbow_flex", "wrist_flex")},
    loads={j: int(r.loads[j]) for j in ("shoulder_lift", "elbow_flex", "wrist_flex")})
# 2. Dead band: 3 deg further down (with gravity) from the holding pose.
before = dict(r.joints)
d.goal["shoulder_lift"] += 3
r.goal(**d.goal)
rows = sample(2)
out("deadband_3deg_down", lift_before=round(before["shoulder_lift"], 2), lift_after=round(r.joints["shoulder_lift"], 2),
    load_before=rows[0][2], load_after=rows[-1][2])
# 3. Another 3 deg: the band gives way at once.
d.goal["shoulder_lift"] += 3
r.goal(**d.goal)
out("release_6deg_down", series=sample(1.5, 0.05))
r.wait(2)
# 4. Slow descent, 4 deg/s for 3 s, in free air: lag of present behind goal.
r.settle(timeout=3)
start = d.goal["shoulder_lift"]
lags = []
for i in range(1, 61):
    d.goal["shoulder_lift"] = start - 4 * 0.05 * i  # upwards, against gravity
    r.goal(**d.goal)
    r.wait(0.05)
    lags.append(round(r.joints["shoulder_lift"] - d.goal["shoulder_lift"], 2))
out("slow_move_up_4dps", lag_present_minus_goal=lags[::6])
for i in range(1, 61):
    d.goal["shoulder_lift"] = start - 12 + 4 * 0.05 * i  # back down, with gravity
    r.goal(**d.goal)
    r.wait(0.05)
    lags.append(round(r.joints["shoulder_lift"] - d.goal["shoulder_lift"], 2))
out("slow_move_down_4dps", lag_present_minus_goal=lags[60::6])
# 5. A move interpolated from the present position instead of the current goal.
p = r.settle(timeout=3)
r.wait(1)
load0 = int(r.loads["shoulder_lift"])
d.goal["shoulder_lift"] = p["shoulder_lift"]
r.goal(**d.goal)
out("goal_to_present", load_before=load0, series=sample(1.5, 0.05))
# 6. Approach direction: settle at the same goal coming from above and from below.
settled = {}
for side, offset in (("from_above", -8.0), ("from_below", 8.0)):
    d.glide(seconds=2, **{**EXT, "shoulder_lift": EXT["shoulder_lift"] + offset})
    r.settle(timeout=3)
    d.glide(seconds=2, **EXT)
    r.settle(timeout=3)
    r.wait(2)
    settled[side] = dict(lift=round(r.joints["shoulder_lift"], 2), load=int(r.loads["shoulder_lift"]))
out("approach_direction", **settled)
# 7. The slow moves again at the factory P=32.
r.p_gain(shoulder_lift=32, elbow_flex=32, wrist_flex=32)
d.glide(seconds=3, **EXT)
r.settle(timeout=3)
r.wait(2)
start = d.goal["shoulder_lift"]
lags = []
for i in range(1, 61):
    d.goal["shoulder_lift"] = start - 4 * 0.05 * i
    r.goal(**d.goal)
    r.wait(0.05)
    lags.append(round(r.joints["shoulder_lift"] - d.goal["shoulder_lift"], 2))
for i in range(1, 61):
    d.goal["shoulder_lift"] = start - 12 + 4 * 0.05 * i
    r.goal(**d.goal)
    r.wait(0.05)
    lags.append(round(r.joints["shoulder_lift"] - d.goal["shoulder_lift"], 2))
out("slow_moves_p32", up=lags[:60:6], down=lags[60::6], load=int(r.loads["shoulder_lift"]))
