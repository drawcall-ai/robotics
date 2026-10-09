"""Builder diagnostic (Round 4): where the jaws really are relative to the queen at the neck
close, against what the solver's calibrated model assumes, from runs' bags (true joints, queen pose)
or from given readings. Base frame, metres; the board top is at z 0.0198 (seed 0).
Model = reading + (0, -0.09, 1.55, -0.71) deg with the roll at its reading, i.e. the real
calibration the solver uses; truth also has the hidden +100 deg roll zero error."""
import json
import math
import subprocess
import sys

import numpy as np

sys.path.insert(0, "evidence")
from truth import gripper  # noqa: E402

J = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
OFF = [0, -0.09, 1.55, -0.71, 0]
ROLL = 100.0
BOARD = 0.0198
TCP = np.array([-0.0079, -0.0002, -0.0981, 1])  # URDF gripper_frame point
FIXED_TIP = np.array([-0.0119, 0, -0.1044, 1])  # fixed pad, tip centre
GRIPPER_CLOSED, TRAVEL = -0.19, math.radians(130)


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


def moving_tip(g, jaw):
    t = np.eye(4)
    t[:3, 3] = [0.0202, 0.0188, -0.0234]
    return (g @ t @ rx(1.5708) @ rz(jaw) @ np.array([-0.0083, -0.082, 0.0189, 1]))[:3]


def true_angles(readings):
    q = {j: math.radians(readings[j] + o) for j, o in zip(J, OFF)}
    q["wrist_roll"] = math.radians(readings["wrist_roll"] + ROLL)
    return q


def model_tcp(readings):
    return (gripper({j: math.radians(readings[j] + o) for j, o in zip(J, OFF)}) @ TCP)[:3]


def describe(name, q_true, readings, queen=None):
    g = gripper(q_true)
    jaw = q_true.get("gripper", GRIPPER_CLOSED + readings.get("gripper", 50) / 100 * TRAVEL)
    pts = {"tcp": (g @ TCP)[:3], "fixed tip": (g @ FIXED_TIP)[:3], "moving tip": moving_tip(g, jaw), "model tcp": model_tcp(readings)}
    row = {"name": name, "readings": {k: round(v, 1) for k, v in readings.items()}}
    for k, p in pts.items():
        row[k] = {"xy": [round(v * 1000, 1) for v in p[:2]], "above board mm": round((p[2] - BOARD) * 1000, 1)}
        if queen is not None:
            row[k]["from queen axis mm"] = [round(v * 1000, 1) for v in (p[:2] - queen)]
    return row


def bag(run, topic, start=None):
    cmd = ["pixi", "run", "drawcall-robotics", "cat", run, "--topics", topic] + (["--start", str(start)] if start else [])
    out = subprocess.run(cmd, capture_output=True, text=True).stdout
    return [json.loads(line) for line in out.splitlines() if line.startswith("{")]


if __name__ == "__main__":
    out = open("evidence/neck_diag.jsonl", "a")
    for run in sys.argv[1:]:
        goals = bag(run, "/follower/goal")
        # The close: the first gripper goal under 40 after the jaws were opened to 45 or more.
        opened, close = False, None
        for m in goals:
            if "gripper" not in m["msg"]["name"]:
                continue
            g = m["msg"]["position"][m["msg"]["name"].index("gripper")]
            opened = opened or g >= 45
            if opened and g < 40:
                close = m["t"]
                break
        t0 = close - 0.05
        joints = min(bag(run, "/robot/joint_states", int(t0) - 1), key=lambda m: abs(m["t"] - t0))["msg"]
        q = dict(zip(joints["name"], joints["position"]))
        reads = min(bag(run, "/follower/joint_states", int(t0) - 1), key=lambda m: abs(m["t"] - t0))["msg"]
        readings = dict(zip(reads["name"], reads["position"]))
        beat = [json.loads(m["msg"]["data"]) for m in bag(run, "/sim/audit", int(t0) - 2) if "heartbeat" in m["msg"]["data"]][0]
        queen = np.array([beat["queen"]["x"], beat["queen"]["y"]])
        row = describe(f"{run} close at {close:.1f}s", q, readings, queen)
        row["queen tilt deg"] = round(beat["queen"]["tiltDeg"], 1)
        print(json.dumps(row))
        out.write(json.dumps(row) + "\n")
