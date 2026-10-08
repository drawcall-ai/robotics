"""Builder check of the withdrawal rule from a run's bag: the distance between the queen (a capsule
as wide as its base) and the gripper's, jaws' and camera's box colliders, from the recorded true
joints and queen pose, sampled at box corners and centres (the old rule) and exactly.
Usage: clearance.py <heartbeats.jsonl> <joint_states.jsonl> [scale]"""
import json
import math
import sys

import numpy as np

sys.path.insert(0, "evidence")
from truth import gripper  # noqa: E402

CAMERA = np.array([[0.1476, -0.7005, -0.6983, 0.0999], [-0.9889, -0.0942, -0.1144, 0.0165],
                   [0.0144, 0.7074, -0.7066, -0.03], [0, 0, 0, 1]])
ROLL = math.radians(100)


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])


def boxes(q):
    """(pose in base frame, half size) of each hand collider."""
    g = gripper(q)
    out = [(g @ np.r_[np.c_[np.eye(3), c], [[0, 0, 0, 1]]], np.array(s) / 2) for s, c in [
        ([0.04, 0.035, 0.035], [0, 0, -0.022]), ([0.02, 0.02, 0.022], [-0.018, 0, -0.05]),
        ([0.008, 0.012, 0.0488], [-0.0119, 0, -0.08])]]
    jaw = np.eye(4)
    jaw[:3, 3] = [0.0202, 0.0188, -0.0234]
    jaw = g @ jaw @ rx(1.5708) @ rz(q["gripper"])
    t = np.eye(4)
    t[:3, 3] = [-0.0083, -0.0595, 0.0189]
    out.append((jaw @ t, np.array([0.004, 0.0225, 0.006])))
    cam = g @ rz(-ROLL) @ CAMERA @ np.diag([1, -1, -1, 1])
    t = np.eye(4)
    t[:3, 3] = [0, 0, 0.007]
    out.append((cam @ t, np.array([0.0125, 0.012, 0.006])))
    return out


def distances(q, base, axis, length, radius):
    samples = [base + axis * length * k / 40 for k in range(41)]
    corners, exact = [], []
    for pose, half in boxes(q):
        inv = np.linalg.inv(pose)
        for a in (-1, 0, 1):
            for b in (-1, 0, 1):
                for c in (-1, 0, 1):
                    if (a == 0) + (b == 0) + (c == 0) in (0, 3):
                        p = (pose @ np.r_[half * [a, b, c], 1])[:3]
                        t = np.clip((p - base) @ axis, 0, length)
                        corners.append(np.linalg.norm(p - (base + t * axis)) - radius)
        for s in samples:
            local = (inv @ np.r_[s, 1])[:3]
            exact.append(np.linalg.norm(local - np.clip(local, -half, half)) - radius)
    return min(corners), min(exact)


if __name__ == "__main__":
    scale = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
    beats = [json.loads(json.loads(line)["msg"]["data"]) for line in open(sys.argv[1])]
    states = [json.loads(line) for line in open(sys.argv[2])]
    for beat in beats:
        state = min(states, key=lambda s: abs(s["t"] - beat["time"]))["msg"]
        q = dict(zip(state["name"], state["position"]))
        queen = beat["queen"]
        base = np.array([queen["x"], queen["y"], queen["z"] - 0.003])
        c, e = distances(q, base, np.array(queen["axis"]), 0.073 * scale, 0.0113 * scale)
        print(f"t {beat['time']:.1f}: corners {c * 100:.2f} cm, exact {e * 100:.2f} cm")
