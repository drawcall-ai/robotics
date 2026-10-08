"""Builder-only motion helpers for fidelity checks, on top of the solver's harness."""
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot import Robot  # noqa: E402
from truth import Truth, tip_height  # noqa: E402

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
HERE = os.path.dirname(os.path.abspath(__file__))


class Driver:
    def __init__(self):
        self.robot = Robot()
        self.truth = Truth(self.robot)
        self.goal = {j: self.robot.joints[j] for j in ARM + ["gripper"]}
        self.log = self.robot.get_logger().info

    def glide(self, seconds=2.0, steps=40, **target):
        start = dict(self.goal)
        for i in range(1, steps + 1):
            self.goal.update({k: start[k] + (v - start[k]) * i / steps for k, v in target.items()})
            self.robot.goal(**self.goal)
            self.robot.wait(seconds / steps)

    def reach(self, tol=0.3, rounds=10, **target):
        """Glide to readings `target`, then nudge goals until the readings match (as the real
        controller's integral nudges did)."""
        self.glide(**target)
        present = self.robot.settle(timeout=3)
        for _ in range(rounds):
            err = {j: target[j] - present[j] for j in target if j != "gripper"}
            if max(abs(e) for e in err.values()) < tol:
                break
            for j, e in err.items():
                self.goal[j] += e
            self.robot.goal(**self.goal)
            present = self.robot.settle(timeout=3)
        return present

    def report(self, name):
        r = self.robot
        present = r.settle(timeout=2)
        q = dict(self.truth.q)
        h = tip_height(q) * 100
        line = {
            "name": name,
            "readings": {k: round(v, 1) for k, v in present.items()},
            "goals": {k: round(v, 1) for k, v in self.goal.items()},
            "loads": {k: int(v) for k, v in r.loads.items()},
            "true_deg": {k: round(np.degrees(v), 1) for k, v in q.items() if k in ARM},
            "fixed_jaw_tip_above_table_cm": round(h, 2),
        }
        self.log("RESULT " + json.dumps(line))
        return line

    def shoot(self, name):
        rgb = self.robot.image()
        rgb = self.robot.image()
        cv2.imwrite(os.path.join(HERE, f"sim-{name}.jpg"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
