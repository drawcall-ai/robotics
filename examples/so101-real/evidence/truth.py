"""Builder-only tools for checking fidelity: true joint angles (from the simulator's
/robot/joint_states) and forward kinematics of the SO-101 model. Never part of a policy."""
import math
import threading

import numpy as np
from sensor_msgs.msg import JointState

CHAIN = [
    ("shoulder_pan", [0.0388353, -8.97657e-9, 0.0624], [3.14159, 4.18253e-17, -3.14159]),
    ("shoulder_lift", [-0.0303992, -0.0182778, -0.0542], [-1.5708, -1.5708, 0.0]),
    ("elbow_flex", [-0.11257, -0.028, 1.73763e-16], [-3.63608e-16, 8.74301e-16, 1.5708]),
    ("wrist_flex", [-0.1349, 0.0052, 3.62355e-17], [4.02456e-15, 8.67362e-16, -1.5708]),
    ("wrist_roll", [5.55112e-17, -0.0611, 0.0181], [1.5708, 0.0486795, 3.14159]),
]
# The fixed (white) jaw's lowest point and the camera board, gripper_link frame.
FIXED_TIP = np.array([-0.012, 0.0, -0.1044, 1.0])
TABLE = -0.003  # table plane in base_link z


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = map(lambda f: f, (math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)))
    return np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])


def gripper(q):
    """gripper_link pose in base_link for joint angles q (radians by name)."""
    m = np.eye(4)
    for name, xyz, r in CHAIN:
        t = np.eye(4)
        t[:3, :3] = rpy(*r)
        t[:3, 3] = xyz
        z = np.eye(4)
        c, s = math.cos(q.get(name, 0.0)), math.sin(q.get(name, 0.0))
        z[:2, :2] = [[c, -s], [s, c]]
        m = m @ t @ z
    return m


def tip_height(q):
    """Height of the fixed jaw's tip above the table, metres."""
    return float((gripper(q) @ FIXED_TIP)[2] - TABLE)


class Truth:
    """Subscribes the simulator's true joint angles on a node."""

    def __init__(self, node):
        self.q = {}
        self.lock = threading.Lock()
        node.create_subscription(JointState, "/robot/joint_states", self.update, 10)

    def update(self, msg):
        with self.lock:
            self.q = dict(zip(msg.name, msg.position))
