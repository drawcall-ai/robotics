"""SO101 transforms from the published robot description, and bounded numerical inverse kinematics."""

import xml.etree.ElementTree as ElementTree

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

# The tool point in the gripper frame, between the fingertips.
TCP = np.array([0.0031, 0, -0.090, 1.0])


def transform(xyz, rotation):
    t = np.eye(4)
    t[:3, :3] = rotation.as_matrix()
    t[:3, 3] = xyz
    return t


def origin(joint):
    element = joint.find("origin")
    xyz = [float(v) for v in element.get("xyz").split()]
    rpy = [float(v) for v in element.get("rpy").split()]
    return transform(xyz, Rotation.from_euler("xyz", rpy))


class Arm:
    """The arm chain from base_link to gripper_link, read from URDF.

    Poses are in the CAD link frames the robot is authored in: the base frame
    has the base_link origin and the world's axes, and the gripper frame is the
    CAD gripper body's. The export turns every link frame by the same rotation
    against its CAD frame, and the footprint joint, level with the world, undoes
    it for the base.
    """

    def __init__(self, urdf):
        robot = ElementTree.fromstring(urdf)
        joints = {joint.find("child").get("link"): joint for joint in robot.findall("joint")}
        chain = []
        link = "gripper_link"
        while link != "base_link":
            chain.insert(0, joints[link])
            link = chain[0].find("parent").get("link")
        self.turn = np.eye(4)
        self.turn[:3, :3] = origin(joints["base_link"])[:3, :3]
        self.origins = [origin(joint) for joint in chain]
        self.axes = [
            np.array([float(v) for v in joint.find("axis").get("xyz").split()])
            for joint in chain
        ]
        self.names = [joint.get("name") for joint in chain] + ["gripper"]
        self.limits = np.array(
            [
                [float(joint.find("limit").get(bound)) for bound in ("lower", "upper")]
                for joint in chain
            ]
        )
        # The camera's optical frame (+Z forward, +X right, +Y down) in the gripper frame.
        self.camera = self.turn @ origin(joints["wrist_camera_optical"])

    def fk(self, q):
        t = self.turn.copy()
        for joint, axis, angle in zip(self.origins, self.axes, q):
            t = t @ joint @ transform((0, 0, 0), Rotation.from_rotvec(axis * float(angle)))
        return t @ self.turn.T

    def ik(self, position, seed=None, roll=0):
        """Joints placing the tool point with the fingers down and the wrist at `roll`.

        The roll is exact: it barely moves the tool point, and squaring the jaws to
        a cube face depends on it. The four other joints trade the downward finger
        direction for position where both cannot be met.
        """
        target = np.array(position)
        lower, upper = self.limits[:4, 0], self.limits[:4, 1]
        best = None
        seeds = [
            np.array(seed[:4]) if seed is not None else np.array([0, -0.4, 0.7, 0.9]),
            np.array([0, -0.8, 1.2, 0.8]),
        ]
        for q in seeds:

            def residual(a):
                t = self.fk(np.r_[a, roll])
                return np.r_[
                    (t @ TCP)[:3] - target,
                    (0.10 if target[2] < 0.04 else 0.025)
                    * (t[:3, 2] - np.array([0, 0, 1])),
                ]

            fit = least_squares(
                residual,
                np.clip(q, lower + 1e-5, upper - 1e-5),
                bounds=(lower, upper),
                max_nfev=100,
                ftol=1e-8,
                gtol=1e-8,
                xtol=1e-8,
            )
            score = np.linalg.norm(residual(fit.x))
            if best is None or score < best[0]:
                best = (score, np.r_[fit.x, roll])
        return best[1], best[0]
