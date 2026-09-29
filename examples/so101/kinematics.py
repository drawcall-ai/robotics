"""SO101 transforms from the published robot description, and bounded numerical inverse kinematics."""

import base64
import math
import xml.etree.ElementTree as ElementTree

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

# The tool point in the gripper frame, between the fingertips.
TCP = np.array([0.0031, 0, -0.090, 1.0])


def transform(xyz, rotation):
    t = np.eye(4)
    t[:3, :3] = rotation.as_matrix()
    t[:3, 3] = xyz
    return t


def triangles(visual):
    """Triangles, (n, 3, 3), of a visual's embedded binary STL, in its link frame."""
    url = visual.find("geometry/mesh").get("filename")
    data = base64.b64decode(url.split(",", 1)[1])
    count = int(np.frombuffer(data, np.uint32, 1, 80)[0])
    records = np.frombuffer(data, np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count, 84)
    points = records["v"].astype(float)
    element = visual.find("origin")
    if element is not None:
        pose = origin(visual)
        points = points @ pose[:3, :3].T + pose[:3, 3]
    return points


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
        # The surface the robot stands on: base_footprint, under the lowest collider.
        self.floor = -float(origin(joints["base_link"])[2, 3])
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
        # The jaws' visual surfaces, for masking them out of the wrist camera's images.
        jaw = joints["moving_jaw_so101_v1_link"]
        self.jaw = (
            self.turn @ origin(jaw),
            np.array([float(v) for v in jaw.find("axis").get("xyz").split()]),
        )
        links = {link.get("name"): link for link in robot.findall("link")}
        self.meshes = {
            name: np.concatenate([triangles(v) for v in links[name].findall("visual")])
            for name in ("gripper_link", "moving_jaw_so101_v1_link")
        }
        # The arm's links up to the wrist, the base first, as convex hulls: enough to mask the
        # robot out of images, and fast. The gripper and jaw keep their meshes: their hulls
        # would cover the gap between the jaws.
        self.body = []
        for name in ["base_link"] + [joint.find("child").get("link") for joint in chain[:-1]]:
            points = np.concatenate([triangles(v) for v in links[name].findall("visual")]).reshape(-1, 3)
            hull = ConvexHull(points)
            self.body.append(points[hull.simplices])

    def pads(self, jaw, low, high):
        """The fixed and the moving pad's gripping faces at a jaw angle, as positions along
        the gripper's x axis, between heights low and high in the tool's frame."""
        q = np.zeros(6)
        q[5] = jaw
        inverse = np.linalg.inv(self.fk(q))
        fixed, moving = (
            np.c_[t.reshape(-1, 3), np.ones(t.size // 3)] @ inverse.T for t in self.surfaces(q)
        )
        # The fingertips, where they face each other.
        band = lambda p: p[(p[:, 2] > low) & (p[:, 2] < high) & (np.abs(p[:, 1]) < 0.008), 0]
        return band(fixed).max(), band(moving).min()

    def fk(self, q):
        t = self.turn.copy()
        for joint, axis, angle in zip(self.origins, self.axes, q):
            t = t @ joint @ transform((0, 0, 0), Rotation.from_rotvec(axis * float(angle)))
        return t @ self.turn.T

    def body_surfaces(self, q):
        """The whole robot's visual triangles at joints q, in the base frame."""
        frames = [self.turn]
        t = self.turn.copy()
        for joint, axis, angle in zip(self.origins, self.axes, q):
            t = t @ joint @ transform((0, 0, 0), Rotation.from_rotvec(axis * float(angle)))
            frames.append(t.copy())
        found = [mesh @ frame[:3, :3].T + frame[:3, 3] for mesh, frame in zip(self.body, frames)]
        return found + self.surfaces(q)

    def surfaces(self, q):
        """The gripper's and moving jaw's visual triangles at joints q, in the base frame."""
        gripper = self.fk(q) @ self.turn
        joint, axis = self.jaw
        jaw = self.fk(q) @ joint @ transform((0, 0, 0), Rotation.from_rotvec(axis * float(q[5])))
        return [
            self.meshes["gripper_link"] @ gripper[:3, :3].T + gripper[:3, 3],
            self.meshes["moving_jaw_so101_v1_link"] @ jaw[:3, :3].T + jaw[:3, 3],
        ]

    def aim(self, position, direction, seed=None):
        """Joints putting the camera's optical centre at `position`, looking along `direction`.

        Returns the five arm joints, the position error in metres and the cosine between
        the view and `direction`.
        """
        target = np.asarray(position, float)
        direction = np.asarray(direction, float) / np.linalg.norm(direction)
        lower, upper = self.limits[:, 0], self.limits[:, 1]
        seeds = [np.array(seed[:5])] if seed is not None else []
        pan = math.atan2(target[1], target[0])
        seeds += [
            np.array([pan, -1.4, 0.6, 1.6, -1.5]),
            np.array([pan, -0.8, 0.6, 1.5, 1.6]),
            np.array([pan, -1.5, 1.0, 1.5, -0.9]),
            np.array([pan, -0.5, 0.3, 1.5, -1.0]),
        ]
        best = None
        for q in seeds:

            def residual(a):
                t = self.fk(a) @ self.camera
                return np.r_[t[:3, 3] - target, 0.1 * (t[:3, 2] - direction)]

            fit = least_squares(residual, np.clip(q, lower + 1e-5, upper - 1e-5), bounds=(lower, upper))
            if best is None or fit.cost < best.cost:
                best = fit
        t = self.fk(best.x) @ self.camera
        return best.x, float(np.linalg.norm(t[:3, 3] - target)), float(t[:3, 2] @ direction)

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


    def pose(self, point, position, z_axis, x_axis, seed=None):
        """Joints putting `point`, in the gripper frame, at `position` with the gripper's z
        and x axes along the given directions, as nearly as the five joints allow.

        Returns the five arm joints, the position error in metres and the larger angle, in
        radians, between one of the two axes and its direction.
        """
        target = np.asarray(position, float)
        point = np.r_[np.asarray(point, float), 1.0]
        z_axis, x_axis = (np.asarray(v, float) / np.linalg.norm(v) for v in (z_axis, x_axis))
        lower, upper = self.limits[:5, 0], self.limits[:5, 1]
        pan = math.atan2(target[1], target[0])
        seeds = [np.array(seed[:5])] if seed is not None else []
        seeds += [np.array([pan, -0.4, 0.7, 0.9, r]) for r in (-1.6, 0.0, 1.6)]
        best = None
        for q in seeds:

            def residual(a):
                t = self.fk(a)
                return np.r_[(t @ point)[:3] - target, 0.05 * (t[:3, 2] - z_axis), 0.05 * (t[:3, 0] - x_axis)]

            fit = least_squares(residual, np.clip(q, lower + 1e-5, upper - 1e-5), bounds=(lower, upper))
            if best is None or fit.cost < best.cost:
                best = fit
        t = self.fk(best.x)
        error = float(np.linalg.norm((t @ point)[:3] - target))
        angle = float(max(math.acos(np.clip(t[:3, 2] @ z_axis, -1, 1)), math.acos(np.clip(t[:3, 0] @ x_axis, -1, 1))))
        return best.x, error, angle
