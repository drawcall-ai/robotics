"""Nominal finger geometry and collision checks against the visually located bowl."""

import math
import itertools

import cv2

import numpy as np

from kinematics import fk, transform

# Hardware pad boxes, subdivided along their lengths for tighter clearance tests.
FIXED = np.array([-0.0119, 0.0, -0.080])
MOVING = np.array([-0.0083, -0.0595, 0.0189])
JAW = transform((0.0202, 0.0188, -0.0234), (math.pi / 2, 0.0, 0.0))


def pad_boxes(q):
    gripper = fk(q)
    jaw = gripper @ JAW @ transform((0.0, 0.0, 0.0), (0.0, 0.0, float(q[5])))
    yield gripper, np.array([-0.018, 0.0, -0.05]), np.array([0.020, 0.020, 0.022])
    for i in range(5):
        fixed_offset = (i - 2) * (0.0488 / 5)
        moving_offset = (i - 2) * (0.045 / 5)
        yield (
            gripper,
            FIXED + (0.0, 0.0, fixed_offset),
            np.array([0.008, 0.012, 0.0488 / 5]),
        )
        yield (
            jaw,
            MOVING + (0.0, moving_offset, 0.0),
            np.array([0.008, 0.045 / 5, 0.012]),
        )


CORNERS = np.array(list(itertools.product((-1.0, 1.0), repeat=3)))
EDGES = [
    (i, j)
    for i in range(8)
    for j in range(i + 1, 8)
    if np.count_nonzero(CORNERS[i] != CORNERS[j]) == 1
]


def clears_bowl(q, bowl):
    """Exact projected pad polygon below the guarded rim height.

    An AABB quickly rejects distant boxes. Intersecting boxes are clipped at
    rim height before testing the circular bowl, avoiding false corner alarms.
    """
    if bowl is None:
        return True
    for frame, center, size in pad_boxes(q):
        world = frame[:3, :3] @ center + frame[:3, 3]
        half = np.abs(frame[:3, :3]) @ (size / 2)
        nearest = np.maximum(np.abs(world[:2] - bowl) - half[:2], 0.0)
        if np.linalg.norm(nearest) >= 0.085 or world[2] - half[2] >= 0.055:
            continue
        vertices = (center + CORNERS * (size / 2)) @ frame[:3, :3].T + frame[:3, 3]
        clipped = list(vertices[vertices[:, 2] <= 0.055])
        for i, j in EDGES:
            first, second = vertices[i], vertices[j]
            if (first[2] - 0.055) * (second[2] - 0.055) < 0:
                clipped.append(
                    first
                    + (0.055 - first[2]) / (second[2] - first[2]) * (second - first)
                )
        polygon = cv2.convexHull(np.asarray(clipped, dtype=np.float32)[:, :2].copy())
        if cv2.pointPolygonTest(polygon, tuple(map(float, bowl)), True) > -0.085:
            return False
    return True


def edge_clears_bowl(start, end, bowl):
    return all(
        clears_bowl(start + t * (end - start), bowl) for t in np.linspace(0.0, 1.0, 17)
    )


class PlanningError(ValueError):
    pass


def plan_move(start, destination, grip, bowl, duration, roll=0.0):
    """Cartesian segments whose interpolated joint edges clear the bowl."""
    from kinematics import TCP, ik

    destination = np.asarray(destination)
    origin = (fk(start) @ TCP)[:3]
    count = max(1, int(np.ceil(np.linalg.norm(destination - origin) / 0.025)))
    if bowl is None:
        count = 1
    result = []
    previous = start.copy()
    for fraction in np.linspace(0.0, 1.0, count + 1)[1:]:
        point = origin + fraction * (destination - origin)
        q, _ = ik(point, previous, roll=roll)
        command = np.r_[q, start[5] + fraction * (grip - start[5])]
        error = np.linalg.norm((fk(q) @ TCP)[:3] - point)
        if error > 0.008:
            raise PlanningError(
                f"Unreachable waypoint {point.round(3)}: {error * 1000:.1f}mm error"
            )
        if not edge_clears_bowl(previous, command, bowl):
            raise PlanningError(f"Finger/bowl clearance rejected at {point.round(3)}")
        result.append((command, max(0.16, duration / count)))
        previous = command
    return result


def lowest_finger(q):
    return min(
        (frame[:3, :3] @ center + frame[:3, 3])[2]
        - (np.abs(frame[:3, :3]) @ (size / 2))[2]
        for frame, center, size in pad_boxes(q)
    )


def closure_floor_clearance(q):
    """Full closing sweep, evaluated from measured arm joint positions."""
    return (
        min(lowest_finger(np.r_[q[:5], grip]) for grip in np.linspace(-0.174, 0.4, 17))
        + 0.003
    )


def choose_grasp_roll(start, xy, preferred, bowl):
    """Square faces admit quarter-turn alternatives; choose a clear jaw opening."""
    from kinematics import LIMITS

    errors = []
    for offset in (0.0, math.pi / 2, -math.pi / 2, math.pi, -math.pi):
        roll = preferred + offset
        if not LIMITS[4, 0] < roll < LIMITS[4, 1]:
            continue
        try:
            aligned = plan_move(start, (*xy, 0.08), 0.4, bowl, 1.0, roll)[-1][0]
            target = np.r_[np.asarray(xy) - 0.002 * fk(aligned)[:2, 0], 0.015]
            low = plan_move(aligned, target, 0.4, bowl, 2.0, roll)[-1][0]
            closed = plan_move(low, target, -0.174, bowl, 1.2, roll)[-1][0]
            plan_move(closed, (*target[:2], 0.085), -0.174, bowl, 2.0, roll)
            return roll
        except PlanningError as error:
            errors.append(str(error))
    raise PlanningError("No clear square-face grasp: " + "; ".join(errors))


def camera_floor_clearance(q, top, normal):
    """Jaw sweep above the table inferred from a horizontal 22mm cube."""
    gripper = fk(q)
    floor = gripper[:3, 3] + gripper[:3, :3] @ (top - 0.022 * normal)
    up = gripper[:3, :3] @ normal
    clearance = float("inf")
    for angle in np.linspace(-0.174, 0.4, 17):
        pose = np.array(q, copy=True)
        pose[5] = angle
        for frame, center, size in pad_boxes(pose):
            corners = (center + CORNERS * (size / 2)) @ frame[:3, :3].T + frame[:3, 3]
            clearance = min(clearance, float(np.min((corners - floor) @ up)))
    return clearance
