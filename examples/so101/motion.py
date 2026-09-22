"""Grasp geometry and Cartesian paths for the SO101 gripper."""

import math

import numpy as np

from kinematics import TCP

OPEN = 0.4
CLOSED = -0.174
GRASP_Z = 0.015
# Cube-top centre in gripper coordinates, midway between the open pads: the pad
# faces bracket a 22mm cube from 0.0031 to 0.0228, so the closing jaw slides the
# cube against the fixed pad from either side.
GRASP_TOP = np.array([0.0129, 0.0, -0.086])
# Tool speed the drives follow without trailing, and the fastest any joint is
# asked to turn: past the servo's rated 1.5rad/s there is no torque left.
TOOL_SPEED = 0.05
JOINT_RATE = 0.6


def grasp_tool(arm, xy, q):
    """Tool point that puts a cube seen at xy between the open pads."""
    return np.asarray(xy) - (GRASP_TOP[0] - TCP[0]) * arm.fk(q)[:2, 0]


def pad_bearing(arm, q):
    """World bearing the pad faces look along."""
    axis = arm.fk(q)[:3, 0]
    return math.atan2(float(axis[1]), float(axis[0]))


def face_gap(bearing, yaw):
    """Signed angle from the pads to the nearest cube face, within +-45deg."""
    return (yaw - bearing + math.pi / 4) % (math.pi / 2) - math.pi / 4


def grasp_pose(arm, xy, roll):
    """Joints with the open pads bracketing a cube top seen at xy."""
    q, _ = arm.ik((*xy, GRASP_Z), roll=roll)
    for _ in range(3):
        q, _ = arm.ik((*grasp_tool(arm, xy, q), GRASP_Z), q, roll=roll)
    return q


def square_roll(arm, xy, yaw, seed=0.0):
    """Wrist roll that squares the pads to a cube face at the grasp itself.

    The bearing the pads look along is set by the whole arm, not the wrist
    alone, so it is solved where the grasp happens. Jaws that close between a
    face and a diagonal tip the cube as they take its weight.
    """
    roll = seed
    for _ in range(6):
        gap = face_gap(pad_bearing(arm, grasp_pose(arm, xy, roll)), yaw)
        if abs(gap) < math.radians(0.25):
            break
        roll += gap
    return roll


# Corners of everything that reaches below the bowl rim at a grasp, in the
# gripper frame around the tool point: the fixed pad and its block on one side
# and the open jaw's pad on the other.
LOW = np.array(
    [[x, y] for x in (-0.031, 0.045) for y in (-0.012, 0.012)]
    + [[0.007, -0.012], [0.007, 0.012]]
) + TCP[:2]
# Outer bowl wall, and a margin for arm and vision errors.
BOWL_WALL = 0.076
GUARD = 0.006


def low_corners(arm, q):
    """World xy of the low gripper corners at joints q."""
    frame = arm.fk(q)
    return LOW @ frame[:2, :2].T + frame[:2, 3]


def clearance(arm, q, bowl, cubes):
    """Least gap from the low corners to the bowl wall and to the other cubes."""
    corners = low_corners(arm, q)
    gaps = [np.min(np.linalg.norm(corners - bowl, axis=1)) - BOWL_WALL]
    for cube in cubes:
        gaps.append(np.min(np.linalg.norm(corners - cube, axis=1)) - 0.016)
    return float(min(gaps))


def choose_roll(arm, xy, yaw, bowl, cubes):
    """Square-face roll that keeps the low gripper corners clear of the bowl and cubes."""
    squared = square_roll(arm, xy, yaw)
    best = None
    for offset in (0.0, math.pi / 2, -math.pi / 2, math.pi, -math.pi):
        roll = square_roll(arm, xy, yaw, squared + offset)
        if not arm.limits[4, 0] + 0.05 < roll < arm.limits[4, 1] - 0.05:
            continue
        q = grasp_pose(arm, xy, roll)
        if np.linalg.norm((arm.fk(q) @ TCP)[:2] - grasp_tool(arm, xy, q)) > 0.003:
            continue
        gap = clearance(arm, q, bowl, cubes)
        if best is None or gap > best[0] + 1e-6:
            best = (gap, roll)
    if best is None:
        raise ValueError(f"No square grasp roll within the wrist limits at {xy}")
    if best[0] < GUARD:
        raise ValueError(f"Grasp at {xy} comes within {best[0] * 1000:.0f}mm of the bowl")
    return best[1]


def duration(start, end, seconds):
    """Stretch a segment until no joint outruns JOINT_RATE at its eased peak."""
    travel = float(np.max(np.abs(np.asarray(end) - np.asarray(start))))
    return max(seconds, 1.5 * travel / JOINT_RATE)


def path(arm, start, destination, grip, roll, seconds=1.0):
    """Joint waypoints along a straight tool line, and how long to take.

    Interpolating joints between two distant poses sweeps the tool through an
    arc; a centimetre of waypoints keeps it on the line the caller checked.
    """
    destination = np.asarray(destination, dtype=float)
    origin = (arm.fk(start) @ TCP)[:3]
    span = float(np.linalg.norm(destination - origin))
    count = max(1, int(np.ceil(span / 0.01)))
    points = [np.array(start, dtype=float)]
    for fraction in np.linspace(0.0, 1.0, count + 1)[1:]:
        point = origin + fraction * (destination - origin)
        turn = start[4] + fraction * (roll - start[4])
        q, _ = arm.ik(point, points[-1], roll=turn)
        error = float(np.linalg.norm((arm.fk(q) @ TCP)[:3] - point))
        if error > 0.008:
            raise ValueError(
                f"Unreachable waypoint {point.round(3)}: {error * 1000:.1f}mm off"
            )
        points.append(np.r_[q, start[5] + fraction * (grip - start[5])])
    seconds = max(seconds, span / TOOL_SPEED)
    for a, b in zip(points, points[1:]):
        seconds = max(seconds, count * duration(a, b, 0.0))
    return points, seconds
