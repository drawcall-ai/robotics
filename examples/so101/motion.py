"""Tool paths and clearances for the SO101 gripper."""

import numpy as np

from kinematics import TCP

# Jaw angles: open wide enough around a piece's head, and fully closed.
OPEN = 0.25
CLOSED = -0.174
# The pad bottoms hang this far below the tool point.
PAD_DEPTH = 0.015
# Tool speed the drives follow without trailing, and the fastest any joint is asked to
# turn: past the servo's rated 1.5 rad/s there is no torque left.
TOOL_SPEED = 0.05
JOINT_RATE = 0.6


def between_pads(arm, jaw):
    """Where an object's axis sits between the pads at a jaw angle, along the gripper's x
    axis: midway between the pad faces."""
    return float(np.mean(arm.pads(jaw, TCP[2] - PAD_DEPTH, TCP[2])))


def grasp_tool(arm, xy, q, axis):
    """Tool point that puts an object's axis, `axis` along the gripper's x, at xy."""
    return np.asarray(xy) - (axis - TCP[0]) * arm.fk(q)[:2, 0]


def clearance(arm, q, obstacles, floor):
    """Least height of the gripper over the floor, a height or the height under each xy,
    and over each obstacle (xy, radius, top)."""
    points = np.concatenate([t.reshape(-1, 3) for t in arm.surfaces(q)])
    gap = (points[:, 2] - (floor(points[:, :2]) if callable(floor) else floor)).min()
    for xy, radius, top in obstacles:
        near = np.linalg.norm(points[:, :2] - xy, axis=1) < radius
        if near.any():
            gap = min(gap, points[near, 2].min() - top)
    return float(gap)


def duration(start, end, seconds):
    """Stretch a segment until no joint outruns JOINT_RATE at its eased peak."""
    travel = float(np.max(np.abs(np.asarray(end) - np.asarray(start))))
    return max(seconds, 1.5 * travel / JOINT_RATE)


def path(arm, start, destination, grip, roll, seconds=1.0):
    """Joint waypoints along a straight tool line, and how long to take.

    Interpolating joints between two distant poses sweeps the tool through an arc; a
    centimetre of waypoints keeps it on the line.
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
            raise ValueError(f"Unreachable waypoint {point.round(3)}: {error * 1000:.1f}mm off")
        points.append(np.r_[q, start[5] + fraction * (grip - start[5])])
    seconds = max(seconds, span / TOOL_SPEED)
    for a, b in zip(points, points[1:]):
        seconds = max(seconds, count * duration(a, b, 0.0))
    return points, seconds
