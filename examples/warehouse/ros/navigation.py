from math import atan2, cos, hypot, isfinite, pi, sin

GOALS = ((4.0, 0.0), (4.0, 3.0), (0.0, 3.0), (0.0, 0.0))
RADIUS = 0.22
TRACK = 0.9


def wrap(angle):
    return atan2(sin(angle), cos(angle))


def wheels(speed, turn):
    return ((speed - turn * TRACK / 2) / RADIUS,
            (speed + turn * TRACK / 2) / RADIUS)


def clearance(ranges, angle_min, angle_increment, range_max):
    front, left, right = [], [], []
    for index, distance in enumerate(ranges):
        if not isfinite(distance) or distance <= 0:
            continue
        angle = angle_min + index * angle_increment
        distance = min(distance, range_max)
        if abs(angle) < pi / 6:
            front.append(distance)
        if angle > pi / 6:
            left.append(distance)
        if angle < -pi / 6:
            right.append(distance)
    return (min(front, default=0.0), min(left, default=0.0),
            min(right, default=0.0))


def command(x, y, yaw, goal, distances, docking=False):
    dx, dy = goal[0] - x, goal[1] - y
    distance = hypot(dx, dy)
    if distance < 0.3:
        return 0.0, 0.0, 'arrived'
    front, left, right = distances
    if front < 0.75:
        return 0.0, 0.7 if left >= right else -0.7, 'avoiding'
    error = wrap(atan2(dy, dx) - yaw)
    turn = max(-1.1, min(1.1, 1.8 * error))
    speed = min(0.75, 0.8 * distance) * max(0.0, cos(error))
    if abs(error) > 0.65:
        speed = 0.0
    speed *= min(1.0, max(0.0, (front - 0.75) / 0.8))
    if docking:
        speed = min(speed, 0.2)
    return speed, turn, 'docking' if docking else 'patrolling'
