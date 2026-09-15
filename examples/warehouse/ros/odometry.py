"""Differential-drive dead reckoning from encoder angles, never world poses."""
from math import atan2, cos, isfinite, sin

RADIUS = 0.22
TRACK = 0.9


def wheel_speeds(speed, turn):
    return ((speed - turn * TRACK / 2) / RADIUS,
            (speed + turn * TRACK / 2) / RADIUS)


def wrap(angle):
    return atan2(sin(angle), cos(angle))


class WheelOdometry:
    def __init__(self):
        self.previous = None
        self.x = self.y = self.yaw = 0.0
        self.speed = self.turn = 0.0
        self.distance = 0.0

    def update(self, time, left, right):
        if not all(isfinite(v) for v in (time, left, right)):
            raise ValueError('Nonfinite encoder measurement')
        previous = self.previous
        if previous is not None and time < previous[0]:
            raise RuntimeError('Simulation clock restarted; restart the ROS stack to reset SLAM and odometry')
        if previous is not None and time == previous[0]:
            return False
        self.previous = (time, left, right)
        if previous is None:
            return False
        dt = time - previous[0]
        dl = wrap(left - previous[1]) * RADIUS
        dr = wrap(right - previous[2]) * RADIUS
        distance, angle = (dl + dr) / 2, (dr - dl) / TRACK
        half = angle / 2
        chord = distance * (sin(half) / half if abs(half) > 1e-9 else 1.0)
        self.x += chord * cos(self.yaw + half)
        self.y += chord * sin(self.yaw + half)
        self.yaw = wrap(self.yaw + angle)
        self.speed, self.turn = distance / dt, angle / dt
        self.distance += abs(distance)
        return True
