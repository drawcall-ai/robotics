"""ROS policy math tests; no simulator/environment inputs."""

import math
import unittest

import cv2
import numpy as np

from controller import GRASP_Z, HOME, LOOK_Z, Move
from kinematics import CAMERA, LIMITS, TCP, fk, ik
from motion import (
    JOINT_RATE,
    OPEN,
    TOOL_SPEED,
    choose_roll,
    face_gap,
    clearance,
    grasp_pose,
    GUARD,
    pad_bearing,
    path,
    square_roll,
)
from vision import detect

FOCAL = 480 / (2 * np.tan(np.deg2rad(35)))
FIXED_PAD_BOTTOM = np.array([-0.0119, 0.0, -0.080 - 0.0244, 1.0])


def pixel(camera, point):
    local = np.linalg.solve(camera, np.r_[point, 1.0])
    return [320 - FOCAL * local[0] / local[2], 240 - FOCAL * local[1] / local[2]]


class GeometryTest(unittest.TestCase):
    def test_reachable_pick_and_observation(self):
        for xyz in [(0.20, 0, 0.09), (0.22, 0, 0.008), (0.15, -0.1, 0.008), (0.23, 0.08, 0.008)]:
            q, error = ik(xyz)
            self.assertLess(error, 0.001)
            np.testing.assert_allclose((fk(q) @ TCP)[:3], xyz, atol=0.001)

    def test_roll_squares_the_jaws_to_a_face_where_the_grasp_happens(self):
        for xy in [(0.22, 0.0), (0.18, -0.12), (0.25, 0.08), (0.16, 0.13)]:
            for yaw in (0.0, 0.3, -0.6, 1.2):
                roll = square_roll(np.array(xy), yaw)
                q = grasp_pose(np.array(xy), roll)
                self.assertLess(abs(face_gap(pad_bearing(q), yaw)), math.radians(0.5))

    def test_chosen_roll_keeps_the_low_corners_off_the_bowl(self):
        cube = np.array([0.22, 0.0])
        squared = square_roll(cube, 0.0)
        # A bowl as close as the layout allows, straight along the jaw axis.
        axis = fk(grasp_pose(cube, squared))[:2, 0]
        bowl = cube + 0.11 * axis / np.linalg.norm(axis)
        chosen = choose_roll(cube, 0.0, bowl, [])
        self.assertGreater(abs(chosen - squared), 1.0)
        self.assertLess(
            abs(face_gap(pad_bearing(grasp_pose(cube, chosen)), 0.0)),
            math.radians(0.5),
        )
        self.assertGreater(clearance(grasp_pose(cube, chosen), bowl, []), GUARD)
        self.assertGreater(
            clearance(grasp_pose(cube, chosen), bowl, []),
            clearance(grasp_pose(cube, squared), bowl, []),
        )
        self.assertTrue(LIMITS[4, 0] < chosen < LIMITS[4, 1])

    def test_path_stays_on_the_tool_line_within_speed_and_joint_rate(self):
        start = np.r_[ik(HOME)[0], OPEN]
        destination = np.array([0.28, 0.10, 0.015])
        points, seconds = path(start, destination, OPEN, 0.0)
        origin = (fk(start) @ TCP)[:3]
        line = (destination - origin) / np.linalg.norm(destination - origin)
        for point in points:
            tool = (fk(point) @ TCP)[:3]
            offset = tool - origin
            self.assertLess(np.linalg.norm(offset - line * (offset @ line)), 0.005)
        np.testing.assert_allclose((fk(points[-1]) @ TCP)[:3], destination, atol=0.005)
        self.assertGreaterEqual(seconds, np.linalg.norm(destination - origin) / TOOL_SPEED)
        segment = seconds / (len(points) - 1)
        for a, b in zip(points, points[1:]):
            self.assertLessEqual(1.5 * np.max(np.abs(b - a)) / segment, JOINT_RATE + 1e-6)

    def test_move_eases_between_endpoints_and_settles(self):
        first = np.r_[ik(HOME)[0], OPEN]
        last = np.r_[ik((0.25, 0.08, 0.05))[0], OPEN]
        move = Move([first, last], 2.0)
        move.start = 10.0
        np.testing.assert_allclose(move.target(10.0), first)
        np.testing.assert_allclose(move.target(12.0), last)
        np.testing.assert_allclose(move.target(11.0), (first + last) / 2)
        self.assertFalse(move.done(11.0, last))
        self.assertTrue(move.done(12.0, last))
        self.assertFalse(move.done(12.0, first))
        self.assertTrue(move.done(14.5, first))

    def test_pads_clear_the_desk_at_the_grasp_and_the_cube_at_the_look(self):
        q, _ = ik((0.22, 0.0, GRASP_Z))
        self.assertAlmostEqual((fk(q) @ TCP)[2], GRASP_Z, delta=0.001)
        self.assertGreater((fk(q) @ FIXED_PAD_BOTTOM)[2], -0.003)
        q, _ = ik((0.22, 0.0, LOOK_Z))
        self.assertGreater((fk(q) @ FIXED_PAD_BOTTOM)[2], 0.019 + 0.005)


class VisionTest(unittest.TestCase):
    def test_square_top_pose_and_yaw(self):
        q, _ = ik((0.20, 0, 0.09))
        camera = fk(q) @ CAMERA
        yaw = np.deg2rad(28)
        rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        corners = np.array([[-0.011, -0.011], [0.011, -0.011], [0.011, 0.011], [-0.011, 0.011]])
        xy = corners @ rotation.T + (0.23, 0)
        pixels = [pixel(camera, np.r_[point, 0.019]) for point in xy]
        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.fillConvexPoly(rgb, np.round(pixels).astype(np.int32), (255, 0, 0))
        angles = {}
        positions = detect(rgb, q, angles)
        np.testing.assert_allclose(positions["red"], (0.23, 0), atol=0.001)
        error = (angles["red"] - yaw + np.pi / 4) % (np.pi / 2) - np.pi / 4
        self.assertLess(abs(error), np.deg2rad(3))

    def test_camera_color_ray_round_trip(self):
        q, _ = ik((0.20, 0, 0.09))
        point = np.array([0.23, 0, 0.019])
        u, v = (round(c) for c in pixel(fk(q) @ CAMERA, point))
        self.assertTrue(0 <= u < 640 and 0 <= v < 480)
        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.circle(rgb, (u, v), 8, (255, 0, 0), -1)
        np.testing.assert_allclose(detect(rgb, q)["red"], point[:2], atol=0.001)


if __name__ == "__main__":
    unittest.main()
