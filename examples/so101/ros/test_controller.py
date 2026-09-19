"""ROS policy math tests; no simulator/environment inputs."""

import unittest
import numpy as np
import cv2
from kinematics import CAMERA, TCP, fk, ik
from vision import detect, held_color, cube_in_gripper
from motion import (
    clears_bowl,
    camera_floor_clearance,
    plan_move,
    PlanningError,
    closure_floor_clearance,
    choose_grasp_roll,
    pad_boxes,
    CORNERS,
)


class GeometryTest(unittest.TestCase):
    def test_relative_square_pose_without_table_height(self):
        # Camera/gripper relative pose stays observable at arbitrary arm heights.
        f = 480 / (2 * np.tan(np.radians(35)))
        corners = np.array(
            [
                [-0.011, -0.011, 0],
                [0.011, -0.011, 0],
                [0.011, 0.011, 0],
                [-0.011, 0.011, 0],
            ]
        )
        for top in [
            np.array([0.006, -0.003, -0.086]),
            np.array([0.004, 0.002, -0.115]),
        ]:
            rotation, _ = cv2.Rodrigues(np.array([0.12, -0.08, 0.35]))
            points = corners @ rotation.T + top
            optical = (points - CAMERA[:3, 3]) @ CAMERA[:3, :3] * [-1, -1, 1]
            pixels = optical[:, :2] / optical[:, 2, None] * f + [320, 240]
            rgb = np.zeros((480, 640, 3), np.uint8)
            cv2.fillConvexPoly(rgb, np.round(pixels).astype(np.int32), (240, 30, 20))
            recovered = cube_in_gripper(rgb, "red")
            self.assertIsNotNone(recovered)
            np.testing.assert_allclose(recovered, top, atol=0.0015)

    def test_camera_floor_clearance_ignores_nominal_arm_height(self):
        top = np.array([0.0041, 0.0, -0.086])
        normal = np.array([0.0, 0.0, 1.0])
        clearances = []
        for height in [0.013, 0.030]:
            q, _ = ik((0.22, 0.0, height))
            clearances.append(camera_floor_clearance(np.r_[q, 0.4], top, normal))
        self.assertAlmostEqual(clearances[0], clearances[1], places=5)
        self.assertGreater(clearances[0], 0.0015)
        q, _ = ik((0.22, 0.0, 0.020))
        self.assertLess(
            camera_floor_clearance(np.r_[q, 0.4], top + [0, 0, 0.004], normal), 0.0
        )

    def test_reachable_pick_and_observation(self):
        for xyz in [
            (0.20, 0, 0.09),
            (0.22, 0, 0.008),
            (0.15, -0.1, 0.008),
            (0.23, 0.08, 0.008),
        ]:
            q, error = ik(xyz)
            self.assertLess(error, 0.001)
            np.testing.assert_allclose((fk(q) @ TCP)[:3], xyz, atol=0.001)

    def test_diagonal_rim_clearance_keeps_full_margin(self):
        q, _ = ik((0.24, 0.08, 0.015), roll=0.8)
        pose = np.r_[q, 0.4]
        points = []
        for frame, center, size in pad_boxes(pose):
            corners = (center + CORNERS * (size / 2)) @ frame[:3, :3].T + frame[:3, 3]
            points.extend(corners[corners[:, 2] < 0.055, :2])
        points = np.array(points)
        diagonal = np.array([1.0, 1.0]) / np.sqrt(2)
        corner = points[np.argmax(points @ diagonal)]
        self.assertFalse(clears_bowl(pose, corner + 0.084 * diagonal))
        self.assertTrue(clears_bowl(pose, corner + 0.086 * diagonal))

    def test_alternate_square_face_avoids_bowl(self):
        xy = np.array([0.170, -0.030])
        bowl = np.array([0.244, 0.076])
        preferred = 0.982
        q, _ = ik((*xy, 0.08), roll=preferred)
        chosen = choose_grasp_roll(np.r_[q, 0.4], xy, preferred, bowl)
        self.assertAlmostEqual(abs(chosen - preferred), np.pi / 2)
        self.assertGreater(chosen, -2.74385)
        self.assertLess(chosen, 2.84121)

    def test_closure_sweep_detects_table_contact(self):
        low, _ = ik((0.22, 0, 0.013))
        clear, _ = ik((0.22, 0, 0.018))
        self.assertLess(closure_floor_clearance(np.r_[low, 0.4]), 0.0015)
        self.assertGreater(closure_floor_clearance(np.r_[clear, 0.4]), 0.0015)

    def test_bowl_clearance_and_high_transfer(self):
        low, _ = ik((0.20, 0, 0.008))
        high, _ = ik((0.20, 0, 0.11))
        bowl = np.array([0.20, 0.0])
        self.assertFalse(clears_bowl(np.r_[low, 0.4], bowl))
        self.assertTrue(clears_bowl(np.r_[high, 0.4], bowl))
        with self.assertRaises(PlanningError):
            plan_move(np.r_[high, 0.4], (0.20, 0, 0.008), 0.4, bowl, 2.0)
        path = plan_move(np.r_[high, -0.174], (0.24, 0.05, 0.11), -0.174, bowl, 2.0)
        self.assertGreater(len(path), 1)
        self.assertTrue(all(clears_bowl(q, bowl) for q, _ in path))

    def test_grasp_confirmation_rejects_distant_table_cube(self):
        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        rgb[230:260, 300:330] = (255, 0, 0)
        self.assertFalse(held_color(rgb, "red"))
        rgb[60:150, 300:390] = (255, 0, 0)
        self.assertTrue(held_color(rgb, "red"))

    def test_square_top_pose_and_yaw(self):
        q, _ = ik((0.20, 0, 0.09))
        camera = fk(q) @ CAMERA
        yaw = np.deg2rad(28)
        rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        xy = np.array(
            [[-0.011, -0.011], [0.011, -0.011], [0.011, 0.011], [-0.011, 0.011]]
        ) @ rotation.T + (0.23, 0)
        pixels = []
        f = 480 / (2 * np.tan(np.deg2rad(35)))
        for point in xy:
            local = np.linalg.solve(camera, np.r_[point, 0.019, 1.0])
            pixels.append(
                [320 - f * local[0] / local[2], 240 - f * local[1] / local[2]]
            )
        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.fillConvexPoly(rgb, np.round(pixels).astype(np.int32), (255, 0, 0))
        angles = {}
        positions = detect(rgb, q, angles)
        np.testing.assert_allclose(positions["red"], (0.23, 0), atol=0.001)
        error = (angles["red"] - yaw + np.pi / 4) % (np.pi / 2) - np.pi / 4
        self.assertLess(abs(error), np.deg2rad(3))

    def test_camera_color_ray_round_trip(self):
        q, _ = ik((0.20, 0, 0.09))
        camera = fk(q) @ CAMERA
        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        point = np.array([0.23, 0, 0.019, 1])
        local = np.linalg.solve(camera, point)
        f = 480 / (2 * np.tan(np.deg2rad(35)))
        u = round(320 - f * local[0] / local[2])
        v = round(240 - f * local[1] / local[2])
        self.assertTrue(0 <= u < 640 and 0 <= v < 480)
        cv2.circle(rgb, (u, v), 8, (255, 0, 0), -1)
        found = detect(rgb, q)
        np.testing.assert_allclose(found["red"], point[:2], atol=0.001)


if __name__ == "__main__":
    unittest.main()
