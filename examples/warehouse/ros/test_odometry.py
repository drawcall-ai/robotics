import unittest
from math import pi
from odometry import RADIUS, TRACK, WheelOdometry, wheel_speeds


class OdometryTests(unittest.TestCase):
    def test_first_sample_establishes_encoder_zero_not_world_position(self):
        odom = WheelOdometry()
        self.assertFalse(odom.update(100, 1.2, -2.4))
        self.assertEqual((odom.x, odom.y, odom.yaw), (0, 0, 0))

    def test_straight_encoder_travel(self):
        odom = WheelOdometry()
        odom.update(0, 0, 0)
        odom.update(0.1, 0.2, 0.2)
        self.assertAlmostEqual(odom.x, 0.2 * RADIUS)
        self.assertAlmostEqual(odom.speed, 2 * RADIUS)
        self.assertEqual(odom.y, 0)

    def test_wrapped_angles_do_not_teleport(self):
        odom = WheelOdometry()
        odom.update(0, pi - 0.1, pi - 0.1)
        odom.update(0.1, -pi + 0.1, -pi + 0.1)
        self.assertAlmostEqual(odom.x, 0.2 * RADIUS)

    def test_in_place_turn_and_command_signs(self):
        odom = WheelOdometry()
        odom.update(0, 0, 0)
        odom.update(0.1, -0.1, 0.1)
        self.assertEqual((odom.x, odom.y), (0, 0))
        self.assertAlmostEqual(odom.yaw, 0.2 * RADIUS / TRACK)
        left, right = wheel_speeds(0, 1)
        self.assertLess(left, 0)
        self.assertGreater(right, 0)

    def test_clock_reset_fails_instead_of_mixing_slam_sessions(self):
        odom = WheelOdometry()
        odom.update(10, 0, 0)
        with self.assertRaisesRegex(RuntimeError, 'restart the ROS stack'):
            odom.update(0, 0, 0)


if __name__ == '__main__':
    unittest.main()
