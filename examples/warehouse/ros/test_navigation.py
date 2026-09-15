import unittest
from math import pi
from navigation import clearance, command, wheels, wrap


class NavigationTest(unittest.TestCase):
    def test_differential_drive(self):
        self.assertEqual(wheels(0, 0), (0, 0))
        left, right = wheels(0.44, 0)
        self.assertAlmostEqual(left, 2)
        self.assertAlmostEqual(right, 2)
        left, right = wheels(0, 1)
        self.assertLess(left, 0)
        self.assertGreater(right, 0)

    def test_forward_turn_and_arrival(self):
        speed, turn, state = command(0, 0, 0, (4, 0), (6, 6, 6))
        self.assertGreater(speed, 0)
        self.assertEqual(turn, 0)
        speed, turn, state = command(0, 0, 0, (0, 3), (6, 6, 6))
        self.assertEqual(speed, 0)
        self.assertGreater(turn, 0)
        self.assertEqual(command(3.9, 0, 0, (4, 0), (6, 6, 6)), (0, 0, 'arrived'))
        self.assertAlmostEqual(wrap(2 * pi + 0.2), 0.2)

    def test_lidar_and_docking_affect_driving(self):
        speed, turn, state = command(0, 0, 0, (4, 0), (0.4, 1, 3))
        self.assertEqual(speed, 0)
        self.assertLess(turn, 0)
        self.assertEqual(state, 'avoiding')
        speed, _, state = command(0, 0, 0, (1, 0), (6, 6, 6), docking=True)
        self.assertLessEqual(speed, 0.2)
        self.assertEqual(state, 'docking')

    def test_scan_angles_and_missing_front(self):
        self.assertEqual(clearance([3, 2, 1], -pi / 2, pi / 2, 6), (2, 1, 3))
        self.assertEqual(clearance([], -pi / 2, 0.1, 6), (0, 0, 0))
        self.assertEqual(clearance([0, float('nan'), 4], -pi / 2, pi / 2, 6), (0, 4, 0))


if __name__ == '__main__':
    unittest.main()
