"""The solver's policy goes here. This placeholder only reports the arm's readings and exits."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot import Robot  # noqa: E402

robot = Robot()
robot.get_logger().info(f"readings {robot.joints}")
