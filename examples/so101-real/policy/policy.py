"""The solver's policy goes here. This placeholder only reports the arm's readings and exits."""
from robot import Robot

robot = Robot()
robot.get_logger().info(f"readings {robot.joints}")
