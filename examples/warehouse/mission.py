"""Drive the patrol, then exit; the evaluator judges whether it succeeded."""
import sys
from math import cos, pi, sin

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

GOALS = ((4.0, 0.0, pi / 2), (4.0, 3.0, pi), (0.0, 3.0, -pi / 2), (0.0, 0.0, 0.0))


def pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = sin(yaw / 2), cos(yaw / 2)
    return p


def main():
    rclpy.init()
    nav = BasicNavigator('mission')
    nav.waitUntilNav2Active(localizer='slam_toolbox')
    for i, goal in enumerate(GOALS, 1):
        nav.goToPose(pose(nav, *goal))
        while not nav.isTaskComplete():
            pass
        if nav.getResult() != TaskResult.SUCCEEDED:
            nav.error(f'Goal {i}/{len(GOALS)} failed')
            sys.exit(1)
        nav.info(f'Reached goal {i}/{len(GOALS)}')


try:
    main()
except (KeyboardInterrupt, ExternalShutdownException):
    pass
finally:
    if rclpy.ok():
        rclpy.shutdown()
