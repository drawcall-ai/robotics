"""Drive the patrol; the simulator evaluator owns successful completion."""
import sys
from math import cos, pi, sin

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
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
    start = nav.create_client(Trigger, '/sim/start_evaluation')
    if not start.wait_for_service(timeout_sec=30):
        raise RuntimeError('Simulator did not expose /sim/start_evaluation')
    request = start.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(nav, request, timeout_sec=30)
    if not request.done() or request.result() is None or not request.result().success:
        raise RuntimeError('Evaluation could not start')
    for i, goal in enumerate(GOALS, 1):
        nav.goToPose(pose(nav, *goal))
        while not nav.isTaskComplete():
            pass
        if nav.getResult() != TaskResult.SUCCEEDED:
            nav.error(f'Goal {i}/{len(GOALS)} failed')
            sys.exit(1)
        nav.info(f'Reached goal {i}/{len(GOALS)}')
    rclpy.spin(nav)


try:
    main()
except (KeyboardInterrupt, ExternalShutdownException):
    pass
finally:
    if rclpy.ok():
        rclpy.shutdown()
