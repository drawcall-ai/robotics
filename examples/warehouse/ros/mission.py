"""Submit operator waypoints to Nav2; localization and control stay in ROS packages."""
import json
from math import cos, pi, sin

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan, CompressedImage
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

GOALS = ((4.0, 0.0, pi / 2), (4.0, 3.0, pi), (0.0, 3.0, -pi / 2), (0.0, 0.0, 0.0))


class Mission(Node):
    def __init__(self):
        super().__init__('warehouse_mission')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.lifecycle = self.create_client(GetState, "/bt_navigator/get_state")
        self.ready = False
        self.checking = False
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.status = self.create_publisher(String, '/robot/status', 1)
        self.waypoint = 0
        self.completed = 0
        self.active = False
        self.failed = False
        self.state = 'Waiting for SLAM and Nav2'
        self.images = self.scans = self.maps = 0
        self.speed = 0.0
        self.pose = None
        self.create_subscription(LaserScan, '/robot/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, '/robot/camera/image/compressed', self.image, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self.map, QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(0.2, self.update)

    def scan(self, _message):
        self.scans += 1

    def image(self, _message):
        self.images += 1

    def odom(self, message):
        self.speed = message.twist.twist.linear.x

    def map(self, _message):
        self.maps += 1

    def update(self):
        if not self.ready and not self.checking and self.lifecycle.service_is_ready():
            self.checking = True
            self.lifecycle.call_async(GetState.Request()).add_done_callback(self.lifecycle_state)
        try:
            transform = self.tf.lookup_transform('map', 'base_footprint', Time())
            self.pose = [transform.transform.translation.x, transform.transform.translation.y]
        except TransformException:
            self.pose = None
        if (self.ready and self.pose is not None and self.maps and self.client.server_is_ready()
                and not self.active and not self.failed and self.completed < len(GOALS)):
            self.send_goal()
        self.status.publish(String(data=json.dumps(dict(
            state=self.state, waypoint=self.waypoint, goal=GOALS[self.waypoint][:2],
            completed=self.completed, pose=self.pose, speed=self.speed,
            images=self.images, scans=self.scans, maps=self.maps))))

    def lifecycle_state(self, future):
        self.checking = False
        self.ready = future.result().current_state.id == State.PRIMARY_STATE_ACTIVE

    def send_goal(self):
        x, y, yaw = GOALS[self.waypoint]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = sin(yaw / 2), cos(yaw / 2)
        self.active = True
        self.state = 'Navigating with Nav2'
        self.client.send_goal_async(goal).add_done_callback(self.accepted)

    def accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.active = False
            self.failed = True
            self.state = 'Nav2 rejected goal'
            self.get_logger().error(self.state)
            return
        handle.get_result_async().add_done_callback(self.finished)

    def finished(self, future):
        result = future.result()
        self.active = False
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.failed = True
            self.state = f'Nav2 goal failed (status {result.status})'
            self.get_logger().error(self.state)
            return
        self.completed += 1
        self.state = 'Circuit complete' if self.completed == len(GOALS) else 'Waypoint reached'
        self.waypoint = min(self.completed, len(GOALS) - 1)
        self.get_logger().info(f'Nav2 completed {self.completed}/{len(GOALS)} goals')


def main():
    rclpy.init()
    node = Mission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
