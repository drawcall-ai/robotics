"""ROS hardware adapter: encoder odometry and cmd_vel to native wheel motors."""
from math import cos, isfinite, sin
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster

from odometry import WheelOdometry, wheel_speeds


class Bridge(Node):
    def __init__(self):
        super().__init__('warehouse_bridge')
        self.odometry = WheelOdometry()
        self.command = None
        self.encoder_wall = None
        self.left = self.create_publisher(Float64, '/robot/left_wheel/command', 1)
        self.right = self.create_publisher(Float64, '/robot/right_wheel/command', 1)
        self.odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(JointState, '/robot/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 1)
        self.create_timer(0.05, self.drive)

    def on_command(self, message):
        speed, turn = message.linear.x, message.angular.z
        if not all(isfinite(v) for v in (speed, turn)):
            raise ValueError('Nonfinite cmd_vel')
        self.command = (time.monotonic(), *wheel_speeds(speed, turn))

    def on_joints(self, message):
        if len(message.name) != len(message.position):
            raise ValueError('JointState names and positions differ')
        positions = dict(zip(message.name, message.position))
        stamp = message.header.stamp
        if not self.odometry.update(stamp.sec + stamp.nanosec / 1e9,
                                    positions['left_wheel'], positions['right_wheel']):
            return
        self.encoder_wall = time.monotonic()
        state = self.odometry
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x, odom.pose.pose.position.y = state.x, state.y
        odom.pose.pose.orientation.z = sin(state.yaw / 2)
        odom.pose.pose.orientation.w = cos(state.yaw / 2)
        odom.twist.twist.linear.x = state.speed
        odom.twist.twist.angular.z = state.turn
        # Conservative uncalibrated encoder uncertainty; grows with travel and cannot observe slip.
        for index in (0, 7, 35):
            odom.pose.covariance[index] = 0.001 + 0.02 * state.distance
            odom.twist.covariance[index] = 0.02
        for index in (14, 21, 28):
            odom.pose.covariance[index] = odom.twist.covariance[index] = 1e6
        self.odom.publish(odom)
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x = state.x
        transform.transform.translation.y = state.y
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(transform)

    def drive(self):
        now = time.monotonic()
        fresh = (self.command is not None and now - self.command[0] < 0.3 and
                 self.encoder_wall is not None and now - self.encoder_wall < 0.3)
        self.left.publish(Float64(data=self.command[1] if fresh else 0.0))
        self.right.publish(Float64(data=self.command[2] if fresh else 0.0))


def main():
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.left.publish(Float64(data=0.0))
            node.right.publish(Float64(data=0.0))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
