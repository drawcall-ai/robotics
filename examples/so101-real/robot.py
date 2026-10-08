"""A small harness for the SO-101 follower, mirroring what LeRobot gives on the real arm.

    from robot import Robot

    try:
        robot = Robot()
        robot.goal(shoulder_lift=10.0, gripper=50)   # LeRobot degrees, gripper 0..100
        robot.wait(1.0)                               # simulated seconds
        robot.get_logger().info(f"{robot.joints} {robot.loads}")
        rgb = robot.image()                           # 960x1280x3
    except KeyboardInterrupt:  # an eject
        pass

Goals persist: each servo holds its last goal until given another, also between turns, as
Feetech servos do. Readings are the servos' present positions; they are not the true joint
angles to better than a few degrees, and a P-only servo sags under load.
"""
import json
import threading

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


class Robot(Node):
    def __init__(self, name="policy"):
        rclpy.init()
        super().__init__(name)
        # Present_Position by joint: degrees, gripper 0..100.
        self.joints = {}
        # Present_Velocity, degrees (gripper units) per second.
        self.velocities = {}
        # Present_Load, tenths of a percent of stall torque, signed towards increasing readings.
        self.loads = {}
        # Present_Temperature in C, torque enabled, P coefficient: from the status, every 0.5 s.
        self.status = {}
        self.time = 0.0
        self.frame = None
        self.updated = threading.Condition()
        self.goals = self.create_publisher(JointState, "/follower/goal", 10)
        self.gains = self.create_publisher(JointState, "/follower/p_gain", 10)
        self.create_subscription(JointState, "/follower/joint_states", self.measured, 10)
        self.create_subscription(String, "/follower/status", self.reported, 10)
        self.create_subscription(Image, "/robot/wrist_camera/image_raw", self.seen, 2)
        self.create_subscription(Clock, "/clock", self.ticked, 10)
        threading.Thread(target=self.spin, daemon=True).start()
        with self.updated:
            if not self.updated.wait_for(lambda: self.joints, 10):
                raise TimeoutError("No joint states received")

    def spin(self):
        try:
            rclpy.spin(self)
        except ExternalShutdownException:
            pass

    def measured(self, msg):
        with self.updated:
            self.joints = dict(zip(msg.name, msg.position))
            self.velocities = dict(zip(msg.name, msg.velocity))
            self.loads = dict(zip(msg.name, msg.effort))
            self.updated.notify_all()

    def reported(self, msg):
        with self.updated:
            self.status = json.loads(msg.data)
            self.updated.notify_all()

    def seen(self, msg):
        with self.updated:
            self.frame = msg
            self.updated.notify_all()

    def ticked(self, msg):
        with self.updated:
            self.time = msg.clock.sec + msg.clock.nanosec * 1e-9
            self.updated.notify_all()

    def goal(self, **joints):
        """Write Goal_Position of the given joints; the servos clamp it to their calibrated range."""
        unknown = joints.keys() - set(JOINTS)
        if unknown:
            raise ValueError(f"Unknown joints: {', '.join(sorted(unknown))}")
        msg = JointState()
        msg.name = list(joints)
        msg.position = [float(v) for v in joints.values()]
        self.goals.publish(msg)

    def p_gain(self, **joints):
        """Write P_Coefficient of the given joints (LeRobot uses 16; the factory default is 32)."""
        msg = JointState()
        msg.name = list(joints)
        msg.position = [float(v) for v in joints.values()]
        self.gains.publish(msg)

    def wait(self, seconds):
        """Sleep for simulated seconds."""
        with self.updated:
            end = self.time + seconds
            self.updated.wait_for(lambda: self.time >= end)

    def settle(self, timeout=5.0, speed=0.5):
        """Wait until every joint moves slower than `speed` per second, or `timeout` passes;
        returns the readings."""
        with self.updated:
            end = self.time + timeout
            self.updated.wait_for(
                lambda: self.time >= end
                or (self.time > end - timeout + 0.1 and all(abs(v) < speed for v in self.velocities.values()))
            )
            return dict(self.joints)

    def image(self):
        """The first wrist camera frame taken after this call, as an RGB array."""
        after = self.time

        def taken(frame):
            return frame.header.stamp.sec + frame.header.stamp.nanosec * 1e-9

        with self.updated:
            self.updated.wait_for(lambda: self.frame is not None and taken(self.frame) > after)
            frame = self.frame
        rows = np.frombuffer(bytes(frame.data), np.uint8).reshape(frame.height, frame.step)
        return rows[:, : frame.width * 3].reshape(frame.height, frame.width, 3)

    def param(self, name, default):
        """A ROS parameter, such as one set with `--ros-args -p name:=value`."""
        if self.has_parameter(name):
            return self.get_parameter(name).value
        return self.declare_parameter(name, default).value
