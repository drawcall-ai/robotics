"""A small harness for writing an SO101 policy as a plain script.

    from robot import Robot

    try:
        robot = Robot()
        robot.move(shoulder_lift=-0.5, gripper=0.8)
        rgb = robot.image()
    except KeyboardInterrupt:  # an eject
        pass

The node spins in a thread of its own, so the script may block. Its timer keeps publishing the
last target at 50 Hz: the servos let go of a target that is not refreshed within 0.5 s.
"""
import threading

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState


class Robot(Node):
    def __init__(self, name="policy"):
        rclpy.init()
        super().__init__(name)
        # Joint positions by name, as last measured.
        self.joints = {}
        # Joint positions by name, as commanded; empty until the first move.
        self.target = {}
        self.frame = None
        # Notified on every joint state and image.
        self.updated = threading.Condition()
        self.commands = self.create_publisher(JointState, "/robot/joint_commands", 10)
        self.create_subscription(JointState, "/robot/joint_states", self.measured, 10)
        self.create_subscription(Image, "/robot/wrist_camera/image_raw", self.seen, 10)
        self.create_timer(0.02, self.command)
        threading.Thread(target=self.spin, daemon=True).start()

    def spin(self):
        try:
            rclpy.spin(self)
        except ExternalShutdownException:
            pass

    def measured(self, msg):
        with self.updated:
            self.joints = dict(zip(msg.name, msg.position))
            self.updated.notify_all()

    def seen(self, msg):
        with self.updated:
            self.frame = msg
            self.updated.notify_all()

    def command(self):
        target = self.target
        if not target:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(target)
        msg.position = [float(v) for v in target.values()]
        self.commands.publish(msg)

    def move(self, tolerance=0.01, timeout=10.0, **joints):
        """Set the targets of the given joints, the others keeping theirs, and wait until each
        is within `tolerance` radians of its target, raising TimeoutError after `timeout` seconds.
        A gripper closing on something stops short of its target: give it a tolerance it can
        reach."""
        with self.updated:
            if not self.updated.wait_for(lambda: self.joints, timeout):
                raise TimeoutError("No joint states received")
            unknown = joints.keys() - self.joints.keys()
            if unknown:
                raise ValueError(f"Unknown joints: {', '.join(sorted(unknown))}")
            self.target = {**self.joints, **self.target, **joints}

            def off():
                return [n for n, v in joints.items() if abs(self.joints[n] - v) > tolerance]

            if not self.updated.wait_for(lambda: not off(), timeout):
                raise TimeoutError(f"Joints off target after {timeout} s: {', '.join(off())}")

    def image(self):
        """The first wrist camera frame taken after this call, as an RGB array."""
        after = self.get_clock().now().nanoseconds

        def taken(frame):
            return frame.header.stamp.sec * 10**9 + frame.header.stamp.nanosec

        with self.updated:
            self.updated.wait_for(lambda: self.frame is not None and taken(self.frame) > after)
            frame = self.frame
        if frame.encoding != "rgb8":
            raise ValueError(f"Expected an rgb8 image, got {frame.encoding}")
        rows = np.frombuffer(bytes(frame.data), np.uint8).reshape(frame.height, frame.step)
        return rows[:, : frame.width * 3].reshape(frame.height, frame.width, 3)

    def param(self, name, default):
        """A ROS parameter, such as one set with `--ros-args -p name:=value`."""
        if self.has_parameter(name):
            return self.get_parameter(name).value
        return self.declare_parameter(name, default).value
