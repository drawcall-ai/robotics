import io
import json
from math import atan2, hypot

from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage, JointState, LaserScan
from std_msgs.msg import Float64, String

from navigation import GOALS, clearance, command, wheels


class Controller(Node):
    def __init__(self):
        super().__init__('warehouse_controller')
        self.reset()
        self.left = self.create_publisher(Float64, '/robot/left_wheel/command', 1)
        self.right = self.create_publisher(Float64, '/robot/right_wheel/command', 1)
        self.status = self.create_publisher(String, '/robot/status', 1)
        self.create_subscription(PoseStamped, '/robot/pose', self.on_pose, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/robot/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, '/robot/camera/image/compressed', self.on_image, qos_profile_sensor_data)
        self.create_subscription(JointState, '/robot/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_timer(0.05, self.update)
        self.get_logger().info('Waiting for browser pose, lidar and camera; ROS owns wheel commands.')

    def reset(self):
        self.pose = None
        self.pose_time = None
        self.scan_time = None
        self.image_time = None
        self.distances = (0.0, 0.0, 0.0)
        self.orange = 0.0
        self.waypoint = 0
        self.pause_until = None
        self.frames = dict(images=0, scans=0, joint_frames=0, pose_frames=0)

    @staticmethod
    def timestamp(message):
        return message.header.stamp.sec + message.header.stamp.nanosec / 1e9

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_pose(self, message):
        time = self.timestamp(message)
        if self.pose_time is not None and time < self.pose_time:
            self.reset()
            self.get_logger().info("Simulation clock restarted; restarting patrol.")
        p, q = message.pose.position, message.pose.orientation
        yaw = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)
        self.pose_time = time
        self.frames['pose_frames'] += 1

    def on_scan(self, message):
        self.distances = clearance(message.ranges, message.angle_min, message.angle_increment, message.range_max)
        self.scan_time = self.timestamp(message)
        self.frames['scans'] += 1

    def on_image(self, message):
        image = Image.open(io.BytesIO(bytes(message.data))).convert('RGB')
        image.thumbnail((80, 60))
        width, height = image.size
        pixels = list(image.crop((width // 4, 0, 3 * width // 4, height)).getdata())
        orange = sum(r > 150 and 60 < g < 200 and b < 100 and r > 1.3 * g for r, g, b in pixels)
        self.orange = orange / len(pixels)
        self.image_time = self.timestamp(message)
        self.frames['images'] += 1

    def on_joints(self, message):
        self.frames['joint_frames'] += 1

    def update(self):
        now = self.now()
        speed, turn, state = 0.0, 0.0, 'waiting for sensors'
        ready = (self.pose is not None and self.pose_time is not None and
                 self.scan_time is not None and self.image_time is not None)
        if ready and all(-0.1 <= now - time < 0.5 for time in (self.pose_time, self.scan_time, self.image_time)):
            speed, turn, state = self.navigate(now)
        elif ready:
            state = 'sensor timeout'
        left, right = wheels(speed, turn)
        self.left.publish(Float64(data=left))
        self.right.publish(Float64(data=right))
        self.status.publish(String(data=json.dumps(dict(
            state=state, waypoint=self.waypoint, goal=GOALS[self.waypoint],
            orange_fraction=round(self.orange, 4), front_distance=round(self.distances[0], 2),
            **self.frames))))

    def navigate(self, now):
        if self.pause_until is not None:
            if now < self.pause_until:
                return 0.0, 0.0, 'waypoint reached'
            self.pause_until = None
            self.waypoint = (self.waypoint + 1) % len(GOALS)
        x, y, yaw = self.pose
        docking = self.waypoint == 3 and hypot(x, y) < 1.5 and self.orange > 0.012
        result = command(x, y, yaw, GOALS[self.waypoint], self.distances, docking)
        if result[2] == 'arrived':
            self.pause_until = now + 2.0
            self.get_logger().info(f'Arrived at waypoint {self.waypoint}: {GOALS[self.waypoint]}')
        return result


def main():
    rclpy.init()
    node = Controller()
    try:
        rclpy.spin(node)
    finally:
        node.left.publish(Float64(data=0.0))
        node.right.publish(Float64(data=0.0))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
