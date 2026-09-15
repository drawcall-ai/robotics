"""Run inside the service while the browser simulation is active."""
import argparse
import io
import json
import math
import time

from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CompressedImage, JointState, LaserScan
from std_msgs.msg import Float64, String


class Smoke(Node):
    def __init__(self, patrol=False):
        super().__init__('warehouse_smoke')
        self.counts = dict(pose=0, scan=0, image=0, joints=0, left=0, right=0, clock=0, status=0)
        self.patrol = patrol
        self.visited = set()
        self.docked = False
        self.moving = set()
        self.clock = None
        self.first_pose = None
        self.moved = False
        topics = ((PoseStamped, '/robot/pose', self.pose),
                  (LaserScan, '/robot/scan', self.scan),
                  (CompressedImage, '/robot/camera/image/compressed', self.image),
                  (JointState, '/robot/joint_states', self.joints),
                  (Clock, '/clock', self.on_clock),
                  (String, '/robot/status', self.status))
        for kind, topic, callback in topics:
            self.create_subscription(kind, topic, callback, qos_profile_sensor_data)
        for side in ('left', 'right'):
            self.create_subscription(Float64, f'/robot/{side}_wheel/command',
                                     lambda message, side=side: self.wheel(side, message), qos_profile_sensor_data)

    def pose(self, message):
        p = message.pose.position
        assert all(math.isfinite(v) for v in (p.x, p.y, p.z)), 'Nonfinite pose'
        if self.first_pose is None:
            self.first_pose = (p.x, p.y)
        self.moved |= math.hypot(p.x - self.first_pose[0], p.y - self.first_pose[1]) > 0.05
        self.counts['pose'] += 1

    def scan(self, message):
        assert len(message.ranges) >= 10, 'Missing lidar rays'
        assert message.angle_increment > 0 and message.range_max > message.range_min
        assert all(math.isfinite(v) and v >= 0 for v in message.ranges), 'Invalid lidar range'
        self.counts['scan'] += 1

    def image(self, message):
        with Image.open(io.BytesIO(bytes(message.data))) as image:
            image.verify()
        self.counts['image'] += 1

    def joints(self, message):
        assert len(message.name) >= 2 and len(message.velocity) == len(message.name)
        assert all(math.isfinite(v) for v in message.velocity)
        self.counts['joints'] += 1

    def wheel(self, side, message):
        assert math.isfinite(message.data), 'Nonfinite wheel command'
        self.counts[side] += 1
        if abs(message.data) > 0.01:
            self.moving.add(side)

    def on_clock(self, message):
        stamp = message.clock.sec + message.clock.nanosec / 1e9
        if self.clock is not None:
            assert stamp >= self.clock, 'Simulation clock moved backwards'
            if stamp > self.clock:
                self.counts['clock'] += 1
        self.clock = stamp

    def status(self, message):
        status = json.loads(message.data)
        if status['state'] in ('arrived', 'waypoint reached'):
            self.visited.add(status['waypoint'])
        self.docked |= status['state'] == 'docking'
        if status['images'] > 0 and status['scans'] > 0 and status['joint_frames'] > 0:
            self.counts['status'] += 1

    def complete(self):
        if self.patrol and (len(self.visited) < 4 or not self.docked):
            return False
        return all(count >= 3 for count in self.counts.values()) and len(self.moving) == 2 and self.moved


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=60)
    parser.add_argument('--patrol', action='store_true', help='Require all four waypoints and camera-assisted docking')
    args = parser.parse_args()
    rclpy.init(args=[])
    node = Smoke(args.patrol)
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline and not node.complete():
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.complete():
            raise RuntimeError(f'ROS smoke timeout: streams={node.counts}, moving={node.moving}, pose_moved={node.moved}, visited={node.visited}, docked={node.docked}')
        print(json.dumps(dict(result='pass', counts=node.counts, commanded=list(node.moving), pose_moved=node.moved, visited=sorted(node.visited), docked=node.docked)))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
