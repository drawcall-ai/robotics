"""Verify the real SLAM/Nav2 data path inside the ROS service."""
import argparse
import io
import json
import math
import time

from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CompressedImage, JointState, LaserScan
from std_msgs.msg import Float64, String
from tf2_msgs.msg import TFMessage


class Smoke(Node):
    def __init__(self, patrol=False):
        super().__init__('warehouse_smoke')
        self.counts = dict(odom=0, scan=0, image=0, joints=0, clock=0, status=0)
        self.patrol = patrol
        self.completed = 0
        self.map_cells = 0
        self.edges = set()
        self.moving = set()
        self.clock = None
        self.first_pose = None
        self.moved = False
        topics = ((Odometry, '/odom', self.odom),
                  (LaserScan, '/robot/scan', self.scan),
                  (CompressedImage, '/robot/camera/image/compressed', self.image),
                  (JointState, '/robot/joint_states', self.joints),
                  (Clock, '/clock', self.on_clock),
                  (String, '/robot/status', self.status),
                  (TFMessage, '/tf', self.transforms))
        for kind, topic, callback in topics:
            self.create_subscription(kind, topic, callback, qos_profile_sensor_data)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.map, durable)
        self.create_subscription(TFMessage, '/tf_static', self.transforms, durable)
        for side in ('left', 'right'):
            self.create_subscription(Float64, f'/robot/{side}_wheel/command',
                                     lambda message, side=side: self.wheel(side, message), qos_profile_sensor_data)

    def odom(self, message):
        assert message.header.frame_id == 'odom' and message.child_frame_id == 'base_footprint'
        p = message.pose.pose.position
        assert all(math.isfinite(v) for v in (p.x, p.y, p.z)), 'Nonfinite odometry'
        if self.first_pose is None:
            self.first_pose = (p.x, p.y)
        self.moved |= math.hypot(p.x - self.first_pose[0], p.y - self.first_pose[1]) > 0.05
        self.counts['odom'] += 1

    def scan(self, message):
        assert len(message.ranges) == 361 and message.header.frame_id == 'laser'
        assert message.angle_max - message.angle_min > 6.28
        assert all(math.isfinite(v) and v >= 0 for v in message.ranges)
        self.counts['scan'] += 1

    def image(self, message):
        assert message.header.frame_id == 'camera_optical'
        with Image.open(io.BytesIO(bytes(message.data))) as image:
            image.verify()
        self.counts['image'] += 1

    def joints(self, message):
        assert set(message.name) == {'left_wheel', 'right_wheel'}
        assert len(message.position) == 2 and all(math.isfinite(v) for v in message.position)
        self.counts['joints'] += 1

    def wheel(self, side, message):
        assert math.isfinite(message.data)
        if abs(message.data) > 0.01:
            self.moving.add(side)

    def on_clock(self, message):
        stamp = message.clock.sec + message.clock.nanosec / 1e9
        if self.clock is not None:
            assert stamp >= self.clock, 'Clock reset: restart the ROS stack'
            if stamp > self.clock:
                self.counts['clock'] += 1
        self.clock = stamp

    def map(self, message):
        assert message.header.frame_id == 'map'
        assert len(message.data) == message.info.width * message.info.height
        self.map_cells = sum(cell >= 0 for cell in message.data)
        assert any(cell > 50 for cell in message.data), 'SLAM map has no obstacles'

    def transforms(self, message):
        allowed = {('map', 'odom'), ('odom', 'base_footprint'),
                   ('base_footprint', 'base_link'), ('base_link', 'laser'), ('base_link', 'camera_optical')}
        for transform in message.transforms:
            edge = (transform.header.frame_id, transform.child_frame_id)
            assert edge in allowed, f'Unexpected transform (ground truth leak?): {edge}'
            self.edges.add(edge)

    def status(self, message):
        status = json.loads(message.data)
        assert 'failed' not in status['state'].lower() and 'rejected' not in status['state'].lower(), status['state']
        self.completed = status['completed']
        if status['pose'] is not None:
            self.counts['status'] += 1

    def complete(self):
        return (all(count >= 3 for count in self.counts.values()) and len(self.moving) == 2 and self.moved
                and self.map_cells > 100 and len(self.edges) == 5 and (not self.patrol or self.completed == 4))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=90)
    parser.add_argument('--patrol', action='store_true', help='Require Nav2 to complete all four goals')
    args = parser.parse_args()
    rclpy.init(args=[])
    node = Smoke(args.patrol)
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline and not node.complete():
            rclpy.spin_once(node, timeout_sec=0.1)
        assert node.count_publishers('/robot/pose') == 0, 'Ground-truth pose publisher still present'
        result = dict(streams=node.counts, commanded=sorted(node.moving), encoder_motion=node.moved,
                      known_map_cells=node.map_cells, transforms=sorted(node.edges), completed_goals=node.completed)
        if not node.complete():
            raise RuntimeError(f'SLAM/Nav2 smoke timeout: {result}')
        print(json.dumps(dict(result='pass', **result)))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
