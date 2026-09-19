"""ROS image-only SO101 pick/place policy.

Only joint positions and wrist RGB images inform actions. Set
SO101_DEBUG_CAMERAS=1 to save optional sensor snapshots beside this script.
"""

import math
import os
import time
from pathlib import Path
from collections import deque

import cv2
import numpy as np

from kinematics import NAMES, TCP, fk
from motion import PlanningError, plan_move, choose_grasp_roll, camera_floor_clearance
from vision import detect, held_color, cube_in_gripper


def main():
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState, CompressedImage
    from std_srvs.srv import Trigger

    class Controller(Node):
        def __init__(self):
            super().__init__("so101_pick_place")
            self.commands = self.create_publisher(
                JointState, "/robot/joint_commands", 10
            )
            self.create_subscription(JointState, "/robot/joint_states", self.joints, 10)
            self.create_subscription(
                CompressedImage, "/robot/wrist_camera/image/compressed", self.image, 10
            )
            self.start = self.create_client(Trigger, "/sim/start")
            self.start_future = None
            self.debug_images = os.environ.get("SO101_DEBUG_CAMERAS") == "1"
            self.started = False
            self.q = None
            self.rgb = None
            self.target = np.zeros(6)
            self.queue = deque()
            self.until = 0
            self.segment_start = 0
            self.initial = self.target.copy()
            self.goal = self.target.copy()
            self.phase = "start"
            self.known = {}
            self.done = set()
            self.scan = 0
            self.visible = {}
            self.color = None
            self.stage = 0
            self.blocked = {}
            self.grasp = None
            self.bowl_ready = False
            self.grip_track = None
            self.carry_grip = -0.174
            self.roll = 0.0
            self.angles = {}
            self.bowl_views = 0
            self.create_timer(0.04, self.tick)

        def joints(self, msg):
            d = dict(zip(msg.name, msg.position))
            if all(n in d for n in NAMES):
                self.q = np.array([d[n] for n in NAMES])

        def image(self, msg):
            self.rgb = cv2.cvtColor(
                cv2.imdecode(
                    np.frombuffer(bytes(msg.data), dtype=np.uint8), cv2.IMREAD_COLOR
                ),
                cv2.COLOR_BGR2RGB,
            )

        def schedule(self, actions, phase):
            planned = []
            start = self.target.copy()
            bowl = self.known.get("bowl")
            try:
                for position, grip, duration in actions:
                    segment = plan_move(
                        start, position, grip, bowl, duration, roll=self.roll
                    )
                    planned.extend(segment)
                    start = segment[-1][0]
            except PlanningError as error:
                self.get_logger().warning(str(error))
                if self.color in self.known:
                    self.blocked[self.color] = self.known[self.color].copy()
                self.phase = "scan"
                return False
            planned.append((start.copy(), 0.5))
            self.queue.extend(planned)
            self.phase = phase
            self.grip_track = None
            return True

        def grip_settled(self, now):
            angle = float(self.q[5])
            if self.grip_track is None:
                self.grip_track = [angle, now, now]
            if abs(angle - self.grip_track[0]) > 0.003:
                self.grip_track[0] = angle
                self.grip_track[1] = now
            return now - self.grip_track[1] > 0.6

        def retry(self):
            position = (fk(self.q) @ TCP)[:3]
            self.schedule([(position, 0.4, 0.7), ((0.20, 0, 0.11), 0.4, 2.0)], "scan")

        def observe(self, only=None):
            if self.rgb is not None and self.q is not None:
                found = detect(self.rgb, self.q, self.angles)
                self.visible = found
                self.known.update(
                    {
                        k: v
                        for k, v in found.items()
                        if (only is None or k == only)
                        and (k != "bowl" or not self.bowl_ready)
                    }
                )
                if self.debug_images:
                    cv2.imwrite(
                        str(Path(__file__).with_name("wrist_latest.jpg")),
                        cv2.cvtColor(self.rgb, cv2.COLOR_RGB2BGR),
                    )
                    Path(__file__).with_name("joints_latest.txt").write_text(
                        repr(self.q.tolist())
                    )
                self.get_logger().info(
                    "vision "
                    + str({k: np.round(v, 3).tolist() for k, v in found.items()})
                )

        def tick(self):
            now = time.monotonic()
            if not self.started:
                if self.start_future is None and self.start.service_is_ready():
                    self.start_future = self.start.call_async(Trigger.Request())
                if self.start_future is not None and self.start_future.done():
                    response = self.start_future.result()
                    if not response.success:
                        raise RuntimeError(
                            f"Episode start rejected: {response.message}"
                        )
                    self.started = True
                    self.schedule([((0.20, 0, 0.09), 0.4, 2.5)], "scan")
                return
            if self.q is None:
                return
            if now >= self.until:
                if self.queue:
                    if self.debug_images and self.rgb is not None:
                        cv2.imwrite(
                            str(
                                Path(__file__).with_name(f"stage_{self.stage:03d}.jpg")
                            ),
                            cv2.cvtColor(self.rgb, cv2.COLOR_RGB2BGR),
                        )
                        Path(__file__).with_name(
                            f"stage_{self.stage:03d}.txt"
                        ).write_text(repr(self.q.tolist()))
                        self.stage += 1
                    self.initial = self.target.copy()
                    self.goal, duration = self.queue.popleft()
                    self.segment_start = now
                    self.until = now + duration
                elif self.phase == "scan":
                    self.observe()
                    candidates = [
                        c
                        for c in ("red", "green", "magenta")
                        if c in self.visible
                        and c not in self.done
                        and (
                            c not in self.blocked
                            or np.linalg.norm(self.visible[c] - self.blocked[c]) > 0.02
                        )
                    ]
                    if "bowl" in self.known and not self.bowl_ready:
                        bowl = self.known["bowl"]
                        self.schedule([((*bowl, 0.11), 0.4, 2.0)], "bowl")
                    elif "bowl" in self.known and candidates:
                        color = candidates[0]
                        xy = self.known[color]
                        if np.linalg.norm(xy - self.known["bowl"]) < 0.055:
                            self.done.add(color)
                        else:
                            self.color = color
                            self.schedule(
                                [((*xy, 0.11), 0.4, 2.0), ((*xy, 0.080), 0.4, 1.0)],
                                "refine",
                            )
                    elif len(self.done) == 3:
                        self.get_logger().info(
                            "All three colors visually in bowl; holding"
                        )
                        self.phase = "hold"
                    else:
                        y = [-0.10, 0.10, 0][self.scan % 3]
                        self.scan += 1
                        self.schedule([((0.20, y, 0.11), 0.4, 2.0)], "scan")
                elif self.phase == "bowl":
                    previous = self.known["bowl"].copy()
                    self.observe("bowl")
                    self.bowl_views += 1
                    bowl = self.known["bowl"]
                    if np.linalg.norm(bowl - previous) > 0.008 and self.bowl_views < 3:
                        self.schedule([((*bowl, 0.11), 0.4, 2.0)], "bowl")
                    else:
                        self.bowl_ready = True
                        self.schedule([((0.20, 0, 0.11), 0.4, 2.0)], "scan")
                elif self.phase == "refine":
                    self.observe(self.color)
                    if self.color not in self.visible:
                        self.phase = "scan"
                    else:
                        axis = fk(self.q)[:3, 0]
                        current = math.atan2(axis[1], axis[0])
                        delta = (self.angles[self.color] - current + math.pi / 4) % (
                            math.pi / 2
                        ) - math.pi / 4
                        preferred = (
                            self.q[4] + delta + math.pi / 2
                        ) % math.pi - math.pi / 2
                        xy = self.known[self.color]
                        try:
                            self.roll = choose_grasp_roll(
                                self.target, xy, preferred, self.known["bowl"]
                            )
                        except PlanningError as error:
                            self.get_logger().warning(str(error))
                            self.blocked[self.color] = xy.copy()
                            self.phase = "scan"
                        else:
                            self.get_logger().info(
                                f"Selected clear wrist roll {self.roll:.3f}"
                            )
                            self.schedule([((*xy, 0.080), 0.4, 1.0)], "aligned")
                elif self.phase == "aligned":
                    self.observe(self.color)
                    if self.color not in self.visible:
                        self.phase = "scan"
                    else:
                        # Leave room during descent; align from close-range RGB below.
                        xy = self.known[self.color] - 0.002 * fk(self.q)[:2, 0]
                        self.grasp = np.r_[xy, 0.015]
                        self.align_attempts = 0
                        self.get_logger().info(
                            f"Picking {self.color} at {xy}, roll {self.roll:.3f}"
                        )
                        self.schedule([(self.grasp, 0.4, 2.0)], "floor")
                elif self.phase == "floor":
                    pose = cube_in_gripper(self.rgb, self.color, with_normal=True)
                    if pose is None:
                        self.get_logger().info("Top face occluded; refreshing approach")
                        self.retry()
                    else:
                        top, normal = pose
                        rotation = fk(self.q)[:3, :3]
                        up = rotation @ normal
                        if up[2] < math.cos(math.radians(5)):
                            self.get_logger().info(
                                "Top face tilted; refreshing approach"
                            )
                            self.retry()
                            return
                        clearance = camera_floor_clearance(self.q, top, normal)
                        delta = rotation @ (top - np.array([0.0041, 0.0, -0.086]))
                        # Keep the entire closing sweep 2mm above the observed table.
                        delta[2] = max(delta[2], 0.002 - clearance)
                        error = float(np.linalg.norm(delta))
                        self.get_logger().info(
                            f"Relative top {np.round(top, 4)}, correction {error * 1000:.1f}mm, desk {clearance * 1000:.1f}mm"
                        )
                        if error <= 0.001 and clearance >= 0.0015:
                            self.schedule([(self.grasp, -0.174, 1.2)], "clamp")
                        elif error < 0.012 and self.align_attempts < 4:
                            self.align_attempts += 1
                            self.grasp = self.grasp + delta
                            self.schedule([(self.grasp, 0.4, 0.8)], "floor")
                        else:
                            self.get_logger().info(
                                "Relative pose did not converge; refreshing approach"
                            )
                            self.retry()
                elif self.phase == "clamp":
                    if self.grip_settled(now):
                        self.get_logger().info(f"Clamp settled at {self.q[5]:.3f}rad")
                        xy = self.grasp[:2]
                        self.carry_grip = max(-0.174, float(self.q[5] - 0.045))
                        self.schedule([((*xy, 0.085), self.carry_grip, 3.0)], "verify")
                    elif now - self.grip_track[2] > 4.0:
                        self.get_logger().info(
                            "Clamp still moving; retrying from fresh image"
                        )
                        self.retry()
                elif self.phase == "verify":
                    stable = self.grip_settled(now)
                    if not stable and now - self.grip_track[2] < 3.0:
                        pass
                    elif (
                        stable and self.q[5] > 0.0 and held_color(self.rgb, self.color)
                    ):
                        self.get_logger().info(
                            f"Wrist image confirms retained {self.color}"
                        )
                        bowl = self.known["bowl"]
                        offset = [(-0.015, -0.010), (0.015, -0.010), (0, 0.018)][
                            len(self.done) % 3
                        ]
                        dest = bowl + offset
                        self.schedule(
                            [
                                ((*dest, 0.085), self.carry_grip, 6.0),
                                ((*dest, 0.080), self.carry_grip, 1.5),
                                ((*dest, 0.080), 0.4, 1.0),
                                ((*dest, 0.11), 0.4, 1.0),
                                ((0.20, 0, 0.11), 0.4, 2.0),
                            ],
                            "scan",
                        )
                    else:
                        self.get_logger().info(
                            f"No retained {self.color}; refreshing camera estimate"
                        )
                        self.retry()
            if self.until > self.segment_start:
                fraction = min(
                    1.0,
                    (now - self.segment_start)
                    / ((self.until - self.segment_start) * 0.75),
                )
                blend = fraction * fraction * (3 - 2 * fraction)
                self.target = self.initial + (self.goal - self.initial) * blend
            msg = JointState()
            msg.name = NAMES
            msg.position = self.target.tolist()
            self.commands.publish(msg)

    rclpy.init()
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
