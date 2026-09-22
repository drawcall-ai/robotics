"""SO101 pick-and-place from wrist images and joint positions alone.

The policy is the plain script in `Controller.script`: survey the desk once,
then for each cube hover over it, look again, square the jaws to a face, pick
it up and drop it into the bowl. The node only plays that script, one action at
a time, publishing joint-position targets at a steady rate.
"""

import math
import os
import time
from pathlib import Path

import cv2
import numpy as np

from kinematics import TCP, Arm
from motion import CLOSED, GRASP_Z, OPEN, choose_roll, duration, grasp_tool, path
from vision import color_masks, detect

COLORS = ("red", "green", "magenta")
# Fingers-down pose over the desk that the arm moves between tasks.
HOME = (0.20, 0.0, 0.12)
# Joints that point the camera across the whole desk from 25cm up.
SURVEY = [-0.02, -1.40, 0.58, 1.65, -1.51]
# Tool heights: the second look at a cube, the pads bracketing it (pad bottoms a
# few millimetres over the desk), travel clear of the 45mm bowl rim with a cube
# hanging under the pads, and the release just above that rim.
LOOK_Z = 0.07
CARRY_Z = 0.12
RELEASE_Z = 0.075
# Where each cube is let go, around the bowl centre: three cubes dropped on one
# spot stack into a tower that stands over the rim.
SPOTS = [
    0.03 * np.array([math.cos(a), math.sin(a)])
    for a in (math.pi / 2, math.pi / 2 + 2 * math.pi / 3, math.pi / 2 - 2 * math.pi / 3)
]
# A jaw that stops short of its closed limit is resting on a cube.
HELD_JAW = -0.05
SETTLED = 0.01


class Move:
    """Eased joint trajectory through `points`; done once the joints have settled."""

    def __init__(self, points, seconds):
        self.points = points
        self.seconds = seconds
        self.start = 0.0

    def target(self, now):
        f = min(1.0, (now - self.start) / self.seconds)
        s = f * f * (3 - 2 * f) * (len(self.points) - 1)
        i = min(int(s), len(self.points) - 2)
        return self.points[i] + (self.points[i + 1] - self.points[i]) * (s - i)

    def done(self, now, q):
        if now - self.start < self.seconds:
            return False
        error = np.max(np.abs(q[:5] - self.points[-1][:5]))
        return error < SETTLED or now - self.start > self.seconds + 2.0


class Look:
    """Waits for an image taken after the arm came to rest."""

    start = 0.0

    def done(self, image_time):
        return image_time is not None and image_time > self.start + 0.1


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import CompressedImage, JointState
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    class Controller(Node):
        def __init__(self):
            super().__init__("so101_pick_place")
            self.commands = self.create_publisher(
                JointState, "/robot/joint_commands", 10
            )
            self.create_subscription(
                String,
                "/robot/robot_description",
                self.description,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
            self.create_subscription(JointState, "/robot/joint_states", self.joints, 10)
            self.create_subscription(
                CompressedImage, "/robot/wrist_camera/image_raw/compressed", self.image, 10
            )
            # What the policy saw, for the viewer and recordings: the last look
            # with its detections and estimates drawn on it.
            self.annotated = self.create_publisher(
                CompressedImage, "/robot/perception/image/compressed", 10
            )
            self.start = self.create_client(Trigger, "/sim/start_evaluation")
            self.start_future = None
            self.debug = os.environ.get("SO101_DEBUG_CAMERAS") == "1"
            self.snapshots = 0
            self.arm = None
            self.home = None
            self.q = None
            self.rgb = None
            self.image_header = None
            self.image_time = None
            self.target = None
            self.action = None
            self.task = None
            self.create_timer(0.02, self.tick)

        def description(self, msg):
            self.arm = Arm(msg.data)
            self.home, _ = self.arm.ik(HOME)

        def joints(self, msg):
            if self.arm is None:
                return
            d = dict(zip(msg.name, msg.position))
            if all(n in d for n in self.arm.names):
                self.q = np.array([d[n] for n in self.arm.names])

        def image(self, msg):
            self.image_header = msg.header
            self.rgb = cv2.cvtColor(
                cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR),
                cv2.COLOR_BGR2RGB,
            )
            self.image_time = time.monotonic()

        def tick(self):
            now = time.monotonic()
            if self.q is None:
                return
            if self.target is None:
                self.target = self.q.copy()
            if self.task is None:
                if self.start_future is None and self.start.service_is_ready():
                    self.start_future = self.start.call_async(Trigger.Request())
                if self.start_future is not None and self.start_future.done():
                    response = self.start_future.result()
                    if not response.success:
                        raise RuntimeError(f"Episode start rejected: {response.message}")
                    self.task = self.script()
            elif self.action is None or self.finished(now):
                self.action = next(self.task, None)
                if self.action is not None:
                    self.action.start = now
            if isinstance(self.action, Move):
                self.target = self.action.target(now)
            msg = JointState()
            msg.name = self.arm.names
            msg.position = self.target.tolist()
            self.commands.publish(msg)

        def finished(self, now):
            if isinstance(self.action, Move):
                return self.action.done(now, self.q)
            if self.action.done(self.image_time):
                self.snapshot()
                return True
            return False

        def snapshot(self):
            if not self.debug:
                return
            here = Path(__file__).parent
            cv2.imwrite(
                str(here / f"stage_{self.snapshots:03d}.jpg"),
                cv2.cvtColor(self.rgb, cv2.COLOR_RGB2BGR),
            )
            (here / f"stage_{self.snapshots:03d}.txt").write_text(repr(self.q.tolist()))
            self.snapshots += 1

        # The script: each yield hands the node one action and resumes once it is done.

        def joint_move(self, joints, grip, seconds=2.0):
            goal = np.r_[joints, grip]
            return Move([self.target.copy(), goal], duration(self.target, goal, seconds))

        def tool_move(self, xyz, grip, roll, seconds=1.0):
            return Move(*path(self.arm, self.target, xyz, grip, roll, seconds))

        def grip_move(self, grip, seconds=1.5):
            goal = self.target.copy()
            goal[5] = grip
            return Move([self.target.copy(), goal], seconds)

        def tool(self):
            return (self.arm.fk(self.q) @ TCP)[:3]

        def look(self, only=None):
            """Colour positions and yaws on the desk from the latest image."""
            yaws = {}
            seen = detect(self.arm, self.rgb, self.q, yaws)
            self.get_logger().info(
                "vision " + str({k: np.round(v, 3).tolist() for k, v in seen.items()})
            )
            found = {k: (v, yaws.get(k, 0.0)) for k, v in seen.items() if only in (None, k)}
            self.show(found)
            return found

        def show(self, found):
            """Publish the look with its detections drawn on it."""
            image = cv2.cvtColor(self.rgb, cv2.COLOR_RGB2BGR)
            masks = color_masks(self.rgb)
            for name, (xy, yaw) in found.items():
                contours, _ = cv2.findContours(
                    masks[name].astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )
                contour = max(contours, key=cv2.contourArea)
                cv2.drawContours(image, [contour], -1, (255, 255, 255), 2)
                x, y, _, _ = cv2.boundingRect(contour)
                label = f"{name} x{xy[0]:.3f} y{xy[1]:.3f}"
                if name != "bowl":
                    label += f" yaw{math.degrees(yaw):.0f}"
                cv2.putText(
                    image, label, (x, max(y - 6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA,
                )
            msg = CompressedImage()
            msg.header = self.image_header
            msg.format = "jpeg"
            msg.data = cv2.imencode(".jpg", image)[1].tobytes()
            self.annotated.publish(msg)

        def script(self):
            yield self.joint_move(self.home, OPEN, 3.0)
            yield self.joint_move(SURVEY, OPEN, 3.0)
            yield Look()
            found = self.look()
            if "bowl" not in found:
                self.get_logger().error("No bowl in view; holding")
                return
            bowl = found["bowl"][0]
            # A second, closer look at the bowl from straight above it.
            yield self.joint_move(self.home, OPEN, 3.0)
            yield self.tool_move((*bowl, CARRY_Z), OPEN, 0.0)
            yield Look()
            bowl = self.look("bowl").get("bowl", (bowl,))[0]
            cubes = {c: found[c] for c in COLORS if c in found}
            attempts = {c: 0 for c in COLORS}
            placed = 0
            while True:
                pending = [
                    c
                    for c in cubes
                    if attempts[c] < 3 and np.linalg.norm(cubes[c][0] - bowl) > 0.06
                ]
                if not pending:
                    break
                color = min(pending, key=lambda c: np.linalg.norm(cubes[c][0] - self.tool()[:2]))
                attempts[color] += 1
                xy, yaw = cubes[color]
                others = [cubes[c][0] for c in cubes if c != color]
                try:
                    roll = choose_roll(self.arm, xy, yaw, bowl, others)
                except ValueError as error:
                    self.get_logger().warning(str(error))
                    continue
                yield self.tool_move((*xy, LOOK_Z), OPEN, roll)
                yield Look()
                seen = self.look(color)
                if color not in seen:
                    self.get_logger().warning(f"Lost {color}; surveying again")
                    yield from self.survey(cubes, bowl)
                    continue
                cubes[color] = seen[color]
                xy, yaw = seen[color]
                try:
                    roll = choose_roll(self.arm, xy, yaw, bowl, others)
                except ValueError as error:
                    self.get_logger().warning(str(error))
                    continue
                yield self.tool_move((*xy, LOOK_Z), OPEN, roll)
                grasp = grasp_tool(self.arm, xy, self.target)
                self.get_logger().info(f"Picking {color} at {grasp.round(3)}, roll {roll:.2f}")
                yield self.tool_move((*grasp, GRASP_Z), OPEN, roll, 2.0)
                yield self.grip_move(CLOSED)
                yield self.grip_move(CLOSED, 1.0)
                yield self.tool_move((*grasp, CARRY_Z), CLOSED, roll, 2.0)
                if self.q[5] < HELD_JAW:
                    self.get_logger().warning(f"Missed {color} (jaw {self.q[5]:.3f})")
                    yield self.grip_move(OPEN)
                    yield from self.survey(cubes, bowl)
                    continue
                spot = bowl + SPOTS[placed % len(SPOTS)]
                yield self.tool_move((*spot, CARRY_Z), CLOSED, roll)
                yield self.tool_move((*spot, RELEASE_Z), CLOSED, roll)
                yield self.grip_move(OPEN)
                yield self.grip_move(OPEN, 1.0)
                yield self.tool_move((*spot, CARRY_Z), OPEN, roll)
                cubes[color] = (bowl.copy(), 0.0)
                placed += 1
            yield self.tool_move(HOME, OPEN, 0.0)
            self.get_logger().info("Done")

        def survey(self, cubes, bowl):
            """Look over the whole desk again for cubes that have moved."""
            yield self.tool_move(HOME, OPEN, 0.0)
            yield self.joint_move(SURVEY, OPEN, 3.0)
            yield Look()
            found = self.look()
            for color in COLORS:
                if color in found and np.linalg.norm(found[color][0] - bowl) > 0.06:
                    cubes[color] = found[color]
            yield self.joint_move(self.home, OPEN, 3.0)

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
