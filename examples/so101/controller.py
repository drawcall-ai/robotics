"""SO101 chess controller from wrist images and joint positions alone.

The policy is the plain script in `Controller.script`; the node plays it one action at a
time, publishing joint-position targets at a steady rate. Vision runs in a worker thread
so the targets keep flowing while an image is being measured.

Survey the desk, handle one piece, and survey again: a lying piece is stood up, a standing
one looked at from closer, grasped at its middle and set on its home square. When every piece
is in place, or none is left it can handle, the node exits, which ends the episode.
"""

import math
import os
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from kinematics import TCP, Arm
from motion import CLOSED, OPEN, PAD_DEPTH, between_pads, clearance, duration, grasp_tool, path
from detector import Detector
from segment import Segmenter
from vision import Shape, Survey, facing, piece_shapes, View, ground, height, outline, pieces, position, sightings, vote

# Survey views of the reachable desk, as (elevation, azimuth, distance) of the camera
# around a point over the middle of the board's usual place, then two of the desk's sides.
# The board is measured in the first ones; pieces are searched for in all.
CENTER = np.array([0.2, 0.0, 0.01])
SURVEY = [(90, 180, 0.24), (70, 135, 0.2), (70, -135, 0.2), (70, 45, 0.2), (70, -45, 0.2)]
# Desk points beside the board, on arcs through the outer and inner half of the reach, as
# (distance from the base, bearing, camera elevation and azimuth from the point, degrees):
# the outer ones looked at from the robot's side, the inner ones, which the arm cannot look
# at from there, from outside.
SIDES = [(0.25, b, 60, b + 180) for b in (35, -35, 60, -60)] + [(0.16, b, 70, b) for b in (40, -40, 70, -70)]
SIDE_DISTANCE = 0.2
# Fingers-down pose the arm moves through between tasks, tool point in the base frame.
HOME = (0.2, 0.0, 0.14)
# Close looks at a piece, in the order tried: camera distance, elevation and azimuth,
# turned from the direction towards the robot. Near the robot only the nearer ones and
# the look straight down are in reach. The detector wants pieces small in the frame and
# seen from the side, as in the photos it learned from.
CLOSE_LOOKS = [
    (d, math.radians(e), turn) for d in (0.15, 0.12) for e in (40, 55, 75) for turn in (0.0, 0.9, -0.9, 1.8, -1.8)
] + [(0.12, math.radians(90), 0.0)]
# Before anything is known, the gripper stays this high over the desk: above a king on
# the thickest board.
SURVEY_CLEARANCE = 0.08
# The gripper keeps this much air over everything it passes.
MARGIN = 0.01
# Camera heights over a piece for the look straight down, in the order tried: near the
# robot only the lower ones are in reach, over a tall piece only the higher ones clear it.
DOWN_HEIGHTS = (0.18, 0.15, 0.12, 0.21, 0.24)
# How far from where a piece is expected it is searched for from above, in squares.
DOWN_SEARCH = 2.0
# How far a lying piece is turned up before it is let go of, most first: a weighted piece
# falls upright from within about a third of a right angle of standing.
PIVOT_PITCHES = tuple(math.radians(p) for p in (70, 60))
PIVOT_LEAST = math.radians(55)
# How far a planned pose may miss its target, and so how deep it may seem to reach.
POSE_TOLERANCE = 0.002
# Where along a lying piece, as shares of its length from the base, it is gripped to be
# turned up: where its head half is thickest, which turned pieces up; gripped nearer its low
# centre of mass, pieces fell back more often.
PIVOT_GRIPS = ((0.5, 0.9),)
# How far the fingers back out of a piece they let go of, before rising twice that.
RETREAT = 0.03
# The widest frame a board has around its squares.
BOARD_FRAME = 0.02
# Room left between a lying piece and the fixed pad as the jaws come down beside it, for
# the error in where it lies; and how far the pad bottoms stay over the surface under it.
GRIP_GAP = 0.005
PAD_CLEARANCE = 0.001
# Detection confidence for the survey's candidates, below the usual 0.4: each is checked
# again from closer.
SURVEY_CONFIDENCE = 0.25
# A piece's outline from above in squares: standard sets stand pieces from about one square
# (pawns) to two (kings) tall on bases at most about four fifths of a square wide, so that
# they stand on one. Lying, a piece is at least most of a square long.
PIECE_LENGTH = (0.3, 2.5)
PIECE_WIDTH = (0.3, 0.85)
LYING_LENGTH = 0.7
# Pixels between the points the overview and desk views are segmented at.
GRID_SPACING = 40
# The least cosine between the fingers and straight down that counts as fingers down.
FINGERS_DOWN = 0.98
# Attempts at a piece before it is given up on.
ATTEMPTS = 2
# How often the scene is looked at and one piece handled, at most: every piece lying and
# stood up first takes two.
ROUNDS = 40
# Black's home squares, rank 8 nearest the robot.
SQUARES = {
    "rook": ("a8", "h8"), "knight": ("b8", "g8"), "bishop": ("c8", "f8"),
    "queen": ("d8",), "king": ("e8",), "pawn": tuple(f"{f}7" for f in "abcdefgh"),
}
# A carried piece's base keeps this much air over the board, less only where the arm
# cannot rise higher, close to its base; it is let go of this far above the board.
CARRY_CLEARANCES = (0.02, 0.01)
RELEASE_GAP = 0.002
# A jaw that stops short of its closed limit is holding something.
HELD_JAW = -0.15
SETTLED = 0.01
# How long the arm holds still after the script ends before the controller exits: longer
# than the evaluator watches pieces to judge them at rest.
SETTLE_TIME = 3.0
# Joints that move less than this between two readings are at rest.
STILL = 0.001


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
    """Waits for an image taken while the arm is at rest, so the joint positions it is
    paired with are where the camera was."""

    start = 0.0
    # The joints when the arm was last seen still.
    still = None


class Work:
    """Waits for a function to finish in the worker thread; its result is `value`."""

    def __init__(self, function, *args):
        self.function = function
        self.args = args
        self.future = None
        self.start = 0.0

    @property
    def value(self):
        return self.future.result()


def aim_point(elevation, azimuth, distance, center=CENTER):
    e, a = math.radians(elevation), math.radians(azimuth)
    camera = center + distance * np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])
    return camera, center - camera


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import CompressedImage, Image, JointState
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    class Controller(Node):
        def __init__(self):
            super().__init__("so101_chess")
            self.commands = self.create_publisher(JointState, "/robot/joint_commands", 10)
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, "/robot/robot_description", self.description, latched)
            self.create_subscription(JointState, "/robot/joint_states", self.joints, 10)
            # The raw image: the simulator compresses only for subscribers, and it saves a decode.
            self.create_subscription(Image, "/robot/wrist_camera/image_raw", self.image, 10)
            # What the policy saw, for the viewer and recordings.
            self.annotated = self.create_publisher(CompressedImage, "/robot/perception/image/compressed", 10)
            self.start = self.create_client(Trigger, "/sim/start_evaluation")
            self.start_future = None
            self.worker = ThreadPoolExecutor(1)
            self.detector = Detector()
            self.segmenter = Segmenter()
            self.debug = os.environ.get("SO101_DEBUG_CAMERAS") == "1"
            self.snapshots = 0
            self.arm = None
            self.q = None
            self.rgb = None
            self.image_header = None
            self.image_time = None
            self.target = None
            self.action = None
            self.task = None
            # When the script ran out.
            self.ended = None
            self.create_timer(0.02, self.tick)

        def seconds(self):
            """Simulation time under use_sim_time, as the launch file sets it."""
            return self.get_clock().now().nanoseconds * 1e-9

        def description(self, msg):
            self.arm = Arm(msg.data)
            # The shoulder's pan axis: the arm turns in a plane through it.
            self.pan_axis = (self.arm.turn @ self.arm.origins[0] @ self.arm.turn.T)[:3, 3]
            if self.debug:
                (Path(__file__).parent / "stage_robot.urdf").write_text(msg.data)

        def joints(self, msg):
            if self.arm is None:
                return
            d = dict(zip(msg.name, msg.position))
            if all(n in d for n in self.arm.names):
                self.q = np.array([d[n] for n in self.arm.names])

        def image(self, msg):
            self.image_header = msg.header
            self.rgb = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, 3)
            # When the camera took it, not when it arrived.
            self.image_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        def tick(self):
            now = self.seconds()
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
            elif self.ended is not None:
                # Done or given up, nothing changes any more: exiting ends the episode, once
                # the evaluator has seen the scene at rest.
                if now - self.ended > SETTLE_TIME:
                    raise SystemExit
            elif self.action is None or self.finished(now):
                self.action = next(self.task, None)
                if self.action is None:
                    self.ended = now
                else:
                    self.action.start = now
                    if isinstance(self.action, Work):
                        self.action.future = self.worker.submit(self.action.function, *self.action.args)
            if isinstance(self.action, Move):
                self.target = self.action.target(now)
            msg = JointState()
            msg.name = self.arm.names
            msg.position = self.target.tolist()
            self.commands.publish(msg)

        def finished(self, now):
            if isinstance(self.action, Move):
                return self.action.done(now, self.q)
            if isinstance(self.action, Work):
                return self.action.future.done()
            # Wait again whenever the arm has moved since.
            if self.action.still is None or np.max(np.abs(self.q[:5] - self.action.still[:5])) > STILL:
                self.action.still = self.q.copy()
                self.action.start = now
                return False
            if self.image_time is not None and self.image_time > self.action.start:
                self.snapshot()
                return True
            return False

        def snapshot(self):
            if not self.debug:
                return
            here = Path(__file__).parent
            cv2.imwrite(str(here / f"stage_{self.snapshots:03d}.jpg"), cv2.cvtColor(self.rgb, cv2.COLOR_RGB2BGR))
            (here / f"stage_{self.snapshots:03d}.txt").write_text(repr(self.q.tolist()))
            self.snapshots += 1

        def show(self, view, marks):
            """Publish a look with what was measured in it drawn on it."""
            image = cv2.cvtColor((view.rgb * 255).astype(np.uint8), cv2.COLOR_RGB2BGR)
            for point, label in marks:
                uv, depth = view.project(np.asarray(point, float))
                if depth > 0:
                    p = tuple(int(v) for v in uv)
                    cv2.circle(image, p, 4, (0, 0, 255), 2)
                    cv2.putText(image, label, (p[0] + 6, p[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)
            msg = CompressedImage()
            msg.header = self.image_header
            msg.format = "jpeg"
            msg.data = cv2.imencode(".jpg", image)[1].tobytes()
            self.annotated.publish(msg)

        # The script: each yield hands the node one action and resumes once it is done.

        def joint_move(self, joints, grip, seconds=2.0):
            goal = np.r_[joints, grip]
            return Move([self.target.copy(), goal], duration(self.target, goal, seconds))

        def grip_move(self, grip, seconds=1.5):
            goal = self.target.copy()
            goal[5] = grip
            return Move([self.target.copy(), goal], seconds)

        def view(self):
            return View(self.arm, self.q, self.rgb)

        def surface(self, survey):
            """The height of the desk or the board, frame and all, under each of an array of
            xy points: what the gripper must keep clear of."""
            board = survey.board

            def under(xy):
                d = xy - board.origin
                inside = np.maximum(np.abs(d @ board.u), np.abs(d @ board.v)) < 4 * board.s + BOARD_FRAME
                return np.where(inside, board.z, survey.floor)

            return under

        def clear_move(self, joints, obstacles, floor):
            """Move to joints without the gripper passing through an obstacle on the way,
            going by the home pose, high over everything, when the direct way would."""
            start = self.target.copy()
            goal = np.r_[joints, OPEN]
            if min(clearance(self.arm, start + f * (goal - start), obstacles, floor) for f in np.linspace(0, 1, 12)) < MARGIN:
                home, _ = self.arm.ik(HOME, self.target)
                yield self.joint_move(home[:5], OPEN, 2.0)
            yield self.joint_move(joints, OPEN, 2.0)

        def look_from(self, camera, direction, obstacles, floor):
            """Joints aiming the camera, if the gripper keeps clear there; else None."""
            joints, error, cosine = self.arm.aim(camera, direction, self.target)
            if error > 0.005 or cosine < 0.98:
                return None
            if clearance(self.arm, np.r_[joints, OPEN], obstacles, floor) < MARGIN:
                return None
            return joints

        def reachable(self, point):
            """Whether the tool reaches down to a point with the fingers pointing down, as
            every grasp needs."""
            q, _ = self.arm.ik(point, self.target)
            t = self.arm.fk(q)
            return np.linalg.norm((t @ TCP)[:3] - point) < 0.005 and t[2, 2] > FINGERS_DOWN

        def detect(self, view, rgb, confidence=0.4):
            return sightings(view, self.detector(rgb, confidence))

        def script(self):
            """Look around, handle one piece, and look again, until every piece is on its
            square: after each change the scene is seen anew rather than remembered."""
            home, _ = self.arm.ik(HOME)
            yield self.joint_move(home[:5], OPEN, 3.0)
            survey = None
            # The squares this run has filled, and with what; and where handling a piece
            # failed, so that what is not a piece, or cannot be handled, is given up on.
            filled, self.failed = {}, []
            for _ in range(ROUNDS):
                survey, found = yield from self.look_around(survey)
                board = survey.board
                found = [(m, g) for m, g in found if board.square(m[:2]) not in filled]
                found = [(m, g) for m, g in found if sum(np.linalg.norm(f - m[:2]) < board.s for f in self.failed) < ATTEMPTS]
                if not found:
                    self.get_logger().info(f"{self.seconds():.0f} s: every piece is in place")
                    return
                acted = yield from self.take_turn(survey, found, filled)
                if not acted:
                    self.get_logger().error("No piece left that can be handled; holding")
                    return

        def look_around(self, survey):
            """Views over the board and the desk: the board, measured in the first round only,
            and the pieces, standing ones detected, lying ones segmented, most seen first."""
            home, _ = self.arm.ik(HOME, self.target)
            views, detections, masks = [], [], []
            poses = [aim_point(*v) for v in SURVEY] + [
                aim_point(elevation, azimuth, SIDE_DISTANCE, r * np.array([math.cos(math.radians(bearing)), math.sin(math.radians(bearing)), 0.0]))
                for r, bearing, elevation, azimuth in SIDES
            ]
            for i, (camera, direction) in enumerate(poses):
                joints = self.look_from(camera, direction, [], self.arm.floor + SURVEY_CLEARANCE - MARGIN)
                if joints is None:
                    self.get_logger().warning(f"Survey view {i} is out of reach")
                    continue
                yield self.joint_move(joints, OPEN, 2.0)
                yield Look()
                view = self.view()
                if i < len(SURVEY):
                    views.append(view)
                # Each view is searched while the arm moves on to the next, liberally: every
                # candidate is checked again from closer.
                detections.append(self.worker.submit(self.detect, view, self.rgb, SURVEY_CONFIDENCE))
                # The detector knows pieces from the side, standing; lying ones, from above and
                # far, it barely sees. Every object in the overview and the views of the desk is
                # segmented too, to find those.
                if i == 0 or i >= len(SURVEY):
                    masks.append((view, self.worker.submit(self.segmenter.everything, self.rgb, GRID_SPACING, view.own)))
            yield self.joint_move(home[:5], OPEN, 2.5)
            if survey is None:
                work = Work(Survey, views, self.arm.floor)
                yield work
                survey = work.value
            board = survey.board
            # No two pieces share a square.
            found = pieces([s for d in detections for s in d.result()], survey, board.s)
            # Lying pieces the detector did not see, most often segmented first.
            shapes = [
                s for view, m in masks for s in piece_shapes(view, m.result(), survey, PIECE_LENGTH, PIECE_WIDTH)
                if s.lying and s.length > LYING_LENGTH * board.s
            ]
            flat = []
            for shape in shapes:
                if any(np.linalg.norm(shape.center - middle[:2]) < board.s for middle, _ in found):
                    continue
                for other in flat:
                    if np.linalg.norm(shape.center - other[0].center) < board.s:
                        other[1] += 1
                        break
                else:
                    flat.append([shape, 1])
            for shape, count in sorted(flat, key=lambda f: -f[1]):
                found.append((np.r_[shape.center, survey.support(shape.center) + shape.width / 2], []))
            self.get_logger().info(
                f"{self.seconds():.0f} s: board: {board.s * 1000:.1f} mm squares, top {board.z * 1000:.1f} mm, "
                f"centre {np.round(board.origin, 3).tolist()}, yaw {math.degrees(board.yaw):.1f} deg; "
                f"{len(found)} candidates, {len(flat)} of them lying and segmented"
            )
            return survey, found

        def take_turn(self, survey, found, filled):
            """Deal with one piece: stand it up if it lies, else set it on its square. Returns
            whether anything was done; the scene is to be looked at again after."""
            board = survey.board
            for middle, group in found:
                if not self.reachable(middle):
                    continue
                # Confirmed by several views agreeing, or from closer: from above if it lies,
                # from the side if it stands.
                shape = yield from self.look_down(survey, middle, group)
                lying = shape is not None and shape.lying and shape.length > LYING_LENGTH * board.s
                if shape is not None and np.linalg.norm(shape.center - middle[:2]) > board.s:
                    # It found a piece somewhere else: that one is handled, seen afresh.
                    middle, group = np.r_[shape.center, survey.support(shape.center) + shape.width / 2], []
                if lying:
                    if (yield from self.right(survey, shape)) is None:
                        self.failed += [middle[:2], shape.center]
                    return True
                piece = yield from self.inspect(survey, middle, group)
                if piece is None:
                    self.failed.append(middle[:2])
                    continue
                # Seen from above standing, its base is where the widest round part of its
                # outline is: side views see a knight's overhanging head in its middle.
                if shape is not None and not shape.lying and np.linalg.norm(shape.foot - piece[0][:2]) < board.s:
                    piece = (np.r_[shape.foot, piece[0][2]], piece[1])
                # Squares other pieces stand on, to leave free.
                occupied = {board.square(m[:2]) for m, _ in found if np.linalg.norm(m[:2] - piece[0][:2]) > board.s / 2}
                free = {q for q in board.squares() if q not in filled and q not in occupied}
                held = yield from self.grasp(survey, *piece, allowed={k for k, qs in SQUARES.items() if free & set(qs)})
                if held is None:
                    self.failed.append(piece[0][:2])
                    continue
                if held:
                    square = yield from self.place(survey, *held, free, heading=None if shape is None or shape.lying else shape.facing)
                    if square is not None:
                        filled[square] = held[0]
                else:
                    self.failed.append(piece[0][:2])
                return True
            return False

        def across(self, xy, z, direction):
            """The wrist rolls that close the jaws across `direction` over xy at height z: two,
            half a turn apart, where the joint's range allows."""
            direction = np.r_[direction[:2], 0.0] / np.linalg.norm(direction[:2])
            rolls = np.linspace(*self.arm.limits[4], 145)
            along = []
            for roll in rolls:
                q, _ = self.arm.ik((*xy, z), self.target, roll=roll)
                along.append(self.arm.fk(q)[:3, 0] @ direction)
            # Where the jaw axis turns across the direction: the sign changes between samples.
            found = [rolls[i] - along[i] * (rolls[i + 1] - rolls[i]) / (along[i + 1] - along[i])
                     for i in range(len(rolls) - 1) if along[i] * along[i + 1] <= 0 and along[i] != along[i + 1]]
            return sorted(found, key=abs)

        def outward(self, xy):
            """The horizontal direction away from the robot through xy, in the plane the arm
            turns in: from the shoulder's pan axis, not the base's origin."""
            d = np.asarray(xy[:2], float) - self.pan_axis[:2]
            return np.r_[d / np.linalg.norm(d), 0.0]

        def right(self, survey, shape):
            """Stand a lying piece up. The wrist pitches about the jaw axis only when the jaws
            close across the arm's plane, so the piece is first laid pointing away from the
            robot, base inwards, where the arm can pitch far; then it is gripped near its head
            and turned up about its base's edge, the gripper turning with it, until the arm
            can pitch no further. Tilted less than the angle its base spans from its centre of
            mass, a weighted piece then falls upright when let go of. Returns where it stands,
            or None."""
            work = Work(self.pivot_spot, survey, shape)
            yield work
            spot = work.value
            if spot is None:
                self.get_logger().warning("No spot in reach to stand the piece up at")
                return None
            centre, pitch, share, side = spot
            placed = yield from self.lay(survey, shape, centre)
            if not placed:
                return None
            # It may settle a little off where it was let go of: looked at again from above.
            laid = yield from self.look_down(survey, np.r_[centre, survey.support(centre) + shape.width / 2])
            if laid is None:
                self.get_logger().warning("Lost the piece after laying it out")
                return None
            if not laid.lying:
                return laid.center
            return (yield from self.pivot(survey, laid, share, side))

        def pivot_plan(self, survey, base, toward_head, reach, radius, grip_frame, x_axis, seed, pitch):
            """Joints turning a grip at `reach` along a lying piece, its base's edge at `base`,
            up about that edge by up to `pitch`, the gripper turning with it; None out of
            reach."""
            axis = np.cross(toward_head, [0, 0, 1.0])
            grip = base + toward_head * reach + np.array([0, 0, radius])
            points = []
            for angle in np.linspace(0, pitch, 8):
                turn = Rotation.from_rotvec(axis * angle).as_matrix()
                target = base + turn @ (grip - base)
                q, error, off = self.arm.pose(grip_frame, target, turn @ np.array([0, 0, 1.0]), x_axis, seed)
                if error > POSE_TOLERANCE or off > math.radians(3):
                    return None
                # Turning, the fingers must not dig into the surface; at the grasp they sit just
                # over it by design.
                if angle > 0 and clearance(self.arm, np.r_[q, CLOSED], [], base[2]) < -POSE_TOLERANCE:
                    return None
                points.append(q)
                seed = q
            return points

        def pivot_spot(self, survey, shape):
            """Where to lay a piece out, pointing away from the robot, so that it can be turned
            up far enough to right itself: the nearest spot from which the arm pitches it
            furthest, and that pitch."""
            board = survey.board
            grip_frame = np.array([0.0, 0.0, TCP[2] - PAD_DEPTH + shape.width / 2])
            spots = [
                distance * np.array([math.cos(bearing), math.sin(bearing)])
                for bearing in np.radians(np.arange(-60, 61, 15)) for distance in (0.25, 0.23, 0.21)
            ]
            # On the desk, the whole piece and the gripper around it clear of the board.
            board = survey.board
            spots = [
                centre for centre in spots
                if max(np.abs(board.coordinates(centre))) > 4 * board.s + BOARD_FRAME + shape.length / 2 + MARGIN * 2
            ]
            spots.sort(key=lambda centre: np.linalg.norm(centre - shape.center))
            for pitch in PIVOT_PITCHES:
                for grips in PIVOT_GRIPS:
                    share = shape.thickest(*grips)
                    for centre in spots:
                        out = self.outward(centre)
                        base = np.r_[centre - out[:2] * shape.length / 2, survey.support(centre)]
                        # The jaws close across the piece from either side; from one of them the
                        # pitched gripper's body swings down into the surface.
                        for side in (1, -1):
                            x = side * np.array([-out[1], out[0], 0.0])
                            if self.pivot_plan(survey, base, out, share * shape.length, shape.width / 2, grip_frame, x, self.target, pitch):
                                self.get_logger().info(
                                    f"Laying it out at {np.round(centre, 3).tolist()} to turn it up {math.degrees(pitch):.0f} deg, "
                                    f"gripped {share:.2f} of its length from the base"
                                )
                                return centre, pitch, share, side
            return None

        def lay(self, survey, shape, centre):
            """Pick a lying piece up by its middle, a little, and lay it down at `centre`
            pointing away from the robot, base inwards. Returns whether it was laid there."""
            support = survey.support(shape.center)
            local = shape.widths[10] / 2
            beside = self.arm.pads(OPEN, TCP[2] - PAD_DEPTH, TCP[2])[0] + local + GRIP_GAP
            tool_height = support + PAD_CLEARANCE + PAD_DEPTH
            above = support + shape.width + MARGIN + PAD_DEPTH
            head = -shape.axis
            for roll in self.across(shape.center, above, shape.axis):
                try:
                    approach = Move(*path(self.arm, self.target, (*shape.center, above), OPEN, roll, 2.0))
                    tool = grasp_tool(self.arm, shape.center, approach.points[-1], beside)
                    align = Move(*path(self.arm, approach.points[-1], (*tool, above), OPEN, roll))
                    descend = Move(*path(self.arm, align.points[-1], (*tool, tool_height), OPEN, roll, 2.0))
                except ValueError:
                    continue
                break
            else:
                self.get_logger().warning("Cannot reach over the lying piece")
                return False
            self.get_logger().info(
                f"{self.seconds():.0f} s: gripping the lying piece at {np.round(shape.center, 3).tolist()}, "
                f"axis {np.round(shape.axis, 2).tolist()}, {shape.length * 1000:.0f} by {shape.width * 1000:.0f} mm"
            )
            yield approach
            yield align
            yield descend
            yield self.grip_move(CLOSED)
            yield self.grip_move(CLOSED, 1.0)
            if self.q[5] < HELD_JAW:
                self.get_logger().warning(f"Missed the lying piece (jaw {self.q[5]:.3f})")
                yield self.grip_move(OPEN)
                return False
            # The piece's middle is now midway between the pads; it turns with the gripper.
            held = np.r_[between_pads(self.arm, self.q[5]), 0.0, 0.0]
            grasped = self.yaw(self.target)
            out = self.outward(centre)
            turn = math.atan2(out[1], out[0]) - math.atan2(head[1], head[0])
            new_support = survey.support(centre)
            carry = max(support, new_support) + shape.width + MARGIN + PAD_DEPTH
            put = new_support + PAD_CLEARANCE + PAD_DEPTH
            roll = self.roll_for(centre, carry, grasped + turn)
            try:
                up = Move(*path(self.arm, self.target, (*(self.arm.fk(self.target) @ TCP)[:2], carry), CLOSED, self.target[4]))
                over = Move(*path(self.arm, up.points[-1], (*centre, carry), CLOSED, roll, 2.0))
                tool = grasp_tool(self.arm, centre, over.points[-1], held[0])
                over = Move(*path(self.arm, up.points[-1], (*tool, carry), CLOSED, roll, 2.0))
                down = Move(*path(self.arm, over.points[-1], (*tool, put), CLOSED, roll, 2.0))
                opened = np.r_[down.points[-1][:5], OPEN]
                leave = Move(*path(self.arm, opened, (*tool, carry + 0.02), OPEN, roll))
            except ValueError:
                self.get_logger().warning("Cannot carry the lying piece out")
                return False
            yield up
            yield over
            yield down
            yield self.grip_move(OPEN)
            yield leave
            self.get_logger().info(f"{self.seconds():.0f} s: laid it out at {np.round(centre, 3).tolist()}")
            return True

        def pivot(self, survey, shape, share, side):
            """Turn a piece lying pointing away from the robot, base inwards, up about its
            base's edge as far as the arm reaches from where it now lies, gripped near its head
            and the gripper turning with it, and let go of it. Returns where it stands."""
            support = survey.support(shape.center)
            # The arm pitches only about the direction across its plane: the jaws close along
            # that, and the piece turns in the plane, even lying a few degrees off it.
            out = self.outward(shape.center)
            reach = share * shape.length
            local = shape.widths[int(reach / shape.length * 20)] / 2
            radius = shape.width / 2
            base = np.r_[shape.base, support]
            grip = base + out * reach + np.array([0, 0, radius])
            beside = self.arm.pads(OPEN, TCP[2] - PAD_DEPTH, TCP[2])[0] + local + GRIP_GAP
            tool_height = support + PAD_CLEARANCE + PAD_DEPTH
            above = support + shape.width + MARGIN + PAD_DEPTH
            # The roll that closes the jaws from the planned side.
            x = side * np.array([-out[1], out[0], 0.0])
            rolls = [r for r in self.across(grip[:2], above, out) if self.arm.fk(self.arm.ik((*grip[:2], above), self.target, roll=r)[0])[:3, 0] @ x > 0]
            for roll in rolls:
                try:
                    approach = Move(*path(self.arm, self.target, (*grip[:2], above), OPEN, roll, 2.0))
                    tool = grasp_tool(self.arm, grip[:2], approach.points[-1], beside)
                    align = Move(*path(self.arm, approach.points[-1], (*tool, above), OPEN, roll))
                    descend = Move(*path(self.arm, align.points[-1], (*tool, tool_height), OPEN, roll, 2.0))
                except ValueError:
                    continue
                break
            else:
                self.get_logger().warning("Cannot reach over the laid out piece")
                return None
            self.get_logger().info(f"{self.seconds():.0f} s: gripping the laid out piece at {np.round(grip[:2], 3).tolist()}")
            yield approach
            yield align
            yield descend
            yield self.grip_move(CLOSED)
            yield self.grip_move(CLOSED, 1.0)
            if self.q[5] < HELD_JAW:
                self.get_logger().warning(f"Missed the laid out piece (jaw {self.q[5]:.3f})")
                yield self.grip_move(OPEN)
                return None
            # Where the grip holds it now, and so where its base's edge is.
            frame = self.arm.fk(self.target)
            grip_frame = np.array([between_pads(self.arm, self.q[5]), 0.0, TCP[2] - PAD_DEPTH + radius - PAD_CLEARANCE])
            grip = (frame @ np.r_[grip_frame, 1.0])[:3]
            base = np.r_[grip[:2] - out[:2] * reach, support]
            points = None
            for pitch in PIVOT_PITCHES + (PIVOT_LEAST,):
                points = self.pivot_plan(survey, base, out, reach, radius, grip_frame, frame[:3, 0], self.target, pitch)
                if points is not None:
                    break
            if points is None:
                self.get_logger().warning("The turn left reach once the piece was gripped")
                yield self.grip_move(OPEN)
                return None
            self.get_logger().info(f"{self.seconds():.0f} s: turning it up {math.degrees(pitch):.0f} deg")
            yield Move([np.r_[q, CLOSED] for q in points], 4.0)
            yield self.grip_move(OPEN)
            # Back out along the fingers, then straight up, then home around the piece.
            standing = base[:2] + out[:2] * radius
            end = self.arm.fk(self.target)
            back = (end @ TCP)[:3] + end[:3, 2] * RETREAT
            for target in (back, back + np.array([0, 0, 2 * RETREAT])):
                q, _, _ = self.arm.pose(TCP[:3], target, end[:3, 2], end[:3, 0], self.target)
                yield self.joint_move(q, OPEN, 1.0)
            obstacles = [(standing, 0.8 * survey.board.s, support + shape.length + MARGIN)]
            home, _ = self.arm.ik(HOME, self.target)
            yield from self.clear_move(home[:5], obstacles, self.surface(survey))
            self.get_logger().info(f"{self.seconds():.0f} s: turned it up at {np.round(standing, 3).tolist()}")
            return standing

        def look_down(self, survey, middle, group=()):
            """A look straight down at a piece: its outline from above, or None."""
            board = survey.board
            support = survey.support(middle[:2])
            tall = height(group, support, middle[:2]) if group else None
            top = support + (tall if tall is not None else 2.5 * (middle[2] - support))
            obstacles = [(middle[:2], 0.8 * board.s, top + MARGIN)]
            for rise in DOWN_HEIGHTS:
                camera = np.r_[middle[:2], support + rise]
                joints = self.look_from(camera, (0, 0, -1), obstacles, self.surface(survey))
                if joints is None:
                    continue
                yield from self.clear_move(joints, obstacles, self.surface(survey))
                yield Look()
                view, rgb = self.view(), self.rgb
                # The objects around where the piece is expected, prompted a quarter of a
                # square apart, so that even a small piece far below is; and of those the
                # piece-sized one nearest.
                h, w = view.own.shape
                uv, depth = view.project(np.array([[*middle[:2], support]]))
                (u, v), pixels = uv[0], board.s * view.f / depth[0]
                rows, cols = np.mgrid[:h, :w]
                around = (cols - u) ** 2 + (rows - v) ** 2 < (DOWN_SEARCH * pixels) ** 2
                spacing = max(1, round(pixels / 4))
                work = Work(lambda: piece_shapes(view, self.segmenter.everything(rgb, spacing, view.own | ~around), survey, PIECE_LENGTH, PIECE_WIDTH))
                yield work
                # The piece nearest where it was expected: the camera looks straight down at
                # the estimate, and whichever piece that finds is handled.
                near = work.value
                # The heights are for reach and clearance: one look that sees nothing there
                # is enough.
                if not near:
                    self.get_logger().info(f"{self.seconds():.0f} s: no piece from above at {np.round(middle[:2], 3).tolist()}")
                    return None
                shape = min(near, key=lambda s: np.linalg.norm(s.center - middle[:2]))
                self.get_logger().info(
                    f"{self.seconds():.0f} s: from above: {shape.length * 1000:.0f} by {shape.width * 1000:.0f} mm, "
                    f"base {np.round(shape.base, 3).tolist()}, {'lying' if shape.lying else 'standing'}"
                )
                return shape
            reasons = []
            for rise in DOWN_HEIGHTS:
                joints, error, cosine = self.arm.aim(np.r_[middle[:2], support + rise], (0, 0, -1), self.target)
                gap = clearance(self.arm, np.r_[joints, OPEN], obstacles, self.surface(survey))
                reasons.append(f"{rise * 100:.0f} cm: {error * 1000:.1f} mm off, cos {cosine:.3f}, {gap * 1000:.0f} mm clear")
            self.get_logger().info(
                f"No look from above at {np.round(middle[:2], 3).tolist()} (top {top * 1000:.0f} mm): " + "; ".join(reasons)
            )
            return None

        def inspect(self, survey, middle, group, search=1.0):
            """Closer looks from around the robot's side of a piece: where exactly it stands.
            Returns its middle and sightings, or None if none of the looks finds it."""
            board = survey.board
            support = survey.support(middle[:2])
            obstacles = [(middle[:2], 0.8 * board.s, support + 2.5 * (middle[2] - support) + 0.01)]
            close, looks = [], 0
            towards = math.atan2(-middle[1], -middle[0])
            for d, e, turn in CLOSE_LOOKS:
                a = towards + turn
                camera = middle + d * np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])
                joints = self.look_from(camera, middle - camera, obstacles, self.surface(survey))
                if joints is None:
                    continue
                yield from self.clear_move(joints, obstacles, self.surface(survey))
                yield Look()
                view, rgb = self.view(), self.rgb
                work = Work(self.detect, view, rgb)
                yield work
                # The nearest detection within a square of the estimate, which each refines.
                near = sorted((np.linalg.norm(ground(s, middle[2])[:2] - middle[:2]), i) for i, s in enumerate(work.value))
                near = [work.value[i] for d, i in near if d < search * board.s]
                close += near[:1]
                if near:
                    middle = position(close if len(close) >= 2 else group + close, survey)
                self.get_logger().info(f"{self.seconds():.0f} s: close look {len(close)}: {len(work.value)} detections, {len(near)} of the piece")
                self.show(view, [(middle, "piece")])
                looks += 1
                # Three sightings fix it; two looks that find nothing dismiss it.
                if len(close) == 3 or (looks == 2 and not close):
                    break
            if not close:
                self.get_logger().info(f"Nothing at {np.round(middle[:2], 3).tolist()} from closer")
                return None
            return middle, group + close

        def grasp(self, survey, middle, group, allowed=None):
            """Grasp a piece above its middle, the height its sightings met at, and lift it.
            Returns what `place` needs if it is held, False if it slipped, or None if it is
            out of reach."""
            support = survey.support(middle[:2])
            tall = height(group, support, middle[:2])
            kind = vote(group, allowed)
            self.get_logger().info(
                "Sightings: " + ", ".join(f"{s.kind} {s.score:.2f} at {math.degrees(math.asin(-s.ray[2])):.0f} deg" for s in group)
                + ("" if tall is None else f"; {tall * 1000:.0f} mm tall")
            )
            top = support + (tall if tall is not None else 2 * (middle[2] - support))
            self.get_logger().info(
                f"{self.seconds():.0f} s: a {kind} at {np.round(middle[:2], 3).tolist()}, "
                f"about {(top - support) * 1000:.0f} mm tall"
            )
            # The pads grip its upper half, above its low centre of mass, so it hangs from them.
            grip = support + (top - support) / 2 + PAD_DEPTH
            # From the highest point in reach straight above it, at most a margin over its top;
            # close to the robot the arm reaches only lower, where the open jaws straddle it.
            for above in np.arange(top + MARGIN + PAD_DEPTH, grip, -0.005):
                try:
                    # The whole path is planned before the arm moves.
                    approach = Move(*path(self.arm, self.target, (*middle[:2], above), OPEN, 0.0, 2.0))
                    # The piece's axis midway between the open pads, so closing moves it least.
                    tool = grasp_tool(self.arm, middle[:2], approach.points[-1], between_pads(self.arm, OPEN))
                    align = Move(*path(self.arm, approach.points[-1], (*tool, above), OPEN, 0.0))
                    descend = Move(*path(self.arm, align.points[-1], (*tool, grip), OPEN, 0.0, 2.0))
                    closed = np.r_[descend.points[-1][:5], CLOSED]
                    lift = Move(*path(self.arm, closed, (*tool, above), CLOSED, 0.0, 2.0))
                    break
                except ValueError:
                    continue
            else:
                self.get_logger().warning(f"Cannot reach over the {kind}")
                return None
            yield approach
            yield align
            yield descend
            yield self.grip_move(CLOSED)
            yield self.grip_move(CLOSED, 1.0)
            yield lift
            if self.q[5] < HELD_JAW:
                self.get_logger().warning(f"Missed the {kind} (jaw {self.q[5]:.3f})")
                return False
            self.get_logger().info(f"Holding the {kind}")
            # What it is, how far above its base the tool holds it, where its axis is now (the
            # moving jaw pushed it against the fixed one), and which way the gripper faced.
            return kind, grip - support, between_pads(self.arm, self.q[5]), self.yaw(self.target)

        def yaw(self, q):
            """Which way the jaws close, as an angle about the vertical."""
            x = self.arm.fk(q)[:3, 0]
            return math.atan2(x[1], x[0])

        def roll_for(self, xy, z, yaw):
            """The wrist roll that turns the jaws to `yaw` over xy at height z."""
            best = None
            for roll in np.linspace(*self.arm.limits[4], 145):
                q, _ = self.arm.ik((*xy, z), self.target, roll=roll)
                off = abs(math.remainder(self.yaw(np.r_[q, 0.0]) - yaw, 2 * math.pi))
                if best is None or off < best[0]:
                    best = (off, roll)
            return best[1]

        def place(self, survey, kind, hold, axis, grasped, free, heading=None):
            """Set a held piece on the nearest `free` square of its kind and let go of it. A
            knight's `heading`, seen when the gripper faced `grasped`, is turned towards rank 1:
            it turns with the gripper. Returns the square, or None."""
            board = survey.board
            start = (self.arm.fk(self.target) @ TCP)[:2]
            roll = self.target[4]
            release = board.z + hold + RELEASE_GAP
            nearest = sorted((q for q in SQUARES[kind] if q in free), key=lambda q: np.linalg.norm(board.center(q) - start))
            for square, clearance in ((q, c) for q in nearest for c in CARRY_CLEARANCES):
                target = board.center(square)
                # Its base clears the board on the way.
                carry = board.z + hold + clearance
                if kind == "knight" and heading is not None:
                    turn = math.atan2(heading[1], heading[0]) - grasped
                    roll = self.roll_for(board.center(square), carry, math.atan2(board.v[1], board.v[0]) - turn)
                try:
                    # The whole path is planned before the arm moves. The pads' offset from
                    # the piece's axis turns with the arm, so it is aimed from over the square.
                    up = Move(*path(self.arm, self.target, (*start, carry), CLOSED, self.target[4]))
                    over = Move(*path(self.arm, up.points[-1], (*target, carry), CLOSED, roll, 2.0))
                    tool = grasp_tool(self.arm, target, over.points[-1], axis)
                    over = Move(*path(self.arm, up.points[-1], (*tool, carry), CLOSED, roll, 2.0))
                    down = Move(*path(self.arm, over.points[-1], (*tool, release), CLOSED, roll, 2.0))
                    opened = np.r_[down.points[-1][:5], OPEN]
                    leave = Move(*path(self.arm, opened, (*tool, carry), OPEN, roll, 1.5))
                    break
                except ValueError:
                    continue
            else:
                self.get_logger().error(f"No free square for the {kind} in reach; holding it")
                return None
            self.get_logger().info(f"{self.seconds():.0f} s: setting the {kind} on {square}")
            yield up
            yield over
            yield down
            yield self.grip_move(OPEN)
            yield leave
            home, _ = self.arm.ik(HOME, self.target)
            yield self.joint_move(home[:5], OPEN, 2.0)
            self.get_logger().info(f"{self.seconds():.0f} s: the {kind} is on {square}")
            return square

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
