"""SO101 chess controller from wrist images and joint positions alone.

The policy is the plain script in `Controller.script`, which runs in a thread of its own and
hands the node one action at a time; the node's timer publishes joint-position targets at a
steady rate and tells the script when the action is done. However long the script plans or
measures, the targets keep flowing: a servo whose commands stop lets go.

Survey the desk, then examine every piece found: a lying one is stood up, and each standing
one is looked at from closer. The kinds of all of them are decided together, as a set holds
each kind a known number of times; then each is grasped at its middle and set on its home
square. The desk is surveyed again for what is left; when every piece is in place, or none is
left it can handle, the node exits, which ends the episode.
"""

import math
import os
import signal
import threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from kinematics import TCP, Arm
from motion import CLOSED, OPEN, PAD_DEPTH, between_pads, clearance, duration, grasp_tool, path
from detector import KINDS, Detector
from segment import Segmenter
from vision import (
    UP, Sighting, Survey, View, assign, carve, ground, height, knight_axis, piece_shapes, pieces, position, side_look,
    sightings, snout_side,
)

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
# seen from about 55 degrees, as in the photos it learned from: there it tells kinds apart
# best, so those looks come first.
CLOSE_LOOKS = [
    (d, math.radians(e), turn) for e in (55, 40, 75) for d in (0.15, 0.12) for turn in (0.0, 0.9, -0.9, 1.8, -1.8)
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
# How far the fingertips move at most between two poses checked for collisions on a move.
SWEEP_STEP = 0.005
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
# How far from a piece's base seen from above a close look's detection may be, in squares.
CLOSE_SEARCH = 0.5
# A knight's profile is looked at from this far and this high, square to its axis; a turn of
# its axis by more than the angle after a closer look calls for a look square to the new one.
PROFILE_DISTANCE = 0.15
PROFILE_ELEVATION = math.radians(40)
PROFILE_TURN = math.radians(25)
# The elevation a piece that looks lying from above is looked at from the side to tell.
SIDE_CHECK_ELEVATION = math.radians(55)
# Pixels between the points the overview and desk views are segmented at.
GRID_SPACING = 40
# The least cosine between the fingers and straight down that counts as fingers down.
FINGERS_DOWN = 0.98
# Attempts at a piece before it is given up on.
ATTEMPTS = 2
# How often every piece found is handled, at most, and how often the desk is surveyed for
# pieces not yet found before their kinds are decided.
ROUNDS = 6
SURVEYS = 2
# Black's home squares, rank 8 nearest the robot.
SQUARES = {
    "rook": ("a8", "h8"), "knight": ("b8", "g8"), "bishop": ("c8", "f8"),
    "queen": ("d8",), "king": ("e8",), "pawn": tuple(f"{f}7" for f in "abcdefgh"),
}
# How many pieces of each kind a set has: one for each home square.
COUNTS = {kind: len(squares) for kind, squares in SQUARES.items()}
# The order kinds are set up in: the back rank from its middle out, then the pawns. Next to the
# robot the arm reaches only low, so each piece comes in over the squares beside and in front
# of its own while they are still empty.
ORDER = ("queen", "king", "bishop", "knight", "rook", "pawn")
# A carried piece's base keeps air over the pieces standing on the board, at most two squares
# tall, or else this much over the board, only where the arm cannot rise higher, close to
# its base; it is let go of this far above the board.
CARRY_CLEARANCES = (0.02, 0.01)
RELEASE_GAP = 0.002
# What the fingers keep clear of around a standing piece, in squares: a cylinder as wide as
# the widest base and as tall as a king, on any set. Squares are barely wider than a base,
# and the fingers reach most of a square out along the jaws, so the wrist turns them where
# the neighbours leave most room, out of this many rolls.
PIECE_RADIUS = 0.4
PIECE_TOP = 2.0
# Within this many squares of a filled square's centre, what is seen is the piece set there.
PLACED_NEAR = 0.75
ROLLS = 25
# How much wider than a piece the pads open around it to grasp it, and how much further they
# open to let go of it: no wider, to keep off the neighbours.
GRASP_ROOM = 0.008
RELEASE_OPENING = 0.006
# Where the arm reaches high, all round its base: a held piece is carried round on this arc,
# from the shoulder's pan axis, when the straight way passes the base low.
CARRY_RADIUS = 0.2
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


class Piece:
    """A standing piece as examined: its middle, its sightings, its outline from above if
    seen, and how well each kind fits it, from the close looks."""

    def __init__(self, middle, group, shape, evidence):
        self.middle = middle
        self.group = group
        self.shape = shape
        self.evidence = evidence


class Work:
    """A function the script's thread runs while the arm holds still; its result is `value`."""

    def __init__(self, function, *args):
        self.function = function
        self.args = args
        self.value = None


def aim_point(elevation, azimuth, distance, center=CENTER):
    e, a = math.radians(elevation), math.radians(azimuth)
    camera = center + distance * np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])
    return camera, center - camera


def main():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import CompressedImage, Image, JointState
    from std_msgs.msg import String

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
            # The action the script waits on, and the script's thread, set when it is done.
            self.action = None
            self.done = threading.Event()
            self.task = None
            self.failure = None
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
                self.task = threading.Thread(target=self.play, daemon=True)
                self.task.start()
            elif self.failure is not None:
                raise self.failure
            elif self.ended is not None:
                # Done or given up, nothing changes any more: exiting ends the episode, once
                # the evaluator has seen the scene at rest.
                if now - self.ended > SETTLE_TIME:
                    raise SystemExit
            action = self.action
            if isinstance(action, Move):
                self.target = action.target(now)
            if action is not None and self.finished(action, now):
                self.action = None
                self.done.set()
            msg = JointState()
            msg.name = self.arm.names
            msg.position = self.target.tolist()
            self.commands.publish(msg)

        def play(self):
            """Run the script in this thread: each move or look it yields is handed to the
            timer, which says when it is done; each piece of work is done here."""
            # Debug: wall seconds spent per kind of work and planning between actions.
            import time
            spent, mark = {}, time.perf_counter()
            try:
                for action in self.script():
                    now = time.perf_counter()
                    spent["planning"] = spent.get("planning", 0.0) + now - mark
                    if isinstance(action, Work):
                        action.value = action.function(*action.args)
                        name = getattr(action.function, "__name__", "work")
                        spent[name] = spent.get(name, 0.0) + time.perf_counter() - now
                        mark = time.perf_counter()
                        continue
                    action.start = self.seconds()
                    self.done.clear()
                    self.action = action
                    self.done.wait()
                    kind = type(action).__name__
                    spent[kind] = spent.get(kind, 0.0) + time.perf_counter() - now
                    mark = time.perf_counter()
                self.ended = self.seconds()
            except Exception as error:
                self.failure = error
            finally:
                self.get_logger().info("Wall seconds: " + ", ".join(f"{k} {v:.0f}" for k, v in sorted(spent.items(), key=lambda kv: -kv[1])))

        def finished(self, action, now):
            if isinstance(action, Move):
                return action.done(now, self.q)
            # Wait again whenever the arm has moved since.
            if action.still is None or np.max(np.abs(self.q[:5] - action.still[:5])) > STILL:
                action.still = self.q.copy()
                action.start = now
                return False
            if self.image_time is not None and self.image_time > action.start:
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

        def joint_move(self, joints, grip, seconds=1.0):
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
            """Move to joints without the gripper passing through an obstacle or the surface on
            the way: straight, or by the home pose, high over everything. Returns whether there
            was such a way; the arm stays where it is if not."""
            obstacles = list(obstacles) + self.standing()
            home = np.r_[self.arm.ik(HOME, self.target)[0], OPEN]
            goal = np.r_[joints, OPEN]
            clear = lambda a, b: min(clearance(self.arm, q, obstacles, floor) for q in self.sweep(a, b)) >= MARGIN
            for way in ([goal], [home, goal]):
                if all(clear(a, b) for a, b in zip([self.target.copy(), *way], way)):
                    for q in way:
                        yield self.joint_move(q[:5], OPEN, 1.0)
                    return True
            return False

        def sweep(self, start, end):
            """Joint positions along a joint move, close enough together that the fingertips
            move at most a few millimetres between them."""
            tips = [(self.arm.fk(q) @ TCP)[:3] for q in (start, end)]
            # Joints turning sweep the fingertips along an arc longer than the chord between its
            # ends: the arm's reach times the largest turn bounds it.
            travel = max(np.linalg.norm(tips[1] - tips[0]), 0.35 * float(np.max(np.abs(end[:5] - start[:5]))))
            count = max(2, math.ceil(travel / SWEEP_STEP) + 1)
            return [start + f * (end - start) for f in np.linspace(0, 1, count)]

        def standing(self, away=None):
            """The pieces seen or set standing, as cylinders the gripper keeps clear of, but for
            any within a square of `away`."""
            if self.survey is None:
                return []
            board = self.survey.board
            cylinders = [(xy, PIECE_RADIUS * board.s, self.survey.support(xy) + PIECE_TOP * board.s) for xy in self.around] + self.placed
            return [c for c in cylinders if away is None or np.linalg.norm(c[0] - away) > board.s]

        def overhead(self, survey, xy, z, grip, roll, seconds=2.0):
            """Joint waypoints to over xy at height z, coming straight down from over the
            tallest piece there, as high as the arm reaches, and how long they take."""
            board = survey.board
            high = self.reach(xy, board.z + PIECE_TOP * board.s + MARGIN + PAD_DEPTH, z)
            points, seconds = path(self.arm, self.target, (*xy, high), grip, roll, seconds)
            down, more = path(self.arm, points[-1], (*xy, z), grip, roll)
            return points + down[1:], seconds + more

        def look_from(self, camera, direction, obstacles, floor):
            """Joints aiming the camera, if the gripper keeps clear there; else None."""
            joints, error, cosine = self.arm.aim(camera, direction, self.target)
            if error > 0.005 or cosine < 0.98:
                return None
            if clearance(self.arm, np.r_[joints, OPEN], list(obstacles) + self.standing(), floor) < MARGIN:
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
            """Look around, examine every piece found and set them on their squares, and
            look again for what is left, until every piece is in place."""
            home, _ = self.arm.ik(HOME)
            yield self.joint_move(home[:5], OPEN, 3.0)
            survey = None
            # The squares this run has filled, and with what; and where handling a piece
            # failed, so that what is not a piece, or cannot be handled, is given up on.
            filled, self.failed = {}, []
            # The pieces set on their squares, which nothing comes near again, and the others seen
            # standing, where they were: obstacles.
            self.placed, self.around, self.survey = [], [], None
            # Pieces examined and not placed yet, which keep what was seen of them.
            waiting, left = True, []
            for _ in range(ROUNDS):
                # Every piece still to place is examined before any kind is decided: the desk
                # is looked at again while fewer have been found than the set has left.
                known, found = list(left), []
                for _ in range(SURVEYS):
                    survey, found = yield from self.look_around(survey)
                    board = survey.board
                    self.survey, self.around = survey, [m[:2] for m, _ in found] + [p.middle[:2] for p in known]
                    # What is seen on or beside a filled square is the piece set there: left alone.
                    found = [
                        (m, g) for m, g in found
                        if all(np.linalg.norm(m[:2] - board.center(q)) > PLACED_NEAR * board.s for q in filled)
                        and not self.given_up(m[:2], board)
                    ]
                    fresh = [(m, g) for m, g in found if not any(np.linalg.norm(m[:2] - p.middle[:2]) < board.s for p in known)]
                    if not fresh:
                        break
                    known += yield from self.examine_all(survey, fresh, known)
                    if len(known) >= sum(COUNTS.values()) - len(filled):
                        break
                if not known:
                    if found:
                        self.get_logger().error("No piece left that can be handled; holding")
                    else:
                        self.get_logger().info(f"{self.seconds():.0f} s: every piece is in place")
                    return
                # Pawns wait for the back rank behind them, which is reached over their rank, until
                # a round sets none of it.
                back = sum(k != "pawn" for k in filled.values())
                left = yield from self.place_all(survey, known, found, filled, waiting)
                waiting = sum(k != "pawn" for k in filled.values()) > back

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
                yield self.joint_move(joints, OPEN, 1.0)
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
            yield self.joint_move(home[:5], OPEN, 1.5)
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

        def given_up(self, xy, board):
            """Whether handling a piece within a square of xy failed often enough to stop."""
            return sum(np.linalg.norm(f - xy) < board.s for f in self.failed) >= ATTEMPTS

        def examine_all(self, survey, found, known):
            """Examine every candidate in reach, and every piece seen beside one from above,
            each piece once, besides those `known` already: the standing pieces, lying ones
            stood up first."""
            board = survey.board
            known, examined = list(known), []
            queue = list(found)
            while queue:
                middle, group = queue.pop(0)
                if not self.reachable(middle) or any(np.linalg.norm(middle[:2] - p.middle[:2]) < board.s for p in known):
                    continue
                # Where the other pieces are, to lay a lying one out clear of them: those found,
                # and where handling one failed, which may have left it there.
                others = [p.middle[:2] for p in known] + [m[:2] for m, _ in queue] + self.failed
                piece, beside = yield from self.examine(survey, middle, group, known, others)
                if piece is not None:
                    known.append(piece)
                    examined.append(piece)
                    self.around.append(piece.middle[:2])
                # Pieces seen beside it that no candidate stands for, such as one a candidate
                # merged with its neighbour.
                for shape in beside:
                    points = [p.middle[:2] for p in known] + [m[:2] for m, _ in queue] + self.failed
                    if not any(np.linalg.norm(shape.center - q) < board.s / 2 for q in points):
                        queue.append((np.r_[shape.center, survey.support(shape.center) + shape.width / 2], []))
            return examined

        def examine(self, survey, middle, group, known, others):
            """Look at a candidate from above, stand it up if it lies, and look at it from
            closer: the standing piece, or None, and the other pieces seen around it from
            above."""
            board = survey.board
            # Confirmed by several views agreeing, or from closer: from above if it lies,
            # from the side if it stands.
            seen = yield from self.look_down(survey, middle, group)
            shape, beside = (seen[0], seen[1:]) if seen else (None, [])
            if shape is not None:
                if self.given_up(shape.center, board) or any(np.linalg.norm(shape.center - p.middle[:2]) < board.s for p in known):
                    return None, beside
                if np.linalg.norm(shape.center - middle[:2]) > board.s:
                    # It found a piece somewhere else: that one is handled, seen afresh.
                    middle, group = np.r_[shape.center, survey.support(shape.center) + shape.width / 2], []
            if shape is not None and shape.lying and shape.length > LYING_LENGTH * board.s and (yield from self.stands(survey, shape)):
                # A standing piece's outline with its shadow joined to it: where it is, its
                # close looks tell.
                shape = None
            if shape is not None and shape.lying and shape.length > LYING_LENGTH * board.s:
                standing = yield from self.right(survey, shape, others)
                if standing is None:
                    self.failed += [middle[:2], shape.center]
                    return None, beside
                middle, group = np.r_[standing, survey.support(standing) + shape.length / 2], []
                shape = ((yield from self.look_down(survey, middle)) or [None])[0]
                if shape is None or shape.lying:
                    self.get_logger().warning("The piece did not stay standing")
                    self.failed.append(middle[:2])
                    return None, beside
            # From above, a standing piece's base is found to within millimetres: the close looks
            # centre on it and take only what they see there, not a neighbour a square away.
            search = 1.0
            if shape is not None and not shape.lying:
                middle, search = np.r_[shape.foot, middle[2]], CLOSE_SEARCH
            looked = yield from self.inspect(survey, middle, group, search)
            if looked is None:
                self.failed.append(middle[:2])
                return None, beside
            middle, group, close = looked
            # Seen from above standing, its base is where the widest round part of its
            # outline is: side views see a knight's overhanging head in its middle.
            if shape is not None and not shape.lying and np.linalg.norm(shape.foot - middle[:2]) < board.s:
                middle = np.r_[shape.foot, middle[2]]
            evidence = np.mean([s.scores for s in close], 0)
            best = sorted(zip(evidence, KINDS), reverse=True)[:3]
            self.get_logger().info(
                f"{self.seconds():.0f} s: a piece at {np.round(middle[:2], 3).tolist()}: " + ", ".join(f"{k} {e:.2f}" for e, k in best)
            )
            return Piece(middle, group, shape, evidence), beside

        def stands(self, survey, shape):
            """Whether a piece whose outline from above looks lying stands after all: a shadow
            joined to a standing piece's outline lengthens it. Seen from the side across that
            outline, a standing piece's box is taller than wide, a lying one's wider, when the
            detector sees it at all."""
            board = survey.board
            support = survey.support(shape.center)
            middle = np.r_[shape.center, support + shape.width / 2]
            obstacles = [(middle[:2], 0.8 * board.s, support + shape.length + MARGIN)]
            across = math.atan2(shape.axis[1], shape.axis[0]) + math.pi / 2
            towards = math.atan2(-middle[1], -middle[0])
            # From the robot's side first, where the arm reaches more easily.
            sides = sorted((across, across + math.pi), key=lambda a: -math.cos(a - towards))
            e = SIDE_CHECK_ELEVATION
            for d, a in ((d, a) for d in (0.15, 0.12) for a in sides):
                camera = middle + d * np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])
                joints = self.look_from(camera, middle - camera, obstacles, self.surface(survey))
                if joints is None:
                    continue
                if not (yield from self.clear_move(joints, obstacles, self.surface(survey))):
                    continue
                yield Look()
                view, rgb = self.view(), self.rgb
                work = Work(self.detector, rgb, SURVEY_CONFIDENCE)
                yield work
                tall = [
                    found for found in work.value
                    if np.linalg.norm(ground(Sighting(view, found), middle[2])[:2] - middle[:2]) < board.s
                    and found.box[3] - found.box[1] > found.box[2] - found.box[0]
                ]
                self.get_logger().info(f"{self.seconds():.0f} s: from the side across it: {'standing' if tall else 'lying'}")
                return bool(tall)
            return False

        def place_all(self, survey, known, found, filled, waiting=False):
            """Decide the kinds of the examined pieces together, among those the filled
            squares leave, and set each on a square of its kind as squares come free; pawns,
            if `waiting`, only before back rank squares already filled. Returns the pieces
            left waiting for a square."""
            board = survey.board
            left = {k: n - sum(v == k for v in filled.values()) for k, n in COUNTS.items()}
            kinds = assign([p.evidence for p in known], left)
            self.get_logger().info("Kinds: " + ", ".join(f"{k} at {np.round(p.middle[:2], 3).tolist()}" for p, k in zip(known, kinds)))
            pending = sorted(((p, k) for p, k in zip(known, kinds) if k is not None), key=lambda pk: ORDER.index(pk[1]))
            # What stays where it is: candidates not examined, unless a closer look dismissed
            # them, and pieces of no kind left.
            near = lambda xy, points: any(np.linalg.norm(xy - q[:2]) < board.s for q in points)
            staying = [m[:2] for m, _ in found if not near(m[:2], [p.middle for p in known] + self.failed)]
            staying += [p.middle[:2] for p, k in zip(known, kinds) if k is None]
            if waiting:
                # Pawns standing on the ranks the back rank is reached over, which they would
                # wait on, are first put aside off the board.
                for piece, kind in pending:
                    square = board.square(piece.middle[:2])
                    if kind == "pawn" and square is not None and int(square[1]) >= 6:
                        others = staying + [p.middle[:2] for p, _ in pending if p is not piece] + [board.center(q) for q in filled]
                        yield from self.park(survey, piece, others)
            while pending:
                # The first piece with a free square of its kind; the squares other pieces
                # stand on are left free.
                for piece, kind in pending:
                    occupied = {board.square(q) for q in staying + [p.middle[:2] for p, _ in pending if p is not piece]}
                    free = {q for q in board.squares() if q not in filled and q not in occupied}
                    if waiting:
                        free = {q for q in free if not (q[1] == "7" and f"{q[0]}8" not in filled)}
                    if free & set(SQUARES[kind]):
                        break
                else:
                    self.get_logger().info(f"{len(pending)} pieces wait for a free square")
                    return [p for p, _ in pending]
                pending.remove((piece, kind))
                # The pieces standing elsewhere, which the fingers keep clear of.
                others = staying + [p.middle[:2] for p, _ in pending] + [board.center(q) for q in filled]
                shape = piece.shape
                width = shape.width if shape is not None else PIECE_WIDTH[1] * board.s
                facing = None if shape is None or shape.lying else shape.facing
                if kind == "knight" and facing is not None:
                    facing = yield from self.heading(survey, piece)
                held = yield from self.grasp(survey, piece.middle, piece.group, kind, width, others, free, facing)
                if not held:
                    self.failed.append(piece.middle[:2])
                    continue
                square = yield from self.place(survey, *held, free, piece.middle[:2], others, facing)
                if not square:
                    self.failed.append(piece.middle[:2])
                else:
                    filled[square] = kind
                    self.placed.append((board.center(square), PIECE_RADIUS * board.s, board.z + PIECE_TOP * board.s))
            return []

        def heading(self, survey, piece):
            """Which way a standing knight's head points: along its outline's axis from above,
            towards the side its profile shows the snout on, from a look square to that axis.
            A shadow joined to the outline from above is cut away by what the profile shows, and
            if that turns the axis, the profile is looked at again square to it. Falls back on
            the outline from above alone."""
            board, shape = survey.board, piece.shape
            support = survey.support(shape.foot)
            base = np.r_[shape.foot, support]
            tall = height(piece.group, support, shape.foot) or 2 * (piece.middle[2] - support)
            axis = knight_axis(shape.view, shape.mask, base, tall)
            obstacles = [(base[:2], 0.8 * board.s, support + tall + MARGIN)]
            for _ in range(2):
                camera = side_look(base, tall, axis, self.pan_axis, PROFILE_DISTANCE, PROFILE_ELEVATION)
                middle = base + tall / 2 * UP
                joints = self.look_from(camera, middle - camera, obstacles, self.surface(survey))
                if joints is None or not (yield from self.clear_move(joints, obstacles, self.surface(survey))):
                    break
                yield Look()
                view, rgb = self.view(), self.rgb
                prompts = view.project(np.array([base + 0.45 * tall * UP, base + 0.8 * tall * UP]))[0]
                work = Work(lambda: self.segmenter.decode_points(self.segmenter.embed(rgb), prompts, rgb.shape[:2]))
                yield work
                profile = work.value
                side = snout_side(view, profile, base, tall)
                carved = carve(shape.view, shape.mask, view, profile, base, tall)
                turned = knight_axis(shape.view, carved, base, tall) if carved.sum() > 20 else axis
                if abs(turned @ axis) < math.cos(PROFILE_TURN):
                    axis = turned
                    continue
                if not side.any():
                    break
                facing = axis if axis @ side >= 0 else -axis
                self.get_logger().info(f"{self.seconds():.0f} s: the knight faces {math.degrees(math.atan2(facing[1], facing[0])):.0f} deg")
                return facing
            self.get_logger().info("No clear profile of the knight; going by its outline from above")
            return shape.facing

        def park(self, survey, piece, others):
            """Put a standing piece aside on the desk beside the board, at the nearest free
            spot in reach, clear of the `others` pieces; the piece then stands there."""
            board = survey.board
            spots = [
                distance * np.array([math.cos(bearing), math.sin(bearing)])
                for bearing in np.radians(np.arange(-60, 61, 7.5)) for distance in (0.25, 0.22, 0.19, 0.16)
            ]
            spots = [
                c for c in spots
                if max(np.abs(board.coordinates(c))) > 5 * board.s + BOARD_FRAME
                and all(np.linalg.norm(c - q) > 1.5 * board.s for q in others)
                and self.reachable(np.r_[c, survey.support(c) + 0.02])
            ]
            if not spots:
                return
            spot = min(spots, key=lambda c: np.linalg.norm(c - piece.middle[:2]))
            width = piece.shape.width if piece.shape is not None else PIECE_WIDTH[1] * board.s
            held = yield from self.grasp(survey, piece.middle, piece.group, "pawn", width, others)
            if not held:
                return
            if (yield from self.place(survey, *held, set(), spot, others)) is None:
                self.get_logger().info(f"{self.seconds():.0f} s: put the piece aside at {np.round(spot, 3).tolist()}")
                lift = piece.middle[2] - survey.support(piece.middle[:2])
                # Its sightings were from where it stood; its height above what it stands on stays.
                piece.middle, piece.group = np.r_[spot, survey.support(spot) + lift], []

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

        def right(self, survey, shape, others):
            """Stand a lying piece up. The wrist pitches about the jaw axis only when the jaws
            close across the arm's plane, so the piece is first laid pointing away from the
            robot, base inwards, where the arm can pitch far; then it is gripped near its head
            and turned up about its base's edge, the gripper turning with it, until the arm
            can pitch no further. Tilted less than the angle its base spans from its centre of
            mass, a weighted piece then falls upright when let go of. Returns where it stands,
            or None."""
            work = Work(self.pivot_spot, survey, shape, others)
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
            laid = ((yield from self.look_down(survey, np.r_[centre, survey.support(centre) + shape.width / 2])) or [None])[0]
            if laid is None:
                self.get_logger().warning("Lost the piece after laying it out")
                standing = None
            elif not laid.lying:
                return laid.center
            else:
                standing = yield from self.pivot(survey, laid, share, side)
            # Failing, it is left at the spot, which later pieces are laid out clear of.
            if standing is None:
                self.failed.append(centre)
            return standing

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

        def pivot_spot(self, survey, shape, others):
            """Where to lay a piece out, pointing away from the robot, so that it can be turned
            up far enough to right itself: the nearest spot clear of the `others` pieces from
            which the arm pitches it furthest, and that pitch."""
            board = survey.board
            grip_frame = np.array([0.0, 0.0, TCP[2] - PAD_DEPTH + shape.width / 2])
            spots = [
                distance * np.array([math.cos(bearing), math.sin(bearing)])
                for bearing in np.radians(np.arange(-60, 61, 7.5)) for distance in (0.25, 0.23, 0.21)
            ]
            # On the desk, the whole piece and the gripper around it clear of the board.
            board = survey.board
            spots = [
                centre for centre in spots
                if max(np.abs(board.coordinates(centre))) > 4 * board.s + BOARD_FRAME + shape.length / 2 + MARGIN * 2
                # Clear of other pieces, but for the lying one itself, by half a square and a
                # margin for the fingers.
                and all(
                    np.linalg.norm(centre - q) > shape.length / 2 + board.s / 2 + MARGIN
                    for q in others if np.linalg.norm(q - shape.center) > board.s
                )
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
                    approach = Move(*self.overhead(survey, shape.center, above, OPEN, roll))
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
            put = survey.support(centre) + PAD_CLEARANCE + PAD_DEPTH
            roll = self.roll_for(centre, put, grasped + turn)
            # Over the pieces standing on the way, which it would sweep down lying.
            start = (self.arm.fk(self.target) @ TCP)[:2]
            planned = self.place_paths(
                survey.board, start, centre, put, PAD_CLEARANCE + PAD_DEPTH, held[0], roll, OPEN, self.standing(start), shape.length / 2
            )
            if planned is None:
                self.get_logger().warning("Cannot carry the lying piece out")
                return False
            (up, over, down, leave), gap = planned
            self.get_logger().info(f"{self.seconds():.0f} s: carrying it {gap * 1000:.0f} mm clear")
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
                    approach = Move(*self.overhead(survey, grip[:2], above, OPEN, roll))
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
            if not (yield from self.clear_move(home[:5], obstacles, self.surface(survey))):
                yield self.joint_move(home[:5], OPEN, 1.0)
            self.get_logger().info(f"{self.seconds():.0f} s: turned it up at {np.round(standing, 3).tolist()}")
            return standing

        def look_down(self, survey, middle, group=()):
            """A look straight down at a piece: the outlines from above of it and the pieces
            around it, nearest first; none if nothing is seen there."""
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
                if not (yield from self.clear_move(joints, obstacles, self.surface(survey))):
                    continue
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
                    return []
                near.sort(key=lambda s: np.linalg.norm(s.center - middle[:2]))
                shape = near[0]
                self.get_logger().info(
                    f"{self.seconds():.0f} s: from above: {shape.length * 1000:.0f} by {shape.width * 1000:.0f} mm, "
                    f"base {np.round(shape.base, 3).tolist()}, {'lying' if shape.lying else 'standing'}, "
                    f"head towards {math.degrees(math.atan2(shape.facing[1], shape.facing[0])):.0f} deg"
                )
                return near
            reasons = []
            for rise in DOWN_HEIGHTS:
                joints, error, cosine = self.arm.aim(np.r_[middle[:2], support + rise], (0, 0, -1), self.target)
                gap = clearance(self.arm, np.r_[joints, OPEN], obstacles, self.surface(survey))
                reasons.append(f"{rise * 100:.0f} cm: {error * 1000:.1f} mm off, cos {cosine:.3f}, {gap * 1000:.0f} mm clear")
            self.get_logger().info(
                f"No look from above at {np.round(middle[:2], 3).tolist()} (top {top * 1000:.0f} mm): " + "; ".join(reasons)
            )
            return []

        def inspect(self, survey, middle, group, search=1.0):
            """Closer looks from around the robot's side of a piece: where exactly it stands.
            Returns its middle, all its sightings and the close ones, or None if none of the
            looks finds it."""
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
                if not (yield from self.clear_move(joints, obstacles, self.surface(survey))):
                    continue
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
            return middle, group + close, close

        def jaw_for(self, gap):
            """The least jaw angle that opens the pads `gap` apart, at most fully open."""
            for jaw in np.linspace(CLOSED, OPEN, 60):
                fixed, moving = self.arm.pads(jaw, TCP[2] - PAD_DEPTH, TCP[2])
                if moving - fixed >= gap:
                    return float(jaw)
            return OPEN

        def room(self, q, others, support, s):
            """How far the gripper at joints q keeps from the pieces standing at `others`, xy
            points on a surface at height `support`: the least horizontal distance from its
            points below a piece's top to that piece's side, negative inside one."""
            if not others:
                return math.inf
            points = self.arm.solid(q)
            points = points[points[:, 2] < support + PIECE_TOP * s, :2]
            if not len(points):
                return math.inf
            return float(np.linalg.norm(points[:, None] - np.asarray(others)[None], axis=2).min()) - PIECE_RADIUS * s

        def room_at(self, xy, z, axis, roll, jaws, others, support, s):
            """The room the gripper has from the pieces at `others`, holding a piece's axis at
            xy between the pads, `axis` along its x, the tool at height z and the wrist at
            `roll`, at the least roomy of the jaw angles."""
            q, _ = self.arm.ik((*xy, z), self.target, roll=roll)
            tool = grasp_tool(self.arm, xy, np.r_[q, OPEN], axis)
            q, _ = self.arm.ik((*tool, z), q, roll=roll)
            return min(self.room(np.r_[q, jaw], others, support, s) for jaw in jaws)

        def roomy_roll(self, xy, z, axis, jaws, others, support, s):
            """The wrist roll that keeps the gripper furthest from the other pieces, holding a
            piece's axis at xy as `room_at` does: that room and the roll."""
            rolls = np.linspace(*self.arm.limits[4], ROLLS)
            return max((self.room_at(xy, z, axis, roll, jaws, others, support, s), float(roll)) for roll in rolls)

        def knight_yaw(self, survey, middle, grip, opening, free, others, heading):
            """The jaws' yaw to grasp a knight standing at `middle` and facing `heading` with,
            the pads at height `grip`: it turns with the gripper to face rank 1 on a free square
            of its kind, so the yaw it is gripped at sets the one it is let go of at. The yaw
            whose fingers have most room both times."""
            board = survey.board
            support = survey.support(middle[:2])
            release = board.z + grip - support + RELEASE_GAP
            rolls = np.linspace(*self.arm.limits[4], ROLLS)
            # The yaw of each roll over the knight, and the room there.
            here = [(self.yaw(np.r_[self.arm.ik((*middle[:2], grip), self.target, roll=r)[0], 0.0]), r) for r in rolls]
            grasping = between_pads(self.arm, opening)
            # Its yaw turns by as much as its heading must turn to face rank 1.
            turn = math.atan2(board.v[1], board.v[0]) - math.atan2(heading[1], heading[0])
            best = (-math.inf, None)
            for square in (q for q in SQUARES["knight"] if q in free):
                target = board.center(square)
                for roll in rolls:
                    there = self.yaw(np.r_[self.arm.ik((*target, release), self.target, roll=roll)[0], 0.0])
                    yaw = there - turn
                    off, grasp_roll = min((abs(math.remainder(y - yaw, 2 * math.pi)), r) for y, r in here)
                    if off > math.radians(10):
                        continue
                    room = min(
                        self.room_at(target, release, grasping, roll, (opening,), others, board.z, board.s),
                        self.room_at(middle[:2], grip, grasping, grasp_roll, (opening,), others, support, board.s),
                    )
                    best = max(best, (room, yaw), key=lambda b: b[0])
            return best[1]

        def grasp(self, survey, middle, group, kind, width, others, free=(), heading=None):
            """Grasp a piece of `kind`, `width` across, above its middle, the height its
            sightings met at, and lift it, the fingers kept clear of the pieces standing at
            `others`; a knight facing `heading` so that it has room on the `free` square it is
            set on, too. Returns what `place` needs if it is held, False if it slipped, or None
            if it is out of reach."""
            board = survey.board
            support = survey.support(middle[:2])
            tall = height(group, support, middle[:2])
            top = support + (tall if tall is not None else 2 * (middle[2] - support))
            # The pads grip its upper half, above its low centre of mass, so it hangs from them.
            grip = support + (top - support) / 2 + PAD_DEPTH
            # Open no wider than it needs, the fingers turned where they have most room.
            opening = self.jaw_for(width + GRASP_ROOM)
            axis = between_pads(self.arm, opening)
            yaw = None
            if kind == "knight" and heading is not None:
                work = Work(self.knight_yaw, survey, middle, grip, opening, free, others, heading)
                yield work
                yaw = work.value
            if yaw is None:
                work = Work(self.roomy_roll, middle[:2], grip, axis, (opening,), others, support, board.s)
                yield work
                room, roll = work.value
            else:
                roll = self.roll_for(middle[:2], grip, yaw)
                room = self.room_at(middle[:2], grip, axis, roll, (opening,), others, support, board.s)
            self.get_logger().info(
                f"{self.seconds():.0f} s: a {kind} at {np.round(middle[:2], 3).tolist()}, "
                f"about {(top - support) * 1000:.0f} mm tall; {room * 1000:.0f} mm room for the fingers"
            )
            # From the highest point in reach straight above it, at most a margin over its top;
            # close to the robot the arm reaches only lower, where the open jaws straddle it.
            # Over everything that may stand on the board on the way there, then straight down.
            highest = max(top, board.z + PIECE_TOP * board.s)
            for above in np.arange(highest + MARGIN + PAD_DEPTH, grip, -0.005):
                try:
                    # The whole path is planned before the arm moves.
                    approach = Move(*path(self.arm, self.target, (*middle[:2], above), opening, roll, 1.0))
                    # The piece's axis midway between the open pads, so closing moves it least.
                    tool = grasp_tool(self.arm, middle[:2], approach.points[-1], between_pads(self.arm, opening))
                    align = Move(*path(self.arm, approach.points[-1], (*tool, above), opening, roll))
                    descend = Move(*path(self.arm, align.points[-1], (*tool, grip), opening, roll, 2.0))
                    closed = np.r_[descend.points[-1][:5], CLOSED]
                    lift = Move(*path(self.arm, closed, (*tool, above), CLOSED, roll, 2.0))
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

        def place(self, survey, kind, hold, axis, grasped, free, back, others, heading=None):
            """Set a held piece on a `free` square of its kind and let go of it, or back where
            it was taken from, at `back`, if no such square is in reach, the fingers kept clear
            of the pieces standing at `others`. A knight's `heading`, seen when the gripper
            faced `grasped`, is turned towards rank 1: it turns with the gripper. Returns the
            square; None if it was set down at `back`, False if it is still held."""
            # It is let go of by opening the pads a little, no further.
            held = self.q[5]
            fixed, moving = self.arm.pads(held, TCP[2] - PAD_DEPTH, TCP[2])
            loose = self.jaw_for(moving - fixed + RELEASE_OPENING)
            work = Work(self.plan_place, survey, kind, hold, axis, grasped, free, back, others, heading, (held, loose))
            yield work
            if work.value is None:
                self.get_logger().error(f"Cannot set the {kind} down anywhere; holding it")
                return False
            square, room, gap, (up, over, down, leave) = work.value
            if square is None and free:
                self.get_logger().warning(f"No free square for the {kind} in reach; setting it back")
            self.get_logger().info(
                f"{self.seconds():.0f} s: setting the {kind} on {square}, {room * 1000:.0f} mm room for the fingers, "
                f"carried {gap * 1000:.0f} mm clear"
            )
            yield up
            yield over
            yield down
            yield self.grip_move(loose)
            yield leave
            home, _ = self.arm.ik(HOME, self.target)
            yield self.joint_move(home[:5], OPEN, 1.0)
            self.get_logger().info(f"{self.seconds():.0f} s: the {kind} is on {square}")
            return square

        def plan_place(self, survey, kind, hold, axis, grasped, free, back, others, heading, jaws):
            """Where to set a held piece down, as `place` does, and how: the nearest free
            square of its kind where the fingers have room at both `jaws` angles, else the
            roomiest, else back at `back`. Returns the square, the room and the moves, or None."""
            board = survey.board
            start = (self.arm.fk(self.target) @ TCP)[:2]
            release = board.z + hold + RELEASE_GAP
            spots = []
            for square in (q for q in SQUARES[kind] if q in free):
                target = board.center(square)
                if kind == "knight" and heading is not None:
                    turn = math.atan2(heading[1], heading[0]) - grasped
                    roll = self.roll_for(target, release, math.atan2(board.v[1], board.v[0]) - turn)
                    room = self.room_at(target, release, axis, roll, jaws, others, board.z, board.s)
                else:
                    room, roll = self.roomy_roll(target, release, axis, jaws, others, board.z, board.s)
                order = (room < 0, -room if room < 0 else np.linalg.norm(target - start))
                spots.append((order, square, target, release, room, roll))
            spots.sort(key=lambda spot: spot[0])
            support = survey.support(back)
            room, roll = self.roomy_roll(back, support + hold + RELEASE_GAP, axis, jaws, others, support, board.s)
            spots.append((None, None, back, support + hold + RELEASE_GAP, room, roll))
            obstacles = [(xy, PIECE_RADIUS * board.s, board.z + PIECE_TOP * board.s) for xy in others]
            for _, square, target, z, room, roll in spots:
                planned = self.place_paths(board, start, target, z, hold, axis, roll, jaws[1], obstacles)
                # A square is only gone to clear of the pieces; back where it came from, it goes
                # anyway.
                if planned and (planned[1] >= 0 or square is None):
                    moves, gap = planned
                    return square, room, gap, moves
                if square is not None:
                    nearest = min(others, key=lambda xy: np.linalg.norm(xy - target), default=None)
                    self.get_logger().info(
                        f"Not to {square}: " + ("out of reach" if planned is None else f"carried {-planned[1] * 1000:.0f} mm too low")
                        + ("" if nearest is None else f"; nearest piece at {np.round(nearest, 3).tolist()}")
                    )
            return None

        def reach(self, xy, ceiling, floor):
            """The highest tool height at most `ceiling` at which the arm reaches over xy with
            the fingers down; `floor` if none above it."""
            for z in np.arange(ceiling, floor, -0.005):
                q, _ = self.arm.ik((*xy, z), self.target)
                t = self.arm.fk(np.r_[q, OPEN])
                if np.linalg.norm((t @ TCP)[:3] - (*xy, z)) < 0.003 and t[2, 2] > FINGERS_DOWN:
                    return float(z)
            return floor

        def routes(self, start, target):
            """Ways to carry something from over `start` to over `target`: straight, and round
            the arm's base, where it reaches only low, along an arc further out."""
            yield []
            a, b = (math.atan2(*(np.asarray(p) - self.pan_axis[:2])[::-1]) for p in (start, target))
            span = math.remainder(b - a, 2 * math.pi)
            steps = max(1, math.ceil(abs(span) / math.radians(20)))
            turns = (a + span * i / steps for i in range(steps + 1))
            yield [self.pan_axis[:2] + CARRY_RADIUS * np.array([math.cos(t), math.sin(t)]) for t in turns]

        def carried_gap(self, points, hold, obstacles, reach):
            """The least gap between the gripper, or a piece hanging `hold` below the tool and
            reaching `reach` out from it, and the obstacles, (xy, radius, top) cylinders, along
            joint waypoints."""
            gap = math.inf
            for q in points:
                tool = (self.arm.fk(q) @ TCP)[:3]
                for xy, radius, top in obstacles:
                    if np.linalg.norm(tool[:2] - xy) < radius + reach:
                        gap = min(gap, tool[2] - hold - top)
                gap = min(gap, clearance(self.arm, q, obstacles, -math.inf))
            return gap

        def place_paths(self, board, start, target, release, hold, axis, roll, loose, obstacles, reach=None):
            """The moves carrying a held piece, reaching `reach` out from its axis, from over
            `start` to `target` and letting go of it there at height `release`, kept clear of
            the `obstacles` where a way allows, and how clear; None out of reach. It travels
            high enough to clear the tallest piece, rising and coming down only as high as the
            arm reaches over either end."""
            reach = PIECE_RADIUS * board.s if reach is None else reach
            here = (self.arm.fk(self.target) @ TCP)[2]
            best = None
            for route in self.routes(start, target):
                for air in (PIECE_TOP * board.s + MARGIN, *CARRY_CLEARANCES):
                    carry = board.z + hold + air
                    try:
                        rise = max(here, self.reach(start, carry, here))
                        up = path(self.arm, self.target, (*start, rise), CLOSED, self.target[4])
                        # Through the route at the carrying height, to over the target.
                        points, seconds = up[0][-1:], 0.0
                        for i, (x, y) in enumerate([*route, target]):
                            if i == len(route):
                                # The pads' offset from the piece's axis turns with the arm, so it
                                # is aimed from over the square.
                                z = self.reach((x, y), carry, release)
                                over, _ = path(self.arm, points[-1], (x, y, z), CLOSED, roll)
                                x, y = grasp_tool(self.arm, target, over[-1], axis)
                            else:
                                z = carry
                            segment, time = path(self.arm, points[-1], (x, y, z), CLOSED, roll, 2.0 if i == 0 else 1.0)
                            points, seconds = points + segment[1:], seconds + time
                        tool = (x, y)
                        down = Move(*path(self.arm, points[-1], (*tool, release), CLOSED, roll, 2.0))
                        opened = np.r_[down.points[-1][:5], loose]
                        leave = Move(*path(self.arm, opened, (*tool, z), loose, roll, 1.5))
                    except ValueError:
                        continue
                    gap = self.carried_gap(up[0] + points, hold, obstacles, reach)
                    moves = (Move(*up), Move(points, seconds), down, leave)
                    if gap >= 0:
                        return moves, gap
                    if best is None or gap > best[1]:
                        best = (moves, gap)
            return best

    rclpy.init()
    node = Controller()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # An eject sends SIGINT twice, to the process group and through ros2 launch: ignore the
        # second while the worker thread winds down.
        signal.signal(signal.SIGINT, signal.SIG_IGN)


if __name__ == "__main__":
    main()
