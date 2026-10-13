// The whole black set lies scattered within reach, some pieces on their sides; the robot must
// set up the board: every piece on a square of its kind, knights facing the far side.
import type { RigidBody } from "@drawcall/physics";
import { Evaluator, getROS2 } from "@drawcall/robotics";
import { Quaternion, Vector3 } from "three";
import {
  boardBody,
  boardMoved,
  boardPeak,
  placed,
  random,
  robot,
  scatter,
  still,
  touching,
  type Piece,
} from "./environment.ts";
import { deskHeight, pieces, ros, tilt } from "./task.ts";

export { scene, show } from "./environment.ts";

const log = (message: string) => getROS2().log(message);
const heading = (body: RigidBody) => {
  const [fx, fy] = ros(new Vector3(1, 0, 0).applyQuaternion(body.quaternion));
  return (Math.atan2(fy, fx) * 180) / Math.PI;
};
const describe = ({ kind, body }: Piece) => {
  const [x, y] = ros(body.position);
  const deg = (tilt(body.quaternion) * 180) / Math.PI;
  return `${body.name} at ${x.toFixed(3)} ${y.toFixed(3)}, tilted ${deg.toFixed(0)} deg${kind === "knight" ? `, heading ${heading(body).toFixed(0)} deg` : ""}`;
};

// The share of pieces that start lying; LYING=0 sets every piece up standing, a test of
// recognising and placing them apart from standing them up.
const lying = Number(process.env.LYING ?? 0.3);
const set = pieces.map((kind) => scatter(kind, random() < lying));
log(`Pieces: ${set.map(describe).join("; ")}`);

// Where each piece came to rest on its square, and when.
const rest = new Map<
  Piece,
  { since: number; position: Vector3; rotation: Quaternion }
>();
function settled(piece: Piece, elapsed: number): boolean {
  const { body } = piece;
  if ((touching.get(body)?.size ?? 0) > 0 || !placed(piece)) {
    rest.delete(piece);
    return false;
  }
  const at = rest.get(piece);
  // Rest is judged by pose, not velocity: contact solvers keep bodies trembling by fractions
  // of a millimetre.
  if (
    !at ||
    at.position.distanceTo(body.position) > 0.001 ||
    at.rotation.angleTo(body.quaternion) > Math.PI / 90
  ) {
    rest.set(piece, {
      since: elapsed,
      position: body.position.clone(),
      rotation: body.quaternion.clone(),
    });
    return false;
  }
  return elapsed - at.since >= 1;
}

// A touch of the board lasts from the first robot link touching it until none does.
const onBoard = new Set<RigidBody>();
let boardTouches = 0;
boardBody.addEventListener("contactbegin", ({ otherBody }) => {
  if (otherBody.parent !== robot) return;
  if (onBoard.size === 0) {
    boardTouches++;
    log(`Board touched by ${otherBody.name}`);
  }
  onBoard.add(otherBody);
});
boardBody.addEventListener("contactend", ({ otherBody }) =>
  onBoard.delete(otherBody),
);

// Every move of a piece from one still pose to another: whether both jaws held it on the way,
// and whether it went from standing to lying. Only the robot moves anything in this scene, so
// a piece pushed along by another counts too.
const jaws = ["gripper_link", "moving_jaw_so101_v1_link"];
const held = (body: RigidBody) =>
  jaws.every((jaw) =>
    [...(touching.get(body) ?? [])].some(({ name }) => name === jaw),
  );
const poses = new Map<
  Piece,
  { position: Vector3; tilt: number; held: boolean }
>();
const disturbed = new Set<Piece>();
const knockedOver = new Set<Piece>();
function watch(piece: Piece) {
  const { body } = piece;
  const holding = held(body);
  const angle = tilt(body.quaternion);
  const last = poses.get(piece);
  if (last && holding) last.held = true;
  if (!still(body)) return;
  if (
    last &&
    last.position.distanceTo(body.position) <= 0.003 &&
    Math.abs(last.tilt - angle) <= Math.PI / 9
  )
    return;
  poses.set(piece, {
    position: body.position.clone(),
    tilt: angle,
    held: holding,
  });
  if (!last) return;
  const tipped = last.tilt < Math.PI / 18 && angle > Math.PI / 4;
  if (tipped) knockedOver.add(piece);
  if (!last.held) disturbed.add(piece);
  if (!tipped && last.held) return;
  const by = [...(touching.get(body) ?? [])].map(({ name }) => name).join(" ");
  log(
    `${tipped ? "Knocked over" : "Pushed"} ${describe(piece)}${by ? `, touching ${by}` : ""}`,
  );
}

let done = 0;
let boardLogged = 0;
new Evaluator({
  timeout: 3600,
  metrics: {
    placed: () => done,
    peakBoardDisplacement: () => boardPeak.displacement,
    knockedOver: () => knockedOver.size,
    boardTouches: () => boardTouches,
    piecesDisturbed: () => disturbed.size,
  },
  failure: {
    boardMoved,
    pieceDropped: () =>
      set.some(({ body }) => body.position.y < deskHeight - 0.04),
  },
  success: {
    boardSetUp: ({ elapsed }) => {
      for (const piece of set) watch(piece);
      if (boardPeak.displacement > boardLogged + 0.001) {
        boardLogged = boardPeak.displacement;
        log(`Board pushed ${(boardLogged * 1000).toFixed(1)} mm`);
      }
      done = set.filter((piece) => settled(piece, elapsed)).length;
      return done === set.length;
    },
  },
  score: ({ status, metrics }) =>
    status === "success" ? 100 : (100 * (metrics.placed ?? 0)) / set.length,
});
