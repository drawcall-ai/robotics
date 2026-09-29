// The whole black set lies scattered within reach, some pieces on their sides; the robot must
// set up the board: every piece on a square of its kind, knights facing the far side.
import { Evaluator } from "@drawcall/robotics";
import { Quaternion, Vector3 } from "three";
import {
  boardMoved,
  boardPeak,
  placed,
  random,
  scatter,
  touching,
} from "./environment.ts";
import { deskHeight, pieces } from "./task.ts";

const set = pieces.map((kind) => scatter(kind, random() < 0.3));
// Where each piece came to rest on its square, and when.
const rest = new Map<
  (typeof set)[number],
  { since: number; position: Vector3; rotation: Quaternion }
>();
function settled(piece: (typeof set)[number], elapsed: number): boolean {
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
let done = 0;
new Evaluator({
  timeout: 3600,
  metrics: {
    placed: () => done,
    peakBoardDisplacement: () => boardPeak.displacement,
  },
  failure: {
    boardMoved,
    pieceDropped: () =>
      set.some(({ body }) => body.position.y < deskHeight - 0.04),
  },
  success: {
    boardSetUp: ({ elapsed }) => {
      done = set.filter((piece) => settled(piece, elapsed)).length;
      return done === set.length;
    },
  },
  score: ({ status, metrics }) =>
    status === "success" ? 100 : (100 * (metrics.placed ?? 0)) / set.length,
});
