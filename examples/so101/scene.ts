// One piece lies on its side somewhere within reach; the robot must stand it on its square,
// a knight facing the far side of the board.
import { Evaluator } from "@drawcall/robotics";
import { Quaternion, Vector3 } from "three";
import { kinds, type Kind } from "./pieces.ts";
import {
  boardMoved,
  boardPeak,
  placed,
  random,
  scatter,
  touching,
} from "./environment.ts";
import { deskHeight } from "./task.ts";

const kind = (process.env.PIECE ??
  kinds[Math.floor(random() * kinds.length)]) as Kind;
if (!kinds.includes(kind)) throw new Error(`Unknown piece ${kind}`);
const piece = scatter(kind, true);
console.error(`Piece: a lying ${kind}`);
const { body } = piece;
// Where it came to rest, and when.
let rest:
  { since: number; position: Vector3; rotation: Quaternion } | undefined;
new Evaluator({
  timeout: 600,
  metrics: {
    peakBoardDisplacement: () => boardPeak.displacement,
  },
  failure: {
    boardMoved,
    pieceDropped: () => body.position.y < deskHeight - 0.04,
  },
  success: {
    // Upright on its square, facing the right way, let go of and still there a second later.
    // Rest is judged by pose, not velocity: contact solvers keep bodies trembling by fractions
    // of a millimetre.
    piecePlaced: ({ elapsed }) => {
      const free = (touching.get(body)?.size ?? 0) === 0;
      if (!free || !placed(piece)) {
        rest = undefined;
        return false;
      }
      if (
        !rest ||
        rest.position.distanceTo(body.position) > 0.001 ||
        rest.rotation.angleTo(body.quaternion) > Math.PI / 90
      ) {
        rest = {
          since: elapsed,
          position: body.position.clone(),
          rotation: body.quaternion.clone(),
        };
        return false;
      }
      return elapsed - rest.since >= 1;
    },
  },
  score: ({ status }) => (status === "success" ? 100 : 0),
});
