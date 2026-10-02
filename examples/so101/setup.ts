// The whole black set lies scattered within reach, some pieces on their sides; the robot must
// set up the board: every piece on a square of its kind, knights facing the far side.
import { Evaluator } from "@drawcall/robotics";
import { Quaternion, Vector3 } from "three";
import {
  board,
  boardBody,
  boardMoved,
  robot,
  still,
  boardPeak,
  placed,
  random,
  scatter,
  touching,
} from "./environment.ts";
import { boardCoordinates, deskHeight, pieces, ros, squareAt, tilt } from "./task.ts";

// The share of pieces that start lying; LYING=0 sets every piece up standing, a test of
// recognising and placing them apart from standing them up.
const lying = Number(process.env.LYING ?? 0.3);
const set = pieces.map((kind) => scatter(kind, random() < lying));
console.error(
  "Pieces: " +
    set
      .map(({ kind, body }) => {
        const [fx, fy] = ros(new Vector3(1, 0, 0).applyQuaternion(body.quaternion));
        const heading = kind === "knight" ? ` heading ${((Math.atan2(fy, fx) * 180) / Math.PI).toFixed(0)}` : "";
        return `${kind} ${ros(body.position).map((v) => v.toFixed(3)).join(" ")}${heading}`;
      })
      .join(", "),
);
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
// Debug: which robot links touch the board, and when.
let now = 0;
boardBody.addEventListener("contactbegin", ({ otherBody }) => {
  if (otherBody.parent === robot) console.error(`Board touched at ${now.toFixed(1)} s by ${otherBody.name}`);
});
for (const [i, { body, kind }] of set.entries())
  body.addEventListener("contactbegin", ({ otherBody }) => {
    if (otherBody.parent === robot) console.error(`Piece touched at ${now.toFixed(1)} s: #${i} ${kind} by ${otherBody.name}`);
  });
let done = 0;
let logged = 0;
let boardLogged = 0;
const last = new Map<(typeof set)[number], { position: Vector3; deg: number }>();
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
      now = elapsed;
      done = set.filter((piece) => settled(piece, elapsed)).length;
      for (const [i, piece] of set.entries()) {
        const was = last.get(piece);
        const deg = (tilt(piece.body.quaternion) * 180) / Math.PI;
        if (!was || was.position.distanceTo(piece.body.position) > 0.003 || Math.abs(was.deg - deg) > 20) {
          if (was && still(piece.body)) {
            const [x, y] = ros(piece.body.position);
            const by = [...(touching.get(piece.body) ?? [])].map((b) => b.name).join(" ");
            console.error(`Moved at ${elapsed.toFixed(1)} s: #${i} ${piece.kind} to ${x.toFixed(3)} ${y.toFixed(3)}, ${deg.toFixed(0)} deg${by ? `, touched by ${by}` : ""}`);
          }
          if (!was || still(piece.body)) last.set(piece, { position: piece.body.position.clone(), deg });
        }
      }
      if (boardPeak.displacement > boardLogged + 0.001) {
        boardLogged = boardPeak.displacement;
        console.error(`Board pushed at ${elapsed.toFixed(1)} s: ${(boardLogged * 1000).toFixed(1)} mm`);
      }
      if (elapsed >= logged + 300) {
        logged = elapsed;
        console.error(
          `State at ${elapsed.toFixed(0)} s: ` +
            set
              .map((piece) => {
                const [x, y] = ros(piece.body.position);
                const square = squareAt(board, x, y, 0) ?? "off";
                const [u, v] = boardCoordinates(board, x, y);
                const off = Math.hypot(u / board.s - Math.floor(u / board.s) - 0.5, v / board.s - Math.floor(v / board.s) - 0.5) * board.s * 1000;
                const deg = (tilt(piece.body.quaternion) * 180) / Math.PI;
                const [fx, fy] = ros(new Vector3(1, 0, 0).applyQuaternion(piece.body.quaternion));
                const face = piece.kind === "knight" ? ` faces ${((Math.abs((((Math.atan2(fy, fx) - board.yaw) % (2 * Math.PI)) + 3 * Math.PI) % (2 * Math.PI) - Math.PI) * 180) / Math.PI).toFixed(0)}deg off` : "";
                return `${piece.kind} ${square} ${off.toFixed(1)}mm ${x.toFixed(3)} ${y.toFixed(3)} ${deg.toFixed(0)}deg${face}${placed(piece) ? " PLACED" : ""}`;
              })
              .join(", "),
        );
      }
      return done === set.length;
    },
  },
  score: ({ status, metrics }) =>
    status === "success" ? 100 : (100 * (metrics.placed ?? 0)) / set.length,
});
