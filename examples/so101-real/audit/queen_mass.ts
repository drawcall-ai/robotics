// The sim queen's mass, centre of mass and tipping angle from its collision mesh (density 700) plus
// the base weight (12x6x12 mm, density 11000), as pieces.ts builds it. Run: node audit/queen_mass.ts
import { Box3 } from "three";
import { chessSet, closed, flatBase } from "../assets.ts";
import { QUEEN_BASE, QUEEN_LENGTH } from "../pieces.ts";

const geometry = (await chessSet()).queen!;
const bounds = new Box3().setFromBufferAttribute(geometry.getAttribute("position") as never);
const sx = QUEEN_BASE / (bounds.max.x - bounds.min.x);
const sy = QUEEN_LENGTH / bounds.max.y;
const shape = flatBase(closed(geometry));
const p = shape.getAttribute("position").array as ArrayLike<number>;
const index = shape.getIndex()?.array;
const n = index ? index.length : p.length / 3;
const v = (i: number) => {
  const k = index ? index[i]! : i;
  return [p[3 * k]! * sx, p[3 * k + 1]! * sy, p[3 * k + 2]! * sx];
};
let volume = 0;
let cy = 0;
// Signed tetrahedra against the origin: volume and first moment in y.
for (let i = 0; i < n; i += 3) {
  const [a, b, c] = [v(i), v(i + 1), v(i + 2)];
  const det =
    a[0]! * (b[1]! * c[2]! - b[2]! * c[1]!) -
    a[1]! * (b[0]! * c[2]! - b[2]! * c[0]!) +
    a[2]! * (b[0]! * c[1]! - b[1]! * c[0]!);
  volume += det / 6;
  cy += (det / 6) * ((a[1]! + b[1]! + c[1]!) / 4);
}
const wood = 700 * Math.abs(volume);
const woodY = cy / volume;
const lead = 11000 * 0.012 * 0.006 * 0.012;
const mass = wood + lead;
const com = (wood * woodY + lead * 0.005) / mass;
const tip = (Math.atan2(QUEEN_BASE / 2, com) * 180) / Math.PI;
const uniform = (Math.atan2(QUEEN_BASE / 2, woodY) * 180) / Math.PI;
console.log(
  JSON.stringify({
    volumeCm3: +(Math.abs(volume) * 1e6).toFixed(1),
    woodG: +(wood * 1000).toFixed(1),
    leadG: +(lead * 1000).toFixed(1),
    massG: +(mass * 1000).toFixed(1),
    comCm: +(com * 100).toFixed(2),
    woodOnlyComCm: +(woodY * 100).toFixed(2),
    tipOverDeg: +tip.toFixed(1),
    tipOverUniformDeg: +uniform.toFixed(1),
  }),
);
