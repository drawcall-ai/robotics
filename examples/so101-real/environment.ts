// The real SO-101 queen scene as measured on 2026-10-07/08 (see FIDELITY.md): a white table the
// arm is screwed to, a folding wooden chessboard with 31 mm squares in front of it, and one dark
// wooden queen lying on the table to the board's left. SEED=0 is the measured scene; other seeds
// vary the board, the queen and the light widely (see `variation` below). TASK=scatter starts the
// queen upright on d8 instead, for the opposite task.
//
// Ground truth: a policy must not read this file.
import { BoxCollider, RigidBody } from "@drawcall/physics";
import {
  BoxGeometry,
  Color,
  DataTexture,
  DirectionalLight,
  HemisphereLight,
  LinearMipmapLinearFilter,
  Mesh,
  MeshStandardMaterial,
  PlaneGeometry,
  Quaternion,
  Scene,
  SRGBColorSpace,
  Vector3,
} from "three";
import { environment, material } from "./assets.ts";
import { QUEEN_BASE, QUEEN_LENGTH, queen } from "./pieces.ts";
import { arm } from "./robot.ts";
import {
  boardCoordinates,
  boardPoint,
  deskHeight,
  seeded,
  squareCenter,
  world,
  type Board,
} from "./task.ts";

export const show: string[] = [];

const seed = Number(process.env.SEED ?? 0);
const random = seeded(seed);
const measured = seed === 0;
/** Uniform in ±`amount`, and exactly 0 for the measured scene, seed 0. */
const jitter = (amount: number) => (measured ? 0 : (2 * random() - 1) * amount);
const between = (low: number, high: number) => low + random() * (high - low);
const pick = <T>(options: readonly T[]): T => options[Math.floor(random() * options.length)]!;
const deg = Math.PI / 180;
/** `place`: stand the queen on d8. `scatter`: the queen starts on d8; knock it off. */
export const task = (process.env.TASK ?? "place") as "place" | "scatter";
if (task !== "place" && task !== "scatter") throw new Error(`Unknown TASK ${task}`);
/** The shoulder's pan axis in the base frame, and how far the arm reaches from it. */
const SHOULDER = [0.0388, 0] as const;
const reach = (x: number, y: number) => Math.hypot(x - SHOULDER[0], y - SHOULDER[1]);

// The real camera has no white balance: its images run from neutral to a strong red/pink. A
// per-seed colour cast of the light, 0 neutral to 1 the real camera's, and a brightness.
const cast = measured ? 0 : random() < 0.3 ? 0 : between(0.2, 1);
const brightness = measured ? 1 : between(0.6, 1.3);
const gain = new Color(1, 1 - 0.55 * cast, 1 - 0.3 * cast);
export const scene = new Scene();
scene.background = new Color("#d9d4cf").multiply(gain);
scene.environment = environment(
  "public/environment/stark-white-brutalist-digital-sky-high-contrast-minimalist-terminal-aesthetic.hdr",
);
scene.environmentIntensity = 0.5 * brightness * (1 - 0.6 * cast);
scene.add(new HemisphereLight(new Color("#ffffff").multiply(gain), "#8a8580", 0.8 * brightness));
// A ceiling lamp, a little behind and to the right of the robot.
const lamp = new DirectionalLight(new Color("#fff4e6").multiply(gain), 2 * brightness);
lamp.position.set(0.3, deskHeight + 1.5, 0.4);
lamp.target.position.set(0, deskHeight, -0.25);
lamp.castShadow = true;
lamp.shadow.mapSize.set(2048, 2048);
lamp.shadow.camera.left = lamp.shadow.camera.bottom = -0.5;
lamp.shadow.camera.right = lamp.shadow.camera.top = 0.5;
lamp.shadow.camera.near = 0.5;
lamp.shadow.camera.far = 4;
lamp.shadow.bias = -0.0003;
lamp.shadow.normalBias = 0.002;
scene.add(lamp, lamp.target);

// A plain white table; the robot's base plate is screwed to it.
export const table = new RigidBody({ bodyType: "static" });
table.name = "table";
table.position.set(0, deskHeight - 0.025, -0.2);
table.setMaterial({ staticFriction: 0.5, dynamicFriction: 0.5, restitution: 0 });
const tableTop = new Mesh(
  new BoxGeometry(1.2, 0.05, 0.9),
  new MeshStandardMaterial({ color: "#eeebe6", roughness: 0.85 }),
);
tableTop.receiveShadow = true;
table.add(new BoxCollider({ size: [1.2, 0.05, 0.9] }), tableTop);
export const { robot, bus } = arm(deskHeight);
scene.add(table, robot);

// The folding board as the real arm's careful calibration found it (2026-10-08 evening): 31.0 mm
// squares, a 15 mm frame, playing surface 22.8 mm above the table, yawed 3.5°, rank 8 nearest the
// robot and the a-file on its right; the a8 corner at (18.8, −9.8) cm, d8 at about (19.7, 1.1).
// Other seeds: ±5 cm, ±15°, squares of 28–35 mm, frames of 10–22 mm, ±4 mm in height, resampled
// until the near edge keeps clear of the robot and d8 is within reach.
function drawBoard(): Board {
  if (measured)
    return { s: 0.031, border: 0.015, thickness: 0.0228, x: 0.304, y: 0.033, yaw: 3.5 * deg };
  for (let attempt = 0; attempt < 1000; attempt++) {
    const candidate: Board = {
      s: between(0.028, 0.035),
      border: between(0.01, 0.022),
      thickness: 0.0228 + jitter(0.004),
      x: 0.304 + jitter(0.05),
      y: 0.033 + jitter(0.05),
      yaw: (3.5 + jitter(15)) * deg,
    };
    const half = 4 * candidate.s + candidate.border;
    const corners = [[-1, -1], [-1, 1], [1, -1], [1, 1]].map(([u, v]) =>
      boardPoint(candidate, u! * half, v! * half),
    );
    const [dx, dy] = squareCenter(candidate, "d8");
    if (corners.every(([x]) => x > 0.12) && reach(dx, dy) < 0.27) return candidate;
  }
  throw new Error("Could not place the board");
}
export const board = drawBoard();
const { s, border, thickness } = board;
const width = 8 * s + 2 * border;
const woods = [await material("wood-01"), await material("wood-02")];
const grain = woods[0]!.map.image as { data: Uint8Array; width: number; height: number };
// Inlaid woods: the real board's maple and rosewood, or other pairs, each varied a little.
const vary = (hex: string) =>
  measured ? new Color(hex) : new Color(hex).offsetHSL(jitter(0.02), jitter(0.08), jitter(0.05));
const light = vary(measured ? "#e6c39a" : pick(["#e6c39a", "#efd9b4", "#e2c08e", "#f0e0c0"]));
const dark = vary(measured ? "#6e2e1c" : pick(["#6e2e1c", "#5a2a1a", "#7a4428", "#4a2618"]));
const frame = vary(measured ? "#9c4a2c" : pick(["#9c4a2c", "#7a3a20", "#b0603a", "#5e3420"]));
const seam = new Color("#3a1c10");
const size = 1024;
const pixels = new Uint8Array(size * size * 4);
for (let j = 0; j < size; j++)
  for (let i = 0; i < size; i++) {
    // Texture u runs along -files, v along ranks away from the robot; see `task.ts`.
    const u = -((i + 0.5) / size - 0.5) * width;
    const v = ((j + 0.5) / size - 0.5) * width;
    const inside = Math.max(Math.abs(u), Math.abs(v)) < 4 * s;
    const file = Math.floor(u / s + 4);
    const rank = 8 - Math.floor(v / s + 4);
    const isDark = (file + 1 + rank) % 2 === 0;
    // The fold between ranks 4 and 5 crosses frame and squares alike.
    const color =
      Math.abs(v) < 0.0006 ? seam : !inside ? frame : isDark ? dark : light;
    let a = u / (4 * s) * 0.5 + (inside ? file * 0.37 : 0.3);
    let b = v / (4 * s) * 0.5 + (inside ? rank * 0.21 : 0.7);
    if (inside && isDark) [a, b] = [b, a];
    if (!inside && Math.abs(u) > Math.abs(v)) [a, b] = [b, a];
    const gx = Math.floor((((a % 1) + 1) % 1) * grain.width);
    const gy = Math.floor((((b % 1) + 1) % 1) * grain.height);
    const k = 4 * (gy * grain.width + gx);
    const luminance =
      (grain.data[k]! * 0.3 + grain.data[k + 1]! * 0.59 + grain.data[k + 2]! * 0.11) / 255;
    const shade = 1 + 0.8 * (luminance / 0.45 - 1) * 0.5;
    const o = 4 * (j * size + i);
    pixels[o] = Math.round(255 * Math.min(1, color.r * shade) ** (1 / 2.2));
    pixels[o + 1] = Math.round(255 * Math.min(1, color.g * shade) ** (1 / 2.2));
    pixels[o + 2] = Math.round(255 * Math.min(1, color.b * shade) ** (1 / 2.2));
    pixels[o + 3] = 255;
  }
const boardTexture = new DataTexture(pixels, size, size);
boardTexture.colorSpace = SRGBColorSpace;
boardTexture.minFilter = LinearMipmapLinearFilter;
boardTexture.generateMipmaps = true;
boardTexture.anisotropy = 4;
boardTexture.needsUpdate = true;
export const boardBody = new RigidBody({ mass: 1.0 });
boardBody.name = "chessboard";
boardBody.position.copy(world(board.x, board.y, deskHeight + thickness));
boardBody.rotation.y = board.yaw;
// Lacquered top: 0.2, so the felted queen slides on it (the table's 0.5 still holds the board).
boardBody.setMaterial({ staticFriction: 0.2, dynamicFriction: 0.2, restitution: 0 });
const slab = new BoxCollider({ size: [width, thickness, width] });
slab.position.y = -thickness / 2;
const sides = { ...woods[1]!, color: frame, roughness: 0.6 };
const slabMesh = new Mesh(
  new BoxGeometry(width, thickness, width),
  new MeshStandardMaterial(sides),
);
slabMesh.position.y = -thickness / 2;
slabMesh.castShadow = slabMesh.receiveShadow = true;
const top = new Mesh(
  new PlaneGeometry(width, width),
  new MeshStandardMaterial({ map: boardTexture, roughness: 0.45 }),
);
top.rotation.x = -Math.PI / 2;
top.position.y = 0.0002;
top.receiveShadow = true;
boardBody.add(slab, slabMesh, top);
scene.add(boardBody);

// The queen. Seed 0: lying on the table, its middle at about (24.5, 23.5) cm, the crown pointing
// away from the robot at 64° (REAL_OBSERVATIONS §4 and the later real views). Other seeds: standing
// or lying (on its side; a queen cannot rest on its crown) anywhere within reach on the table or
// the board but off d8, any heading, 0.85–1.15 times the size, dark to medium wood. TASK=scatter:
// standing on d8.
/** The queen's size against the real one. */
export const scale = measured ? 1 : between(0.85, 1.15);
const tint = new Color("#a07060").multiplyScalar(measured ? 1 : between(0.6, 1.5));
export const queenBody = queen(scale, tint);
const length = QUEEN_LENGTH * scale;
const radius = (QUEEN_BASE / 2) * scale;
const [d8x, d8y] = squareCenter(board, "d8");
const boardTop = deskHeight + thickness;
const spin = new Quaternion().setFromAxisAngle(new Vector3(0, 1, 0), random() * 2 * Math.PI);
/** Stands the queen with its base centred at (x, y) on a surface at height `support`. */
function stand(x: number, y: number, support: number) {
  queenBody.quaternion.copy(spin);
  queenBody.position.copy(world(x, y, support + 0.0005));
}
/** Lays the queen on its side, its middle at (x, y), the crown towards `heading`. */
function lay(x: number, y: number, heading: number, support: number) {
  const axis = world(Math.cos(heading), Math.sin(heading), 0).sub(world(0, 0, 0));
  queenBody.quaternion.setFromUnitVectors(new Vector3(0, 1, 0), axis).multiply(spin);
  queenBody.position
    .copy(world(x, y, support + radius + 0.0005))
    .addScaledVector(axis, -length / 2);
}
if (task === "scatter") stand(d8x, d8y, boardTop);
else if (measured) {
  const heading = Math.atan2(24.2 - 17.4, 26.3 - 23.0);
  lay(0.245, 0.235, heading, deskHeight);
} else {
  const lying = random() < 0.6;
  const footprint = lying ? length / 2 + radius : radius;
  const half = 4 * board.s + board.border;
  for (let attempt = 0; ; attempt++) {
    if (attempt > 10000) throw new Error("Could not place the queen");
    const r = between(0.13, lying ? 0.3 : 0.29);
    const angle = between(-75, 75) * deg;
    const x = SHOULDER[0] + r * Math.cos(angle);
    const y = SHOULDER[1] + r * Math.sin(angle);
    const [u, v] = boardCoordinates(board, x, y);
    const edge = Math.max(Math.abs(u), Math.abs(v));
    const onBoard = edge < half - footprint - 0.003;
    if (!onBoard && edge < half + footprint + 0.005) continue;
    if (Math.hypot(x - d8x, y - d8y) < board.s + footprint) continue;
    // The folded arm's gripper rests about 11 cm up over (25, 0.5) cm: no tall queen under it.
    if (!lying && Math.hypot(x - 0.25, y - 0.005) < 0.05) continue;
    const support = onBoard ? boardTop : deskHeight;
    if (lying) lay(x, y, between(-180, 180) * deg, support);
    else stand(x, y, support);
    break;
  }
}
scene.add(queenBody);
console.error(`Scene seed ${seed}, task ${task}`);
