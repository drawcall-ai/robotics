// The real SO-101 queen scene as measured on 2026-10-07/08 (see FIDELITY.md): a white table the
// arm is screwed to, a folding wooden chessboard with 31 mm squares in front of it, and one dark
// wooden queen lying on the table to the board's left. SEED=0 is the measured scene; other seeds
// move the board and queen within the measurements' uncertainty.
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
import { QUEEN_BASE, queen } from "./pieces.ts";
import { arm } from "./robot.ts";
import { boardCoordinates, deskHeight, seeded, world, type Board } from "./task.ts";

export const show: string[] = [];

const seed = Number(process.env.SEED ?? 0);
const random = seeded(seed);
/** Uniform in ±`amount`, and exactly 0 for the measured scene, seed 0. */
const jitter = (amount: number) => (seed === 0 ? 0 : (2 * random() - 1) * amount);
const deg = Math.PI / 180;

export const scene = new Scene();
scene.background = new Color("#d9d4cf");
scene.environment = environment(
  "public/environment/stark-white-brutalist-digital-sky-high-contrast-minimalist-terminal-aesthetic.hdr",
);
scene.environmentIntensity = 0.5;
scene.add(new HemisphereLight("#ffffff", "#8a8580", 0.8));
// A ceiling lamp, a little behind and to the right of the robot.
const lamp = new DirectionalLight("#fff4e6", 2);
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
export const table = new RigidBody({ type: "static" });
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
export const board: Board = {
  s: 0.031,
  border: 0.015,
  thickness: 0.0228,
  x: 0.304 + jitter(0.005),
  y: 0.033 + jitter(0.005),
  yaw: (3.5 + jitter(1)) * deg,
};
const { s, border, thickness } = board;
const width = 8 * s + 2 * border;
const woods = [await material("wood-01"), await material("wood-02")];
const grain = woods[0]!.map.image as { data: Uint8Array; width: number; height: number };
const light = new Color("#e6c39a");
const dark = new Color("#6e2e1c");
const frame = new Color("#9c4a2c");
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
boardBody.setMaterial({ staticFriction: 0.5, dynamicFriction: 0.5, restitution: 0 });
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

// The queen lies on the table, its middle at about (24.5, 23.5) cm, the crown pointing away from
// the robot at 64° (REAL_OBSERVATIONS §4 and the later real views).
const heading = Math.atan2(24.2 - 17.4, 26.3 - 23.0) + jitter(8) * deg;
const baseX = 0.2285 + jitter(0.01);
let baseY = 0.201 + jitter(0.01);
// Never inside the board's frame: the base's rim keeps 5 mm off its left edge.
{
  const [u] = boardCoordinates(board, baseX, baseY);
  const clear = 4 * s + border + QUEEN_BASE / 2 + 0.005;
  if (u < clear) baseY += clear - u;
}
export const queenBody = queen();
const axis = world(Math.cos(heading), Math.sin(heading), 0).sub(world(0, 0, 0));
queenBody.quaternion.setFromUnitVectors(new Vector3(0, 1, 0), axis);
// Turned about its own axis at random: the crown has no preferred side.
queenBody.quaternion.multiply(
  new Quaternion().setFromAxisAngle(new Vector3(0, 1, 0), random() * 2 * Math.PI),
);
queenBody.position.copy(world(baseX, baseY, deskHeight + QUEEN_BASE / 2 + 0.0005));
scene.add(queenBody);
console.error(`Scene seed ${seed}`);
