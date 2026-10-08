// The one piece of the real scene: a dark wooden Staunton queen, the CC0 set's queen scaled to the
// real one as the re-calibrated wrist camera measured it standing on the board (2026-10-08,
// rr_V3): 2.26 cm across the base and about 1.5 cm across the crown's collar, both 0.56 of the CC0
// queen's, and 7.3 cm long (REAL_OBSERVATIONS §4: 7.3–7.8). Its earlier 2.9 cm base came through
// the old, 9°-off calibration.
import { BoxCollider, MeshCollider, RigidBody } from "@drawcall/physics";
import { Box3, Color, Mesh, MeshStandardMaterial } from "three";
import { chessSet, closed, flatBase, texture } from "./assets.ts";

export const QUEEN_LENGTH = 0.073;
export const QUEEN_BASE = 0.0226;

const original = (await chessSet()).queen!;
const bounds = new Box3().setFromBufferAttribute(
  original.getAttribute("position") as never,
);
const base = QUEEN_BASE / (bounds.max.x - bounds.min.x);
const geometry = original.clone().scale(base, QUEEN_LENGTH / bounds.max.y, base);
geometry.computeVertexNormals();
const shape = flatBase(closed(geometry));
const [map, packed, normalMap] = await Promise.all([
  texture("assets/chess/chess_set_pieces_black_diff_1k.jpg", true),
  texture("assets/chess/chess_set_pieces_black_arm_1k.jpg", false),
  texture("assets/chess/chess_set_pieces_black_nor_gl_1k.jpg", false),
]);
for (const t of [map, packed, normalMap]) t.flipY = false;

/**
 * A dynamic queen standing on its base at the body origin, +Y up. Lacquered wood, about 30 g
 * with a small weight in the base, friction 0.35 against the printed jaws.
 */
export function queen(): RigidBody {
  const body = new RigidBody();
  body.name = "queen";
  body.setMaterial({ staticFriction: 0.35, dynamicFriction: 0.35, restitution: 0 });
  const material = new MeshStandardMaterial({
    color: new Color("#a07060"),
    map,
    normalMap,
    aoMap: packed,
    roughnessMap: packed,
    roughness: 0.8,
    metalnessMap: packed,
    metalness: 1,
  });
  const mesh = new Mesh(geometry, material);
  mesh.castShadow = mesh.receiveShadow = true;
  const collider = new MeshCollider({ approximation: "trimesh" }).setGeometry(shape);
  collider.setMaterial({ density: 700 });
  const weight = new BoxCollider({ size: [0.012, 0.006, 0.012] });
  weight.position.y = 0.005;
  weight.setMaterial({ density: 11000 });
  body.add(mesh, collider, weight);
  return body;
}
