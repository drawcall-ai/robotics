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

const felt = { staticFriction: 0.2, dynamicFriction: 0.2, restitution: 0 };

/**
 * A dynamic queen standing on its base at the body origin, +Y up, `scale` times the real one's
 * size and tinted `tint` over the dark wood texture. Lacquered wood, about 20 g at scale 1 with a
 * small weight in the base, on a felt pad: friction 0.2, so a push near the head slides it on
 * the board rather than tipping it, as the real queen did (contacts take the larger friction).
 * Each collider carries the friction: a collider's own material replaces the body's whole.
 */
export function queen(scale = 1, tint = new Color("#a07060")): RigidBody {
  const body = new RigidBody();
  body.name = "queen";
  const material = new MeshStandardMaterial({
    color: tint,
    map,
    normalMap,
    aoMap: packed,
    roughnessMap: packed,
    roughness: 0.8,
    metalnessMap: packed,
    metalness: 1,
  });
  const mesh = new Mesh(geometry, material);
  mesh.scale.setScalar(scale);
  mesh.castShadow = mesh.receiveShadow = true;
  const collider = new MeshCollider({ approximation: "trimesh" }).setGeometry(shape);
  collider.scale.setScalar(scale);
  collider.setMaterial({ ...felt, density: 700 });
  const weight = new BoxCollider({ size: [0.012 * scale, 0.006 * scale, 0.012 * scale] });
  weight.position.y = 0.005 * scale;
  weight.setMaterial({ ...felt, density: 11000 });
  body.add(mesh, collider, weight);
  return body;
}
