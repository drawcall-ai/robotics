// The one piece of the real scene: a dark wooden Staunton queen, the CC0 set's queen scaled to
// the measured size (7.55 cm long, 2.9 cm across the base).
import { BoxCollider, MeshCollider, RigidBody } from "@drawcall/physics";
import { Box3, Color, Mesh, MeshStandardMaterial } from "three";
import { chessSet, closed, flatBase, texture } from "./assets.ts";

export const QUEEN_LENGTH = 0.0755;
export const QUEEN_BASE = 0.029;

const geometry = (await chessSet()).queen!;
const bounds = new Box3().setFromBufferAttribute(
  geometry.getAttribute("position") as never,
);
const sx = QUEEN_BASE / (bounds.max.x - bounds.min.x);
const sy = QUEEN_LENGTH / bounds.max.y;
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
  mesh.scale.set(sx, sy, sx);
  mesh.castShadow = mesh.receiveShadow = true;
  const collider = new MeshCollider({ approximation: "trimesh" }).setGeometry(shape);
  collider.scale.set(sx, sy, sx);
  collider.setMaterial({ density: 700 });
  const weight = new BoxCollider({ size: [0.012, 0.006, 0.012] });
  weight.position.y = 0.005;
  weight.setMaterial({ density: 11000 });
  body.add(mesh, collider, weight);
  return body;
}
