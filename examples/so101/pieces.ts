import { BoxCollider, MeshCollider, RigidBody } from "@drawcall/physics";
import { Box3, BufferGeometry, Color, Mesh, MeshStandardMaterial } from "three";
import { chessSet, closed, flatBase, texture } from "./assets.ts";

export const kinds = [
  "pawn",
  "rook",
  "knight",
  "bishop",
  "queen",
  "king",
] as const;
export type Kind = (typeof kinds)[number];

// The meshes are a real Staunton set with 58 mm squares; a set for other squares scales them.
const SQUARE = 0.058;
const geometries = (await chessSet()) as Record<Kind, BufferGeometry>;
// The knight's nose points along +X, the way the evaluator reads its facing.
{
  const position = geometries.knight.getAttribute("position");
  const top = new Box3().setFromBufferAttribute(position as never).max.y;
  let nx = 0,
    nz = 0;
  for (let i = 0; i < position.count; i++)
    if (position.getY(i) > 0.6 * top) {
      nx += position.getX(i);
      nz += position.getZ(i);
    }
  geometries.knight.rotateY(Math.atan2(nz, nx));
}
// Each piece collides as its own mesh, which the physics backend splits into convex parts.
const shapes = Object.fromEntries(
  kinds.map((kind) => [kind, flatBase(closed(geometries[kind]))]),
) as Record<Kind, BufferGeometry>;
const [map, packed, normalMap] = await Promise.all([
  texture("assets/chess/chess_set_pieces_black_diff_1k.jpg", true),
  // Occlusion, roughness and metalness in the red, green and blue channels.
  texture("assets/chess/chess_set_pieces_black_arm_1k.jpg", false),
  texture("assets/chess/chess_set_pieces_black_nor_gl_1k.jpg", false),
]);
// glTF texture coordinates start at the image's top row.
for (const t of [map, packed, normalMap]) t.flipY = false;

/** Height of a piece of `kind` from a set for squares of size `s`. */
export function pieceHeight(kind: Kind, s: number): number {
  const geometry = geometries[kind];
  geometry.computeBoundingBox();
  return geometry.boundingBox!.max.y * (s / SQUARE);
}

export interface PieceStyle {
  /** Square size the set is made for. */
  s: number;
  /** Proportions of this set against the reference, around 1. */
  height: number;
  width: number;
  /** Tint over the wood texture, and how glossy the finish is. */
  tint: Color;
  roughness: number;
  friction: number;
}

export function pieceMaterial(style: PieceStyle): MeshStandardMaterial {
  return new MeshStandardMaterial({
    color: style.tint,
    map,
    normalMap,
    aoMap: packed,
    roughnessMap: packed,
    roughness: style.roughness,
    metalnessMap: packed,
    metalness: 1,
  });
}

/**
 * A dynamic chess piece standing on its base at the body origin, +Y up. It collides as its
 * own mesh, and a hidden weight inside the base puts its centre of mass low, as in real
 * weighted pieces.
 */
export function piece(
  kind: Kind,
  style: PieceStyle,
  material: MeshStandardMaterial,
): RigidBody {
  const body = new RigidBody();
  body.name = kind;
  body.setMaterial({
    staticFriction: style.friction,
    dynamicFriction: style.friction,
    restitution: 0,
  });
  const x = (style.s / SQUARE) * style.width;
  const y = (style.s / SQUARE) * style.height;
  const mesh = new Mesh(geometries[kind], material);
  mesh.scale.set(x, y, x);
  mesh.castShadow = mesh.receiveShadow = true;
  const collider = new MeshCollider({ approximation: "trimesh" }).setGeometry(
    shapes[kind],
  );
  collider.scale.set(x, y, x);
  collider.setMaterial({ density: 1200 });
  // The weight: a flat block well inside the base, so it never touches anything itself.
  const box = new Box3().setFromBufferAttribute(
    geometries[kind].getAttribute("position") as never,
  );
  const base = 0.3 * Math.min(box.max.x - box.min.x, box.max.z - box.min.z) * x;
  const weight = new BoxCollider({ size: [base, 0.05 * box.max.y * y, base] });
  weight.position.y = 0.06 * box.max.y * y;
  weight.setMaterial({ density: 8000 });
  body.add(mesh, collider, weight);
  return body;
}
