// Textures, lighting and the chess set, loaded as on a page; the simulator supplies the
// file access and image decoding a browser would.
import {
  BufferGeometry,
  EquirectangularReflectionMapping,
  Float32BufferAttribute,
  ImageBitmapLoader,
  LinearMipmapLinearFilter,
  Mesh,
  NoColorSpace,
  RepeatWrapping,
  SRGBColorSpace,
  Texture,
  Vector3,
} from "three";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";
import { HDRLoader } from "three/addons/loaders/HDRLoader.js";
import { mergeVertices } from "three/addons/utils/BufferGeometryUtils.js";

const url = (path: string) => new URL(path, import.meta.url).href;

/** An image file as a repeating, mipmapped texture; colour maps are sRGB, data maps linear. */
export async function texture(path: string, color: boolean): Promise<Texture> {
  const map = new Texture(await new ImageBitmapLoader().loadAsync(url(path)));
  map.colorSpace = color ? SRGBColorSpace : NoColorSpace;
  map.wrapS = map.wrapT = RepeatWrapping;
  map.minFilter = LinearMipmapLinearFilter;
  map.generateMipmaps = true;
  map.anisotropy = 4;
  map.needsUpdate = true;
  return map;
}

/** A market PBR texture set: base colour, normal, roughness and ambient occlusion. */
export async function material(name: string) {
  const [map, normalMap, roughnessMap, aoMap] = await Promise.all([
    texture(`public/${name}_baseColor.png`, true),
    texture(`public/${name}_normal.png`, false),
    texture(`public/${name}_roughness.png`, false),
    texture(`public/${name}_ambientOcclusion.png`, false),
  ]);
  return { map, normalMap, roughnessMap, aoMap };
}

/** An equirectangular HDR image for image-based lighting. */
export async function environmentMap(path: string): Promise<Texture> {
  const map = await new HDRLoader().loadAsync(url(path));
  map.mapping = EquirectangularReflectionMapping;
  return map;
}

/**
 * The black pieces of Poly Haven's CC0 chess set, one geometry per kind, in the set's own
 * metres (58 mm squares) with each piece's base centred on its origin.
 */
export async function chessSet(): Promise<Record<string, BufferGeometry>> {
  const result = await new GLTFLoader().loadAsync(
    url("assets/chess/chess_set_1k.gltf"),
  );
  const pieces: Record<string, BufferGeometry> = {};
  result.scene.updateMatrixWorld(true);
  result.scene.traverse((node) => {
    const match = /^piece_(\w+)_black/.exec(node.name);
    if (!match || pieces[match[1]!]) return;
    let mesh: Mesh | undefined;
    node.traverse((child) => {
      if (!mesh && child instanceof Mesh) mesh = child;
    });
    if (!mesh) return;
    const geometry = mesh.geometry.clone().applyMatrix4(mesh.matrixWorld);
    geometry.computeBoundingBox();
    const box = geometry.boundingBox!;
    geometry.translate(
      -(box.min.x + box.max.x) / 2,
      -box.min.y,
      -(box.min.z + box.max.z) / 2,
    );
    pieces[match[1]!] = geometry;
  });
  return pieces;
}

/**
 * A closed copy of a render mesh for collision: positions only, seams welded, and any hole
 * capped with a fan around its centre, wound against the triangles around it. Convex
 * decomposition needs a closed surface; one of the set's pieces has an open hole.
 */
export function closed(geometry: BufferGeometry): BufferGeometry {
  const bare = new BufferGeometry().setAttribute(
    "position",
    geometry.getAttribute("position"),
  );
  bare.setIndex(geometry.index);
  const welded = mergeVertices(bare, 5e-5);
  const index = Array.from(welded.index!.array);
  const points = Array.from(welded.getAttribute("position").array);
  const edges = new Set<string>();
  for (let i = 0; i < index.length; i += 3)
    for (let k = 0; k < 3; k++)
      edges.add(`${index[i + k]},${index[i + ((k + 1) % 3)]}`);
  // An edge a->b without a b->a twin borders a hole; the cap runs b->a.
  const next = new Map<number, number>();
  for (const edge of edges) {
    const [a, b] = edge.split(",").map(Number) as [number, number];
    if (!edges.has(`${b},${a}`)) next.set(b, a);
  }
  const done = new Set<number>();
  for (const start of next.keys()) {
    if (done.has(start)) continue;
    const loop = [start];
    for (let v = next.get(start)!; v !== start; v = next.get(v)!) loop.push(v);
    for (const v of loop) done.add(v);
    const center = points.length / 3;
    for (let k = 0; k < 3; k++)
      points.push(
        loop.reduce((sum, v) => sum + points[3 * v + k]!, 0) / loop.length,
      );
    for (const [i, v] of loop.entries())
      index.push(v, loop[(i + 1) % loop.length]!, center);
  }
  return new BufferGeometry()
    .setAttribute("position", new Float32BufferAttribute(points, 3))
    .setIndex(index);
}

/**
 * A copy of a piece's collision mesh standing on an exactly flat base, as a real piece's
 * felted base is. The model's base is flat only to a few tenths of a millimetre, and a contact
 * solver rocks a body on such a face: MuJoCo's convex collision picks a different contact set
 * from step to step, so the piece trembles and creeps across the board. The base is the
 * region of faces facing down that holds the lowest point; it is pressed onto that point's
 * plane.
 */
export function flatBase(geometry: BufferGeometry): BufferGeometry {
  const flat = geometry.clone();
  const position = flat.getAttribute("position");
  const index = flat.index!.array;
  const [a, b, c, normal] = [
    new Vector3(),
    new Vector3(),
    new Vector3(),
    new Vector3(),
  ];
  const down: number[][] = [];
  for (let i = 0; i < index.length; i += 3) {
    const t = [index[i]!, index[i + 1]!, index[i + 2]!];
    a.fromBufferAttribute(position, t[0]!);
    b.fromBufferAttribute(position, t[1]!);
    c.fromBufferAttribute(position, t[2]!);
    normal.crossVectors(b.sub(a), c.sub(a)).normalize();
    // Facing the table, within 25 degrees.
    if (normal.y < -0.9) down.push(t);
  }
  let lowest = 0;
  for (let i = 1; i < position.count; i++)
    if (position.getY(i) < position.getY(lowest)) lowest = i;
  const base = new Set([lowest]);
  for (let grown = true; grown;) {
    grown = false;
    for (const t of down)
      if (t.some((v) => base.has(v)) && !t.every((v) => base.has(v))) {
        for (const v of t) base.add(v);
        grown = true;
      }
  }
  const y = position.getY(lowest);
  for (const v of base) position.setY(v, y);
  return flat;
}
