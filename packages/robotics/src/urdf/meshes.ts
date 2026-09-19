import { BufferGeometry, Color, Matrix4, Mesh, SRGBColorSpace } from "three";
import { STLExporter } from "three/addons/exporters/STLExporter.js";
import { ConvexGeometry } from "three/addons/geometries/ConvexGeometry.js";
import { Vector3 } from "three";
import type { Link } from "./links.js";

const basis = new Matrix4().set(
  0,
  0,
  -1,
  0,
  -1,
  0,
  0,
  0,
  0,
  1,
  0,
  0,
  0,
  0,
  0,
  1,
);

/** Inline STL keeps the description self-contained in ROS and MCAP, without file URLs or an asset server. */
export function meshXML(
  geometry: BufferGeometry,
  pose = new Matrix4(),
  convex = false,
): string {
  const transformed = geometry
    .clone()
    .applyMatrix4(basis.clone().multiply(pose));
  let surface = transformed;
  try {
    if (convex) {
      const positions = transformed.getAttribute("position");
      surface = new ConvexGeometry(
        Array.from({ length: positions.count }, (_, i) =>
          new Vector3().fromBufferAttribute(positions, i),
        ),
      );
    }
    const data = new STLExporter().parse(new Mesh(surface), { binary: true });
    const bytes = new Uint8Array(data.buffer, data.byteOffset, data.byteLength);
    let binary = "";
    for (let offset = 0; offset < bytes.length; offset += 8192)
      binary += String.fromCharCode(...bytes.subarray(offset, offset + 8192));
    return `<mesh filename="data:model/stl;base64,${btoa(binary)}"/>`;
  } finally {
    if (surface !== transformed) surface.dispose();
    transformed.dispose();
  }
}

/** Render meshes are distinct from collision proxies. Their full local transforms are baked into link coordinates. */
export function visualsXML(link: Link): string[] {
  const result: string[] = [];
  const inverse = link.origin
    .clone()
    .invert()
    .multiply(link.body.matrixWorld.clone().invert());
  link.body.traverseVisible((object) => {
    if (!(object instanceof Mesh)) return;
    const material = object.material;
    if (Array.isArray(material))
      throw new Error(
        `URDF visual ${object.name || link.name} needs a single material`,
      );
    if ("map" in material && material.map)
      throw new Error(
        `URDF visual ${object.name || link.name} uses an unsupported texture`,
      );
    const color =
      "color" in material && material.color instanceof Color
        ? material.color
        : new Color("#9aa5ab");
    const rgb = color.getRGB(new Color(), SRGBColorSpace);
    const geometry = meshXML(
      object.geometry,
      inverse.clone().multiply(object.matrixWorld),
    );
    result.push(
      "    <visual>",
      `      <geometry>${geometry}</geometry>`,
      `      <material name="${link.name}_visual_${result.length}"><color rgba="${rgb.r} ${rgb.g} ${rgb.b} ${material.opacity}"/></material>`,
      "    </visual>",
    );
  });
  return result;
}
