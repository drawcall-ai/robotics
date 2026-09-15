import { Matrix4, Quaternion, Vector3 } from "three";

// Three.js forward -Z, left -X, up +Y -> ROS forward +X, left +Y, up +Z.
const basis = new Quaternion().setFromRotationMatrix(
  new Matrix4().set(0, 0, -1, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1),
);
export function rosPosition(value: Vector3): {
  x: number;
  y: number;
  z: number;
} {
  return { x: -value.z, y: -value.x, z: value.y };
}
export function rosRotation(value: Quaternion): {
  x: number;
  y: number;
  z: number;
  w: number;
} {
  const result = basis.clone().multiply(value).multiply(basis.clone().invert());
  return { x: result.x, y: result.y, z: result.z, w: result.w };
}
export function rosOpticalRotation(value: Quaternion): {
  x: number;
  y: number;
  z: number;
  w: number;
} {
  const result = basis
    .clone()
    .multiply(value)
    .multiply(new Quaternion(1, 0, 0, 0));
  return { x: result.x, y: result.y, z: result.z, w: result.w };
}
