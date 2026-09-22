import { Euler, Matrix4, Quaternion, Vector3 } from "three";

/** Three.js forward -Z, left -X, up +Y -> ROS forward +X, left +Y, up +Z. */
export function rosBasis(): Matrix4 {
  return new Matrix4().set(0, 0, -1, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1);
}
const basis = new Quaternion().setFromRotationMatrix(rosBasis());
const inverse = basis.clone().invert();
// A Three.js camera looks down -Z with +Y up; a ROS optical frame looks down +Z with +Y down: a half turn about X.
const optical = new Quaternion(1, 0, 0, 0);

export function rosPosition(value: Vector3): Vector3 {
  return new Vector3(-value.z, -value.x, value.y);
}
export function rosRotation(value: Quaternion): Quaternion {
  return basis.clone().multiply(value).multiply(inverse);
}
export function rosOpticalRotation(value: Quaternion): Quaternion {
  return basis.clone().multiply(value).multiply(optical);
}
/** A Three.js pose as a ROS position and rotation; `optical` gives a camera the ROS optical convention. */
export function rosPose(
  matrix: Matrix4,
  optical = false,
): { position: Vector3; rotation: Quaternion } {
  const position = new Vector3();
  const rotation = new Quaternion();
  const scale = new Vector3();
  matrix.decompose(position, rotation, scale);
  if ([scale.x, scale.y, scale.z].some((value) => Math.abs(value - 1) > 1e-6))
    throw new Error("ROS poses must not be scaled");
  return {
    position: rosPosition(position),
    rotation: optical ? rosOpticalRotation(rotation) : rosRotation(rotation),
  };
}
/** URDF fixed-axis roll, pitch and yaw of a ROS rotation: R = Rz(yaw) · Ry(pitch) · Rx(roll). */
export function rollPitchYaw(value: Quaternion): [number, number, number] {
  const euler = new Euler().setFromQuaternion(value, "ZYX");
  return [euler.x, euler.y, euler.z];
}
