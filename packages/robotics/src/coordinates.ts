import { Euler, Matrix4, Quaternion, Vector3 } from "three";

// Three.js forward -Z, left -X, up +Y -> ROS forward +X, left +Y, up +Z.
const basis = new Quaternion().setFromRotationMatrix(
  new Matrix4().set(0, 0, -1, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1),
);
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
/** URDF fixed-axis roll, pitch and yaw of a ROS rotation: R = Rz(yaw) · Ry(pitch) · Rx(roll). */
export function rollPitchYaw(value: Quaternion): [number, number, number] {
  const euler = new Euler().setFromQuaternion(value, "ZYX");
  return [euler.x, euler.y, euler.z];
}
