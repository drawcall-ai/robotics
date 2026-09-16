import { expect, it } from "vitest";
import { Euler, Quaternion, Vector3 } from "three";
import {
  rosPosition,
  rosRotation,
  rosOpticalRotation,
  rollPitchYaw,
} from "../src/coordinates.js";

it("maps forward, left and up to ROS axes and preserves heading rotation", () => {
  expect(rosPosition(new Vector3(0, 0, -1)).toArray()).toEqual([1, -0, 0]);
  expect(rosPosition(new Vector3(-1, 0, 0)).toArray()).toEqual([-0, 1, 0]);
  expect(rosPosition(new Vector3(0, 1, 0)).toArray()).toEqual([-0, -0, 1]);
  const rotation = rosRotation(
    new Quaternion().setFromAxisAngle(new Vector3(0, 1, 0), Math.PI / 2),
  );
  const forward = new Vector3(1, 0, 0).applyQuaternion(rotation);
  expect(forward.x).toBeCloseTo(0);
  expect(forward.y).toBeCloseTo(1);
  expect(forward.z).toBeCloseTo(0);
});

it("maps camera optical right, down and forward into the ROS map frame", () => {
  const rotation = rosOpticalRotation(new Quaternion());
  const axes = [
    { optical: new Vector3(1, 0, 0), expected: new Vector3(0, -1, 0) },
    { optical: new Vector3(0, 1, 0), expected: new Vector3(0, 0, -1) },
    { optical: new Vector3(0, 0, 1), expected: new Vector3(1, 0, 0) },
  ];
  for (const axis of axes) {
    expect(
      axis.optical.applyQuaternion(rotation).distanceTo(axis.expected),
    ).toBeLessThan(1e-12);
  }
});

it("preserves all optical axes for a camera with combined yaw, pitch and roll", () => {
  const camera = new Quaternion().setFromEuler(
    new Euler(0.35, -0.8, 0.27, "YXZ"),
  );
  const original = camera.clone();
  const rotation = rosOpticalRotation(camera);
  const axes = [
    { optical: new Vector3(1, 0, 0), camera: new Vector3(1, 0, 0) },
    { optical: new Vector3(0, 1, 0), camera: new Vector3(0, -1, 0) },
    { optical: new Vector3(0, 0, 1), camera: new Vector3(0, 0, -1) },
  ];
  for (const axis of axes) {
    const world = axis.camera.applyQuaternion(camera);
    const expected = new Vector3(-world.z, -world.x, world.y);
    expect(
      axis.optical.applyQuaternion(rotation).distanceTo(expected),
    ).toBeLessThan(1e-12);
  }
  expect(camera.equals(original)).toBe(true);
  expect(rotation.length()).toBeCloseTo(1);
});

it("expresses ROS rotations as URDF fixed-axis roll, pitch and yaw", () => {
  const rotation = new Quaternion().setFromEuler(
    new Euler(0.3, -0.5, 0.9, "XYZ"),
  );
  const ros = rosRotation(rotation);
  const [roll, pitch, yaw] = rollPitchYaw(ros);
  const rebuilt = new Quaternion().setFromEuler(
    new Euler(roll, pitch, yaw, "ZYX"),
  );
  expect(rebuilt.angleTo(ros)).toBeLessThan(1e-12);
});
