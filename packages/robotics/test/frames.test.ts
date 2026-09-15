import { expect, it } from "vitest";
import { Euler, Quaternion, Vector3 } from "three";
import { rosPosition, rosRotation, rosOpticalRotation } from "../src/frames.js";
import { Schedule, stamp } from "../src/time.js";

it("maps forward, left and up to ROS axes and preserves heading rotation", () => {
  expect(rosPosition(new Vector3(0, 0, -1))).toEqual({ x: 1, y: -0, z: 0 });
  expect(rosPosition(new Vector3(-1, 0, 0))).toEqual({ x: -0, y: 1, z: 0 });
  expect(rosPosition(new Vector3(0, 1, 0))).toEqual({ x: -0, y: -0, z: 1 });
  const rotation = rosRotation(
    new Quaternion().setFromAxisAngle(new Vector3(0, 1, 0), Math.PI / 2),
  );
  const forward = new Vector3(1, 0, 0).applyQuaternion(
    new Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
  );
  expect(forward.x).toBeCloseTo(0);
  expect(forward.y).toBeCloseTo(1);
  expect(forward.z).toBeCloseTo(0);
});

it("normalizes nanosecond rounding across second boundaries", () => {
  expect(stamp(1.9999999998)).toEqual({ sec: 2, nanosec: 0 });
  expect(stamp(0.0125)).toEqual({ sec: 0, nanosec: 12_500_000 });
  expect(() => stamp(-1)).toThrow();
  expect(() => stamp(Infinity)).toThrow();
});

it("schedules by simulation time without a burst after skipped intervals", () => {
  const schedule = new Schedule(10);
  expect(schedule.due(0.01)).toBe(true);
  expect(schedule.due(0.09)).toBe(false);
  expect(schedule.due(0.1)).toBe(true);
  expect(schedule.due(0.45)).toBe(true);
  expect(schedule.due(0.46)).toBe(false);
  expect(schedule.due(0.5)).toBe(true);
});

it("maps camera optical right, down and forward into the ROS map frame", () => {
  const optical = rosOpticalRotation(new Quaternion());
  const rotation = new Quaternion(optical.x, optical.y, optical.z, optical.w);
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
  const value = rosOpticalRotation(camera);
  const rotation = new Quaternion(value.x, value.y, value.z, value.w);
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
