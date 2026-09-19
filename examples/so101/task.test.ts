import { expect, it } from "vitest";
import { Quaternion, Vector3 } from "three";
import {
  bowlFloor,
  bowlHeight,
  bowlRadius,
  contained,
  count,
  layout,
  objectSize,
} from "./task.ts";

it("samples different reachable layouts without initial overlaps", () => {
  const layouts = Array.from({ length: 1000 }, () => layout());
  expect(new Set(layouts.map((p) => JSON.stringify(p))).size).toBe(1000);
  for (const points of layouts) {
    expect(points).toHaveLength(count + 1);
    for (const [i, p] of points.entries()) {
      expect(Math.hypot(p.x, p.z)).toBeLessThanOrEqual(0.3);
      expect(Math.hypot(p.x, p.z) + p.radius).toBeLessThanOrEqual(0.365);
      for (const q of points.slice(i + 1))
        expect(Math.hypot(p.x - q.x, p.z - q.z)).toBeGreaterThanOrEqual(
          p.radius + q.radius + 0.018,
        );
    }
  }
});
it("counts full containment, rejecting rim, above-bowl and below-floor objects", () => {
  const q = new Quaternion(),
    origin = new Vector3();
  const y = bowlFloor + objectSize / 2;
  expect(contained(new Vector3(0, y, 0), q, origin, q)).toBe(true);
  expect(contained(new Vector3(bowlRadius - 0.001, y, 0), q, origin, q)).toBe(
    false,
  );
  expect(contained(new Vector3(0, bowlHeight, 0), q, origin, q)).toBe(false);
  expect(contained(new Vector3(0, 0, 0), q, origin, q)).toBe(false);
});
it("evaluates in the moved bowl's frame", () => {
  const origin = new Vector3(0.1, 0.74, -0.2);
  const rotation = new Quaternion().setFromAxisAngle(new Vector3(0, 1, 0), 1);
  const p = new Vector3(0.01, bowlFloor + objectSize / 2, 0)
    .applyQuaternion(rotation)
    .add(origin);
  expect(contained(p, rotation, origin, rotation)).toBe(true);
});
