import { expect, it } from "vitest";
import { header, identifier, stamp } from "../src/messages.js";

it("stamps simulation time in whole seconds and nanoseconds", () => {
  expect(stamp(1.9999999998)).toEqual({ sec: 2, nanosec: 0 });
  expect(stamp(0.0125)).toEqual({ sec: 0, nanosec: 12_500_000 });
  expect(header(0.5, "laser")).toEqual({
    stamp: { sec: 0, nanosec: 500_000_000 },
    frame_id: "laser",
  });
  expect(() => stamp(-1)).toThrow("finite and nonnegative");
  expect(() => stamp(Infinity)).toThrow("finite and nonnegative");
});

it("accepts only ROS name segments as identifiers", () => {
  expect(identifier("frame", "laser")).toBe("laser");
  expect(identifier("link", "base_link")).toBe("base_link");
  for (const bad of ["", "/laser", "cam/optical", "1st"]) {
    expect(() => identifier("frame", bad)).toThrow(
      `Invalid frame name: ${bad}`,
    );
    expect(() => identifier("link", bad)).toThrow(`Invalid link name: ${bad}`);
  }
});
