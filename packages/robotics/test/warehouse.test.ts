import { readFileSync } from "node:fs";
import { expect, it, vi } from "vitest";
import { LaserSensor } from "../src/devices/laser.js";
import { exportURDF } from "../src/urdf/export.js";
import { offline } from "./fixture.js";

// The example imports the published package; test it against the sources instead of dist.
vi.mock("@drawcall/robotics", () => import("../src/index.js"));

// Regenerate the golden from examples/warehouse: node ../../packages/robotics/dist/cli/main.js urdf scene.ts > ../../packages/robotics/test/warehouse.urdf
it("exports the warehouse example as its golden URDF and scans on /robot/scan", async () => {
  const ros = offline();
  await import("../../../examples/warehouse/scene.js");
  expect(exportURDF({ ros })).toBe(
    readFileSync(new URL("./warehouse.urdf", import.meta.url), "utf8"),
  );
  expect(ros.find(LaserSensor).map((laser) => laser.topic)).toEqual([
    "/robot/scan",
  ]);
});
