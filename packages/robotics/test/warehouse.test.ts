import { readFileSync } from "node:fs";
import { expect, it, vi } from "vitest";
import { buildWorld } from "@drawcall/physics-mujoco";
import { Evaluator } from "../src/evaluation.js";
import { Robot } from "../src/devices/robot.js";
import { CameraSensor } from "../src/devices/camera.js";
import { LaserSensor } from "../src/devices/laser.js";
import { exportURDF } from "../src/urdf/export.js";
import { defer, offline } from "./fixture.js";

// The example imports the published package; test it against the sources instead of dist.
vi.mock("@drawcall/robotics", () => import("../src/index.js"));

// Golden omits binary mesh payloads; mesh geometry is decoded and checked in model.test.ts.
it("exports the warehouse example as its golden URDF and scans on /robot/scan", async () => {
  const ros = offline();
  await import("../../../examples/warehouse/scene.js");
  expect(
    ros.find(CameraSensor).map((camera) => camera.compressedTopic),
  ).toEqual(["/robot/camera/image_raw/compressed"]);
  expect(
    exportURDF({ ros }).replace(
      /data:model\/stl;base64,[^" ]+/g,
      "embedded-stl",
    ),
  ).toBe(readFileSync(new URL("./warehouse.urdf", import.meta.url), "utf8"));
  expect(ros.find(LaserSensor).map((laser) => laser.topic)).toEqual([
    "/robot/scan",
  ]);
  const [evaluation] = ros.find(Evaluator);
  const [robot] = ros.find(Robot);
  if (!evaluation || !robot)
    throw new Error("Example must register its robot and evaluator");
  vi.spyOn(ros.connection, "publish").mockImplementation(() => {});
  const world = await buildWorld();
  defer(() => world.dispose());
  ros.attachWorld(world);
  evaluation.start();
  robot.base.position.z = -2;
  evaluation.evaluate();
  expect(evaluation.result?.metrics.score).toBeCloseTo(12.5);
  robot.base.position.z = -1;
  evaluation.evaluate();
  expect(evaluation.result?.metrics.score).toBeCloseTo(12.5);
});
