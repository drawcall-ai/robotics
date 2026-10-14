import { readFileSync } from "node:fs";
import { expect, it } from "vitest";
import { GLTSLoader } from "@drawcall/glts";
import * as physics from "@drawcall/physics";
import { buildWorld } from "@drawcall/physics-mujoco";
import { LoadingManager } from "three";
import * as robotics from "../src/index.js";
import { Evaluator } from "../src/evaluation.js";
import { Robot } from "../src/devices/robot.js";
import { CameraSensor } from "../src/devices/camera.js";
import { LaserSensor } from "../src/devices/laser.js";
import { exportURDF } from "../src/urdf/export.js";
import { ROS2 } from "../src/runtime.js";
import { ConnectionStub, defer } from "./fixture.js";

// Golden omits binary mesh payloads; mesh geometry is decoded and checked in model.test.ts.
it("exports the warehouse example as its golden URDF and scans on /robot/scan", async () => {
  // The example imports the published package; test it against the sources instead of dist.
  const loader = new GLTSLoader(new LoadingManager(), {
    modules: { "@drawcall/physics": physics, "@drawcall/robotics": robotics },
  });
  defer(() => loader.dispose());
  const scene = await loader.loadAsync(
    new URL("../../../examples/warehouse/scene.glts", import.meta.url),
  );
  expect(
    exportURDF(scene).replace(/data:model\/stl;base64,[^" ]+/g, "embedded-stl"),
  ).toBe(readFileSync(new URL("./warehouse.urdf", import.meta.url), "utf8"));
  const world = await buildWorld({ scene });
  defer(() => world.dispose());
  const ros = new ROS2(new ConnectionStub(), { scene, world });
  defer(() => ros.dispose());
  ros.sync();
  expect(
    ros.find(CameraSensor).map((camera) => camera.compressedTopic),
  ).toEqual(["/robot/camera/image_raw/compressed"]);
  expect(ros.find(LaserSensor).map((laser) => laser.topic)).toEqual([
    "/robot/scan",
  ]);
  const [evaluation] = ros.find(Evaluator);
  const [robot] = ros.find(Robot);
  if (!evaluation || !robot)
    throw new Error("Example must hold its robot and evaluator");
  evaluation.start();
  robot.base.position.z = -2;
  evaluation.evaluate();
  expect(evaluation.result?.metrics.score).toBeCloseTo(12.5);
  robot.base.position.z = -1;
  evaluation.evaluate();
  expect(evaluation.result?.metrics.score).toBeCloseTo(12.5);
});
