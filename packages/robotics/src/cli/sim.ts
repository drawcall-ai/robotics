import { setupWorld } from "@drawcall/physics-rapier";
import { RclConnection } from "../rcl.js";
import { findRobot } from "../robot.js";
import { ROS2 } from "../runtime.js";
import { CameraSensor } from "../sensors/camera.js";
import { LaserSensor } from "../sensors/laser.js";
import { SimulationControl } from "./control.js";
import { createRenderer } from "./dawn.js";
import { defaultLayout } from "./layout.js";
import { runScene } from "./scene.js";

export interface SimOptions {
  scene: string;
  namespace?: string;
  robot?: string;
  viewer: boolean;
}

/** The simulator process: a ROS node that runs a scene until the launch stops it. */
export async function sim(options: SimOptions): Promise<void> {
  // Physics first: instantiating Rapier's wasm after the Dawn renderer exists crashes the process.
  await setupWorld({
    gravity: [0, -9.81, 0],
    fixedDelta: 1 / 120,
    maxSubsteps: 16,
    solverIterations: 12,
  });
  const renderer = await createRenderer();
  const connection = await RclConnection.create("sim");
  const ros = new ROS2(connection, { renderer, namespace: options.namespace });
  await runScene(options.scene);
  const control = new SimulationControl(
    ros,
    connection.node,
    findRobot(ros, options.robot).base,
  );
  console.error(`Simulating ${options.scene} under ${ros.namespace}`);

  let viewer: { url: string; close(): void } | undefined;
  let last = performance.now();
  let reported = last;
  const timer = setInterval(() => {
    const now = performance.now();
    try {
      control.advance(Math.min((now - last) / 1000, 0.25));
      if (now - reported >= 100) {
        control.publishState();
        reported = now;
      }
    } catch (error) {
      stop(ros.error ?? error);
      return;
    }
    last = now;
    if (!ros.connected) stop(ros.error ?? new Error("ROS connection lost"));
  }, 1000 / 60);
  function stop(error?: unknown): void {
    clearInterval(timer);
    viewer?.close();
    if (error !== undefined) console.error(error);
    ros.dispose();
    process.exit(error === undefined ? 0 : 1);
  }
  process.once("SIGINT", () => stop());
  process.once("SIGTERM", () => stop());

  if (!options.viewer) return;
  // After the clock runs: the stack waits for the clock, nobody waits for the viewer's first download.
  const { openBrowser, serveLichtblick } = await import("./lichtblick.js");
  viewer = await serveLichtblick(
    defaultLayout(ros.find(LaserSensor), ros.find(CameraSensor)),
  );
  console.error(`Lichtblick: ${viewer.url}`);
  openBrowser(viewer.url);
}
