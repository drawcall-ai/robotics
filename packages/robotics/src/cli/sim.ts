import { Command } from "commander";
import { resolve } from "node:path";
import { pathToFileURL } from "node:url";
import { Quaternion, Vector3 } from "three";
import { RclConnection } from "../connections/rcl.js";
import { rollPitchYaw, rosPosition, rosRotation } from "../coordinates.js";
import { CameraSensor } from "../devices/camera.js";
import { LaserSensor } from "../devices/laser.js";
import { findRobot } from "../devices/robot.js";
import { ROS2 } from "../runtime.js";
import { defaultLayout } from "./layout.js";
import { serveLichtblick } from "./lichtblick.js";

/** The simulator process: a ROS node that runs a scene until the launch stops it. */
export const sim = new Command("sim")
  .description("run the scene as a ROS 2 node until stopped")
  .argument("<scene>", "the scene script")
  .option("--namespace <ns>", "topic namespace", "/robot")
  .option(
    "--robot <name>",
    "the robot to report on /sim/state, if the scene has several",
  )
  .option(
    "--viewer",
    "serve Lichtblick with a layout for the scene's topics and open it (BROWSER=none: serve only)",
  )
  .option("--no-viewer", "run without a viewer (default)")
  .action(
    async (
      scene: string,
      options: { namespace: string; robot?: string; viewer?: boolean },
    ) => {
      await run(scene, options);
      // Dawn keeps process-lifetime native handles; exit only after scoped cleanup.
      process.exit(0);
    },
  );

async function run(
  scene: string,
  options: { namespace: string; robot?: string; viewer?: boolean },
): Promise<void> {
  // Rapier and Dawn load here, not at import: the other commands never need them.
  // Physics first: instantiating Rapier's wasm after the Dawn renderer exists crashes the process.
  const { setupWorld } = await import("@drawcall/physics-rapier");
  const { createRenderer } = await import("./webgpu.js");
  const world = await setupWorld({
    gravity: [0, -9.81, 0],
    fixedDelta: 1 / 120,
    maxSubsteps: 16,
    solverIterations: 12,
  });
  await using resources = new AsyncDisposableStack();
  resources.defer(() => world.dispose());
  const renderer = await createRenderer();
  resources.defer(() => renderer.dispose());
  const connection = await RclConnection.create("sim");
  let ros: ROS2;
  {
    using setup = new DisposableStack();
    setup.defer(() => connection.dispose());
    ros = new ROS2(connection, {
      renderer,
      namespace: options.namespace,
    });
    resources.defer(() => ros.disposeAsync());
    setup.move();
  }
  // The scene builds against the current runtime and registers its devices there.
  await import(pathToFileURL(resolve(scene)).href);
  const base = findRobot(ros, options.robot).base;
  console.error(`Simulating ${scene} under ${ros.namespace}`);

  // Pause, resume, step and reset plus a ground-truth state topic, the way other simulators expose them.
  let paused = false;
  let pendingSteps = 0;
  const trigger = (
    name: string,
    run: () => { success: boolean; message: string },
  ) => connection.service("std_srvs/srv/Trigger", `/sim/${name}`, run);
  trigger("pause", () => {
    paused = true;
    return { success: true, message: "paused" };
  });
  trigger("resume", () => {
    paused = false;
    pendingSteps = 0;
    return { success: true, message: "running" };
  });
  trigger("step", () => {
    if (!paused) return { success: false, message: "running; pause first" };
    pendingSteps++;
    return { success: true, message: `stepping ${world.fixedDelta} s` };
  });
  trigger("reset", () => {
    world.reset();
    return {
      success: true,
      message: "world reset; the clock restarts at zero",
    };
  });
  /** Ground truth for episode logic; navigation must never subscribe to it. */
  function publishState(): void {
    const position = rosPosition(base.getWorldPosition(new Vector3()));
    const [, , yaw] = rollPitchYaw(
      rosRotation(base.getWorldQuaternion(new Quaternion())),
    );
    ros.publish("/sim/state", "std_msgs/msg/String", {
      data: JSON.stringify({
        time: world.time,
        paused,
        base: { x: position.x, y: position.y, yaw },
      }),
    });
  }

  using activity = new DisposableStack();
  const stopped = new Promise<void>((resolve, reject) => {
    const stop = () => resolve();
    process.once("SIGINT", stop);
    process.once("SIGTERM", stop);
    activity.defer(() => {
      process.off("SIGINT", stop);
      process.off("SIGTERM", stop);
    });
    let last = performance.now();
    let reported = last;
    /** Advances the world by the wall time passed unless paused, when only single steps run; state reports on wall time so a paused world still reports. */
    function tick(): void {
      const now = performance.now();
      const delta = (now - last) / 1000;
      last = now;
      try {
        if (!paused) world.update(delta);
        while (paused && pendingSteps > 0) {
          pendingSteps--;
          world.update(world.fixedDelta);
        }
        if (now - reported >= 100) {
          publishState();
          reported = now;
        }
      } catch (error) {
        reject(ros.error ?? error);
        return;
      }
      if (!ros.connected) reject(ros.error ?? new Error("ROS connection lost"));
    }
    const timer = setInterval(tick, 1000 / 60);
    activity.defer(() => clearInterval(timer));
  });

  if (!options.viewer) {
    await stopped;
    return;
  }
  const viewer = new AbortController();
  const startup = serveLichtblick(
    defaultLayout(
      ros.find(LaserSensor).map((laser) => laser.topic),
      ros.find(CameraSensor).map((camera) => camera.imageTopic),
    ),
    undefined,
    viewer.signal,
  );
  let failure: { error: unknown } | undefined;
  resources.defer(async () => {
    viewer.abort();
    try {
      await startup;
    } catch (error) {
      if (failure && error === failure.error) return;
      if (error === viewer.signal.reason) return;
      if (error instanceof Error && error.cause === viewer.signal.reason)
        return;
      throw error;
    }
  });
  try {
    await Promise.race([startup.then(() => stopped), stopped]);
  } catch (error) {
    failure = { error };
    throw error;
  }
}
