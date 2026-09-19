import { Command } from "commander";
import { resolve } from "node:path";
import { pathToFileURL } from "node:url";
import { Quaternion, Vector3 } from "three";
import { RclConnection } from "../connections/rcl.js";
import { rollPitchYaw, rosPosition, rosRotation } from "../coordinates.js";
import { CameraSensor } from "../devices/camera.js";
import { LaserSensor } from "../devices/laser.js";
import { findRobot } from "../devices/robot.js";
import { Evaluator } from "../evaluation.js";
import { Recorder } from "./recorder.js";
import type { PhysicsWorld } from "@drawcall/physics";
import { stamp } from "../messages.js";
import { ROS2 } from "../runtime.js";
import { modelView } from "../urdf/view.js";
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
  .option("--wait-for-start", "wait for /sim/start before evaluating")
  .option(
    "--record [directory]",
    "record one evaluation episode and verify its final result",
  )
  .action(
    async (
      scene: string,
      options: {
        namespace: string;
        robot?: string;
        viewer?: boolean;
        waitForStart?: boolean;
        record?: string | boolean;
      },
    ) => {
      await run(scene, options);
      // Dawn keeps process-lifetime native handles; exit only after scoped cleanup.
      process.exit(0);
    },
  );

async function run(
  scene: string,
  options: {
    namespace: string;
    robot?: string;
    viewer?: boolean;
    waitForStart?: boolean;
    record?: string | boolean;
  },
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
  const interruption = new AbortController();
  const interrupt = () => interruption.abort();
  process.on("SIGINT", interrupt);
  process.on("SIGTERM", interrupt);
  resources.defer(() => {
    process.off("SIGINT", interrupt);
    process.off("SIGTERM", interrupt);
  });
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
  if (interruption.signal.aborted) return;
  const robot = findRobot(ros, options.robot);
  robot.beforeStep(world.time);
  const view = modelView(robot);
  const base = robot.base;
  const [evaluation] = ros.find(Evaluator);
  if ((options.record || options.waitForStart) && !evaluation)
    throw new Error(
      "Recording and deferred evaluation require an Evaluator in the scene",
    );
  const recorder = options.record
    ? new Recorder(
        typeof options.record === "string"
          ? options.record
          : `rosbag2_${new Date().toISOString().replaceAll(":", "-")}`,
      )
    : undefined;
  if (recorder) {
    resources.defer(() => recorder.stop());
    world.update(world.fixedDelta);
    await recorder.ready(connection, world.time, interruption.signal, [
      robot.descriptionTopic,
      "/tf_static",
    ]);
    if (interruption.signal.aborted) return;
  }
  ros.publish("/clock", "rosgraph_msgs/msg/Clock", {
    clock: stamp(world.time),
  });
  if (!options.waitForStart) evaluation?.start();
  const stepping = new Stepping(world, () => evaluation?.finished ?? false);
  console.error(`Simulating ${scene} under ${ros.namespace}`);

  // Pause, resume, step and reset plus a ground-truth state topic, the way other simulators expose them.
  let paused = false;
  let pendingSteps = 0;
  const trigger = (
    name: string,
    run: () => { success: boolean; message: string },
  ) => connection.service("std_srvs/srv/Trigger", `/sim/${name}`, run);
  trigger("start", () => {
    if (!evaluation)
      return { success: false, message: "scene has no evaluator" };
    if (evaluation.result)
      return {
        success: false,
        message: "episode already started; launch a new episode",
      };
    try {
      evaluation.start();
      return { success: true, message: "evaluation started" };
    } catch (error) {
      ros.fail(error);
      return { success: false, message: String(error) };
    }
  });
  trigger("pause", () => {
    stepping.clear();
    paused = true;
    return { success: true, message: "paused" };
  });
  trigger("resume", () => {
    if (evaluation?.finished)
      return { success: false, message: "episode finished" };
    paused = false;
    pendingSteps = 0;
    return { success: true, message: "running" };
  });
  trigger("step", () => {
    if (evaluation?.finished)
      return { success: false, message: "episode finished" };
    if (!paused) return { success: false, message: "running; pause first" };
    pendingSteps++;
    return { success: true, message: `stepping ${world.fixedDelta} s` };
  });
  trigger("reset", () => {
    if (evaluation)
      return {
        success: false,
        message: "launch a new episode to reset evaluation and recording",
      };
    stepping.clear();
    pendingSteps = 0;
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
  let ended = false;
  let rejectStopped: ((error: unknown) => void) | undefined;
  const stopped = new Promise<void>((resolve, reject) => {
    rejectStopped = reject;
    const stop = () => {
      ended = true;
      try {
        evaluation?.end("aborted", "interrupted");
        resolve();
      } catch (error) {
        reject(error);
      }
    };
    interruption.signal.addEventListener("abort", stop, { once: true });
    activity.defer(() =>
      interruption.signal.removeEventListener("abort", stop),
    );
    let last = performance.now();
    let reported = last;
    /** Advances the world by the wall time passed unless paused, when only single steps run; state reports on wall time so a paused world still reports. */
    function tick(): void {
      if (ended) return;
      const now = performance.now();
      const delta = (now - last) / 1000;
      last = now;
      try {
        recorder?.check();
        if (!paused) stepping.update(delta);
        while (paused && pendingSteps > 0 && !evaluation?.finished) {
          pendingSteps--;
          world.update(world.fixedDelta);
        }
        if (evaluation?.finished) {
          paused = true;
          publishState();
          ended = true;
          resolve();
          return;
        }
        if (now - reported >= 100) {
          publishState();
          reported = now;
        }
      } catch (error) {
        ended = true;
        reject(ros.error ?? error);
        return;
      }
      if (!ros.connected) {
        ended = true;
        reject(ros.error ?? new Error("ROS connection lost"));
      }
    }
    const timer = setInterval(tick, 1000 / 60);
    activity.defer(() => clearInterval(timer));
  });

  const finish = async () => {
    if (recorder && evaluation?.finished)
      await recorder.finish(connection, evaluation);
    if (evaluation?.result)
      console.error(
        `Evaluation ${evaluation.result.status}: ${evaluation.result.reason}`,
      );
  };
  const fail = async (error: unknown) => {
    try {
      evaluation?.end(
        "error",
        error instanceof Error ? error.message : String(error),
      );
      await finish();
    } catch (finalization) {
      throw new AggregateError(
        [error, finalization],
        "Simulation and evaluation finalization failed",
      );
    }
    throw error;
  };
  const completed = stopped.then(finish, fail);
  if (!options.viewer) {
    await completed;
    return;
  }
  const viewer = new AbortController();
  const startup = serveLichtblick(
    defaultLayout(
      ros.find(LaserSensor).map((laser) => laser.topic),
      ros.find(CameraSensor).map((camera) => camera.imageTopic),
      evaluation !== undefined,
      view,
    ),
    undefined,
    viewer.signal,
  ).catch(async (error: unknown) => {
    if (viewer.signal.aborted) throw error;
    ended = true;
    if (!rejectStopped) throw error;
    rejectStopped(error);
    await completed;
    throw error;
  });
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
    await Promise.race([startup.then(() => completed), completed]);
  } catch (error) {
    failure = { error };
    throw error;
  }
}

/** Keep the accumulator outside physics so a terminal step interrupts catch-up immediately. */
export class Stepping {
  private accumulated = 0;
  constructor(
    private readonly world: PhysicsWorld,
    private readonly finished: () => boolean,
  ) {}
  update(delta: number): void {
    this.accumulated = Math.min(
      this.accumulated + delta,
      this.world.fixedDelta * 16,
    );
    while (
      this.accumulated + 1e-12 >= this.world.fixedDelta &&
      !this.finished()
    ) {
      this.accumulated = Math.max(0, this.accumulated - this.world.fixedDelta);
      this.world.update(this.world.fixedDelta);
    }
  }
  clear(): void {
    this.accumulated = 0;
  }
}
