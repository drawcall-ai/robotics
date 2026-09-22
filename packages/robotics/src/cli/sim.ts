import { Command } from "commander";
import { resolve } from "node:path";
import { setTimeout } from "node:timers/promises";
import { pathToFileURL } from "node:url";
import { RclConnection } from "../connections/rcl.js";
import { CameraSensor } from "../devices/camera.js";
import { LaserSensor } from "../devices/laser.js";
import { findRobot, type Robot } from "../devices/robot.js";
import { Evaluator } from "../evaluation.js";
import { Recorder } from "./recorder.js";
import { registry, type PhysicsWorld } from "@drawcall/physics";
import { stamp } from "../messages.js";
import { ROS2 } from "../runtime.js";
import { modelView } from "../urdf/view.js";
import { defaultLayout } from "./layout.js";
import { serveLichtblick } from "./lichtblick.js";

interface SimOptions {
  namespace: string;
  robot?: string;
  viewer?: boolean;
  waitForStart?: boolean;
  perception?: boolean;
  record?: string | boolean;
}

/** The simulator process: a ROS node that runs a scene until the launch stops it. */
export const sim = new Command("sim")
  .description("run the scene as a ROS 2 node until stopped")
  .argument("<scene>", "the scene script")
  .option("--namespace <ns>", "topic namespace", "/robot")
  .option(
    "--robot <name>",
    "the robot to show and record, if the scene has several",
  )
  .option(
    "--viewer",
    "serve Lichtblick with a layout for the scene's topics and open it (BROWSER=none: serve only)",
  )
  .option("--no-viewer", "run without a viewer (default)")
  .option("--wait-for-start", "wait for /sim/start before evaluating")
  .option(
    "--perception",
    "add an image panel for the controller's own view, <namespace>/perception/image/compressed",
  )
  .option(
    "--record [directory]",
    "record one evaluation episode and verify its final result",
  )
  .option("--no-record", "run without recording (default)")
  .action(async (scene: string, options: SimOptions) => {
    await run(scene, options);
    // Dawn keeps process-lifetime native handles; exit only after scoped cleanup.
    process.exit(0);
  });

/** The stop reason of SIGINT and SIGTERM; any other reason is a failure. */
const INTERRUPTED = new Error("interrupted");

async function run(scene: string, options: SimOptions): Promise<void> {
  // MuJoCo and Dawn load here, not at import: the other commands never need them.
  const { buildWorld } = await import("@drawcall/physics-mujoco");
  const { createRenderer } = await import("./webgpu.js");
  await using resources = new AsyncDisposableStack();
  const stop = new AbortController();
  const interrupt = () => stop.abort(INTERRUPTED);
  process.on("SIGINT", interrupt);
  process.on("SIGTERM", interrupt);
  resources.defer(() => {
    process.off("SIGINT", interrupt);
    process.off("SIGTERM", interrupt);
  });
  resources.defer(() => registry.clear());
  resources.defer(() => registry.world?.dispose());
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
  const world = await buildWorld({
    gravity: [0, -9.81, 0],
    fixedDelta: 1 / 1000,
    maxSubsteps: 32,
    solverIterations: 50,
    // Friction constraints stiffer than penetration ones, as MuJoCo Menagerie grippers
    // set them; at the default 1 a grasped object creeps down the pads.
    frictionImpedanceRatio: 10,
    frictionCone: "elliptic",
  });
  ros.attachWorld(world);
  if (stop.signal.aborted) return;
  const robot = findRobot(ros, options.robot);
  robot.beforeStep(world.time);
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
    await recorder.ready(connection, world.time, stop.signal, [
      robot.descriptionTopic,
      "/tf_static",
    ]);
    if (stop.signal.aborted) return;
  }
  ros.publish("/clock", "rosgraph_msgs/msg/Clock", {
    clock: stamp(world.time),
  });
  if (!options.waitForStart) evaluation?.start();
  // Physics waits for camera readbacks, so every frame is captured and published at its own simulation time.
  const stepping = new Stepping(
    world,
    () => (evaluation?.finished ?? false) || ros.busy,
  );
  const controls = serveControls(connection, ros, stepping, evaluation);
  console.error(`Simulating ${scene} under ${ros.namespace}`);
  if (options.viewer)
    serveViewer(ros, robot, evaluation, options.perception, stop, resources);

  const finish = async () => {
    if (recorder && evaluation?.finished)
      await recorder.finish(connection, evaluation);
    if (evaluation?.result)
      console.error(
        `Evaluation ${evaluation.result.status}: ${evaluation.result.reason}`,
      );
  };
  try {
    await simulate(ros, stepping, controls, evaluation, recorder, stop.signal);
    if (stop.signal.aborted && stop.signal.reason !== INTERRUPTED)
      throw stop.signal.reason;
    if (stop.signal.aborted) evaluation?.end("aborted", "interrupted");
  } catch (error) {
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
  }
  await finish();
}

interface Controls {
  paused: boolean;
  /** Single steps requested while paused. */
  steps: number;
}

/** Pause, resume, step and reset plus the evaluation start, the way other simulators expose them. */
function serveControls(
  connection: RclConnection,
  ros: ROS2,
  stepping: Stepping,
  evaluation: Evaluator | undefined,
): Controls {
  const controls: Controls = { paused: false, steps: 0 };
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
    controls.paused = true;
    return { success: true, message: "paused" };
  });
  trigger("resume", () => {
    if (evaluation?.finished)
      return { success: false, message: "episode finished" };
    controls.paused = false;
    controls.steps = 0;
    return { success: true, message: "running" };
  });
  trigger("step", () => {
    if (evaluation?.finished)
      return { success: false, message: "episode finished" };
    if (!controls.paused)
      return { success: false, message: "running; pause first" };
    controls.steps++;
    return { success: true, message: `stepping ${ros.world.fixedDelta} s` };
  });
  trigger("reset", () => {
    if (evaluation)
      return {
        success: false,
        message: "launch a new episode to reset evaluation and recording",
      };
    stepping.clear();
    controls.steps = 0;
    ros.world.reset();
    return {
      success: true,
      message: "world reset; the clock restarts at zero",
    };
  });
  return controls;
}

/**
 * Advances the world by the wall time passed, or by single steps while paused, until the episode
 * finishes or `signal` aborts. A fault ends it with the runtime's error.
 */
async function simulate(
  ros: ROS2,
  stepping: Stepping,
  controls: Controls,
  evaluation: Evaluator | undefined,
  recorder: Recorder | undefined,
  signal: AbortSignal,
): Promise<void> {
  const world = ros.world;
  let last = performance.now();
  for (;;) {
    try {
      await setTimeout(1000 / 60, undefined, { signal });
    } catch (error) {
      if (signal.aborted) return;
      throw error;
    }
    const now = performance.now();
    const delta = (now - last) / 1000;
    last = now;
    try {
      recorder?.check();
      if (!controls.paused) stepping.update(delta);
      while (
        controls.paused &&
        controls.steps > 0 &&
        !evaluation?.finished &&
        !ros.busy
      ) {
        controls.steps--;
        world.update(world.fixedDelta);
      }
    } catch (error) {
      throw ros.error ?? error;
    }
    if (evaluation?.finished) return;
    if (!ros.connected) throw ros.error ?? new Error("ROS connection lost");
  }
}

/** Serves Lichtblick with a layout for the scene; a startup failure stops the simulation with it. */
function serveViewer(
  ros: ROS2,
  robot: Robot,
  evaluation: Evaluator | undefined,
  perception: boolean | undefined,
  stop: AbortController,
  resources: AsyncDisposableStack,
): void {
  const cameras = ros.find(CameraSensor);
  const layout = defaultLayout({
    scans: ros.find(LaserSensor).map((laser) => laser.topic),
    images: [
      ...cameras.map((camera) => camera.imageTopic),
      ...(perception ? [ros.topic("perception/image/compressed")] : []),
    ],
    calibrations: cameras.map((camera) => camera.infoTopic),
    evaluation: evaluation !== undefined,
    model: modelView(robot),
  });
  const cancel = new AbortController();
  const startup = serveLichtblick(layout, undefined, cancel.signal);
  startup.catch((error: unknown) => stop.abort(error));
  resources.defer(async () => {
    cancel.abort();
    try {
      await startup;
    } catch (error) {
      // The failure that stopped the simulation is already reported; cancellation is not a failure.
      if (error === stop.signal.reason || error === cancel.signal.reason)
        return;
      if (error instanceof Error && error.cause === cancel.signal.reason)
        return;
      throw error;
    }
  });
}

/** Keep the accumulator outside physics so a terminal step or a busy device interrupts catch-up immediately. */
export class Stepping {
  private accumulated = 0;
  constructor(
    private readonly world: PhysicsWorld,
    private readonly halted: () => boolean,
  ) {}
  update(delta: number): void {
    this.accumulated = Math.min(
      this.accumulated + delta,
      this.world.fixedDelta * 32,
    );
    while (
      this.accumulated + 1e-12 >= this.world.fixedDelta &&
      !this.halted()
    ) {
      this.accumulated = Math.max(0, this.accumulated - this.world.fixedDelta);
      this.world.update(this.world.fixedDelta);
    }
  }
  clear(): void {
    this.accumulated = 0;
  }
}
