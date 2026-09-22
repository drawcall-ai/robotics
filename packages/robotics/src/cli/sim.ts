import { Command } from "commander";
import { resolve } from "node:path";
import { setTimeout } from "node:timers/promises";
import { pathToFileURL } from "node:url";
import { RclConnection } from "../connections/rcl.js";
import { CameraSensor } from "../devices/camera.js";
import { LaserSensor } from "../devices/laser.js";
import { Robot } from "../devices/robot.js";
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
  viewer?: boolean;
  waitForStart?: boolean;
  show: string[];
  record?: string | boolean;
  recordExclude?: string;
}

/** The simulator process: a ROS node that runs a scene until the launch stops it. */
export const sim = new Command("sim")
  .description("run the scene as a ROS 2 node until stopped")
  .argument("<scene>", "the scene script")
  .option("--namespace <ns>", "topic namespace", "/robot")
  .option(
    "--viewer",
    "serve Lichtblick with a layout for the scene's topics and open it (BROWSER=none: serve only)",
  )
  .option("--no-viewer", "run without a viewer (default)")
  .option(
    "--wait-for-start",
    "wait for /sim/start_evaluation before evaluating",
  )
  .option(
    "--show <topic>",
    "also display a topic in the viewer: a compressed image in its own panel, anything else in 3D (repeatable)",
    (topic: string, shown: string[]) => [...shown, topic],
    [],
  )
  .option(
    "--record [directory]",
    "record one evaluation episode and verify its final result",
  )
  .option("--no-record", "run without recording (default)")
  .option(
    "--record-exclude <regex>",
    "topics the recording leaves out besides /clock and raw images",
  )
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
  const robots = ros.find(Robot);
  // Descriptions and static transforms go out before the first step, for the recorder to receive.
  for (const robot of robots) robot.beforeStep(world.time);
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
        options.recordExclude,
      )
    : undefined;
  if (recorder) {
    resources.defer(() => recorder.stop());
    world.update(world.fixedDelta);
    await recorder.ready(connection, world.time, stop.signal, [
      ...robots.map((robot) => robot.descriptionTopic),
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
    serveViewer(ros, robots, evaluation, options.show, stop, resources);

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
  /** Step requests that answer once their steps have run. */
  stepped: (() => void)[];
}

// simulation_interfaces constants: Result, SimulationState, SimulatorFeatures and ResetSimulation.
const OK = 1;
const UNSUPPORTED = 0;
const FAILED = 4;
const ALREADY_IN_TARGET_STATE = 101;
const PLAYING = 1;
const PAUSED = 2;
const FEATURES = [20, 24, 25, 26, 31, 32];
const SCOPE_DEFAULT = 0;
const SCOPE_ALL = 255;

function result(code: number, message = "") {
  return { result: { result: code, error_message: message } };
}
/** A number in a service request, such as `state.state`. */
function field(request: unknown, ...path: string[]): number {
  let value = request;
  for (const key of path)
    value =
      typeof value === "object" && value !== null
        ? Reflect.get(value, key)
        : undefined;
  if (typeof value === "bigint") return Number(value);
  if (typeof value !== "number")
    throw new Error(`Request field ${path.join(".")} is missing`);
  return value;
}

/** The ROS simulation_interfaces controls, plus the evaluation start. */
function serveControls(
  connection: RclConnection,
  ros: ROS2,
  stepping: Stepping,
  evaluation: Evaluator | undefined,
): Controls {
  const controls: Controls = { paused: false, steps: 0, stepped: [] };
  const serve = (
    name: string,
    type: string,
    handle: (request: unknown) => unknown,
  ) =>
    connection.service(
      `simulation_interfaces/srv/${type}`,
      `/sim/${name}`,
      handle,
    );
  serve("get_simulator_features", "GetSimulatorFeatures", () => ({
    features: { features: FEATURES, spawn_formats: [], custom_info: "" },
  }));
  serve("get_simulation_state", "GetSimulationState", () => ({
    state: { state: controls.paused ? PAUSED : PLAYING },
    ...result(OK),
  }));
  serve("set_simulation_state", "SetSimulationState", (request) => {
    const state = field(request, "state", "state");
    if (state !== PLAYING && state !== PAUSED)
      return result(UNSUPPORTED, "only playing and paused are supported");
    if (controls.paused === (state === PAUSED))
      return result(ALREADY_IN_TARGET_STATE);
    if (state === PLAYING && evaluation?.finished)
      return result(FAILED, "episode finished");
    stepping.clear();
    controls.paused = state === PAUSED;
    controls.steps = 0;
    return result(OK);
  });
  serve("step_simulation", "StepSimulation", (request) => {
    if (!controls.paused) return result(FAILED, "running; pause first");
    if (evaluation?.finished) return result(FAILED, "episode finished");
    controls.steps += field(request, "steps");
    return new Promise((resolve) =>
      controls.stepped.push(() => resolve(result(OK))),
    );
  });
  serve("reset_simulation", "ResetSimulation", (request) => {
    const scope = field(request, "scope");
    if (scope !== SCOPE_DEFAULT && scope !== SCOPE_ALL)
      return result(UNSUPPORTED, "only a full reset is supported");
    if (evaluation)
      return result(
        FAILED,
        "launch a new episode to reset evaluation and recording",
      );
    stepping.clear();
    controls.steps = 0;
    ros.world.reset();
    return result(OK);
  });
  connection.service("std_srvs/srv/Trigger", "/sim/start_evaluation", () => {
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
      if (controls.steps === 0 || evaluation?.finished)
        for (const answer of controls.stepped.splice(0)) answer();
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
  robots: Robot[],
  evaluation: Evaluator | undefined,
  show: string[],
  stop: AbortController,
  resources: AsyncDisposableStack,
): void {
  const cameras = ros.find(CameraSensor);
  const layout = defaultLayout({
    scans: ros.find(LaserSensor).map((laser) => laser.topic),
    images: cameras.map((camera) => camera.compressedTopic),
    calibrations: cameras.map((camera) => camera.infoTopic),
    show,
    evaluation: evaluation !== undefined,
    models: robots.map(modelView),
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
