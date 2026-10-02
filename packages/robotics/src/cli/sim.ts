import { renameSync, writeFileSync } from "node:fs";
import { Command } from "commander";
import { resolve } from "node:path";
import { setTimeout } from "node:timers/promises";
import { pathToFileURL } from "node:url";
import { RclConnection } from "../connections/rcl.js";
import { JointActuator } from "../devices/actuator.js";
import { CameraSensor } from "../devices/camera.js";
import { LaserSensor } from "../devices/laser.js";
import { Robot } from "../devices/robot.js";
import { Evaluator } from "../evaluation.js";
import { Recorder } from "./recorder.js";
import {
  ALREADY_IN_TARGET_STATE,
  BEHIND,
  FAILED,
  field,
  OK,
  PAUSED,
  PLAYING,
  QUITTING,
  SET_STATE,
  SET_STATE_TYPE,
  START,
  START_TYPE,
  UNSUPPORTED,
} from "./protocol.js";
import { registry, type PhysicsWorld } from "@drawcall/physics";
import { stamp } from "../messages.js";
import { ROS2 } from "../runtime.js";
import { modelView } from "../urdf/view.js";
import { defaultLayout } from "./layout.js";
import { serveLichtblick } from "./lichtblick.js";
import { number } from "./run.js";

interface SimOptions {
  namespace: string;
  viewer?: boolean;
  viewerPort: number;
  bridgePort: number;
  record: string;
  recordAll?: boolean;
  realTimeFactor: number;
}

/**
 * The simulator node that simulation.launch.xml starts: it runs and records a scene's evaluation
 * episode, which waits for /sim/start_evaluation. The scene module may export `show`, extra
 * viewer topics, and `recordExclude`, a regular expression of topics not to record.
 */
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
    "--viewer-port <port>",
    "the port to serve Lichtblick on (default: any free one)",
    number(0, true),
    0,
  )
  .option(
    "--bridge-port <port>",
    "the foxglove_bridge port Lichtblick connects to",
    number(0, true),
    8765,
  )
  .requiredOption("--record <directory>", "the rosbag2 directory to record to")
  .option("--record-all", "also record raw images")
  .option("--no-record-all", "leave raw images out (default)")
  .option(
    "--real-time-factor <factor>",
    "simulated seconds per wall-clock second; 0 runs as fast as the host allows",
    number(0),
    1,
  )
  .action(async (scene: string, options: SimOptions) => {
    await run(scene, options);
    // Dawn keeps process-lifetime native handles; exit only after scoped cleanup.
    process.exit(0);
  });

/** Simulated seconds an evaluation may fall behind its factor, for hiccups such as a recompile. */
const LAG = 2;

/** The stop reason of SIGINT and SIGTERM; any other reason is a failure. */
const INTERRUPTED = new Error("interrupted");
/** Wall-clock milliseconds a paused simulation waits for its next turn before it quits. */
const IDLE = 30 * 60 * 1000;

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
  const { show, recordExclude } = declared(
    await import(pathToFileURL(resolve(scene)).href),
  );
  const world = await buildWorld({
    gravity: [0, -9.81, 0],
    // MuJoCo's default timestep.
    fixedDelta: 1 / 500,
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
  if (!evaluation) throw new Error("The scene needs an Evaluator");
  const recorder = new Recorder(
    options.record,
    recordExclude,
    options.recordAll,
  );
  resources.defer(() => recorder.stop());
  world.update(world.fixedDelta);
  await recorder.ready(connection, world.time, stop.signal, [
    ...robots.map((robot) => robot.descriptionTopic),
    "/tf_static",
  ]);
  if (stop.signal.aborted) return;
  ros.publish("/clock", "rosgraph_msgs/msg/Clock", {
    clock: stamp(world.time),
  });
  // Physics waits for camera readbacks, so every frame is captured and published at its own simulation time.
  const stepping = new Stepping(world, () => evaluation.finished || ros.busy);
  const controls = serveControls(
    connection,
    ros,
    stepping,
    evaluation,
    recorder,
  );
  console.error(`Simulating ${scene} under ${ros.namespace}`);
  if (options.viewer) serveViewer(ros, robots, show, options, stop, resources);

  const finish = async () => {
    if (evaluation.finished) await recorder.finish(connection, evaluation);
    if (evaluation.result) {
      console.error(
        `Evaluation ${evaluation.result.status}: ${evaluation.result.reason}`,
      );
      // For a runner of many episodes: the final result, written whole or not at all.
      const file = process.env.DRAWCALL_RESULT;
      if (file) {
        writeFileSync(`${file}.tmp`, JSON.stringify(evaluation.result));
        renameSync(`${file}.tmp`, file);
      }
    }
  };
  try {
    await simulate(
      ros,
      stepping,
      controls,
      evaluation,
      recorder,
      options.realTimeFactor,
      stop.signal,
    );
    if (stop.signal.aborted && stop.signal.reason !== INTERRUPTED)
      throw stop.signal.reason;
    if (stop.signal.aborted) evaluation.end("aborted", "interrupted");
  } catch (error) {
    try {
      evaluation.end(
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

/** What the scene module declares for the viewer and the recording. */
function declared(scene: Record<string, unknown>): {
  show: string[];
  recordExclude?: string;
} {
  const { show = [], recordExclude } = scene;
  if (!Array.isArray(show) || !show.every((topic) => typeof topic === "string"))
    throw new Error("A scene's show export must be a list of topics");
  if (recordExclude !== undefined && typeof recordExclude !== "string")
    throw new Error("A scene's recordExclude export must be a regex string");
  return { show, recordExclude };
}

interface Controls {
  paused: boolean;
  /** When the simulation last paused, in performance.now() milliseconds. */
  pausedAt: number;
  /** Set by QUITTING: the episode ends as aborted and the simulator exits. */
  quit: boolean;
  /** Single steps requested while paused. */
  steps: number;
  /** Step requests that answer once their steps have run. */
  stepped: (() => void)[];
}

// simulation_interfaces constants: SimulatorFeatures and ResetSimulation.
const FEATURES = [20, 24, 25, 26, 31, 32];
const SCOPE_DEFAULT = 0;
const SCOPE_ALL = 255;

function result(code: number, message = "") {
  return { result: { result: code, error_message: message } };
}

/** The ROS simulation_interfaces controls, plus the evaluation start. */
function serveControls(
  connection: RclConnection,
  ros: ROS2,
  stepping: Stepping,
  evaluation: Evaluator,
  recorder: Recorder,
): Controls {
  const controls: Controls = {
    paused: false,
    pausedAt: 0,
    quit: false,
    steps: 0,
    stepped: [],
  };
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
  connection.service(SET_STATE_TYPE, SET_STATE, async (request) => {
    const state = field(request, "state", "state");
    if (state === QUITTING) {
      controls.quit = true;
      return result(OK);
    }
    if (state !== PLAYING && state !== PAUSED)
      return result(
        UNSUPPORTED,
        "only playing, paused and quitting are supported",
      );
    if (controls.paused === (state === PAUSED))
      return result(ALREADY_IN_TARGET_STATE);
    if (evaluation.finished) return result(FAILED, "episode finished");
    stepping.clear();
    controls.paused = state === PAUSED;
    controls.steps = 0;
    // A turn's commands end with it: the robot holds still at a pause, and again at the next
    // turn, in case the last policy's commands arrived after the pause.
    for (const actuator of ros.find(JointActuator)) actuator.release();
    if (state === PLAYING) return result(OK);
    controls.pausedAt = performance.now();
    // Everything up to the pause becomes a closed, readable file.
    try {
      await recorder.split(connection);
    } catch (error) {
      ros.fail(error);
      return result(FAILED, String(error));
    }
    return result(OK);
  });
  serve("step_simulation", "StepSimulation", (request) => {
    if (!controls.paused) return result(FAILED, "running; pause first");
    if (evaluation.finished) return result(FAILED, "episode finished");
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
  connection.service(START_TYPE, START, () => {
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
 * finishes, the simulation quits or `signal` aborts. A fault ends it with the runtime's error.
 */
async function simulate(
  ros: ROS2,
  stepping: Stepping,
  controls: Controls,
  evaluation: Evaluator,
  recorder: Recorder,
  realTimeFactor: number,
  signal: AbortSignal,
): Promise<void> {
  const world = ros.world;
  // Frames of 1/60 s: each steps physics, then sleeps what is left of it, which also lets
  // ROS messages and camera readbacks through.
  const frame = 1000 / 60;
  let last = performance.now();
  for (;;) {
    const now = performance.now();
    const delta = (now - last) / 1000;
    last = now;
    if (controls.paused && now - controls.pausedAt > IDLE) controls.quit = true;
    if (controls.quit) {
      evaluation.end("aborted", "quit");
      return;
    }
    try {
      recorder.check();
      if (!controls.paused) {
        // Catching up on at most two frames' worth, so a host that falls behind stays behind
        // rather than spiralling, whatever the factor.
        if (realTimeFactor > 0)
          stepping.update(
            delta * realTimeFactor,
            (2 * frame * realTimeFactor) / 1000,
          );
        else stepping.until(now + frame);
      }
      while (
        controls.paused &&
        controls.steps > 0 &&
        !evaluation.finished &&
        !ros.busy
      ) {
        controls.steps--;
        world.update(world.fixedDelta);
      }
      if (controls.steps === 0 || evaluation.finished)
        for (const answer of controls.stepped.splice(0)) answer();
      // Results of an episode that falls behind its factor would not mean what was asked, so
      // it ends with an error instead; before the evaluation, catching up is fine.
      if (!evaluation.active) stepping.behind = 0;
      else if (stepping.behind > LAG)
        throw new Error(
          `${BEHIND} ${realTimeFactor}: ${stepping.behind.toFixed(1)} s behind`,
        );
    } catch (error) {
      throw ros.error ?? error;
    }
    if (evaluation.finished) return;
    if (!ros.connected) throw ros.error ?? new Error("ROS connection lost");
    try {
      await setTimeout(
        Math.max(0, frame - (performance.now() - now)),
        undefined,
        {
          signal,
        },
      );
    } catch (error) {
      if (signal.aborted) return;
      throw error;
    }
  }
}

/** Serves Lichtblick with a layout for the scene; a startup failure stops the simulation with it. */
function serveViewer(
  ros: ROS2,
  robots: Robot[],
  show: string[],
  ports: { viewerPort: number; bridgePort: number },
  stop: AbortController,
  resources: AsyncDisposableStack,
): void {
  const cameras = ros.find(CameraSensor);
  const layout = defaultLayout({
    scans: ros.find(LaserSensor).map((laser) => laser.topic),
    images: cameras.map((camera) => camera.compressedTopic),
    calibrations: cameras.map((camera) => camera.infoTopic),
    show,
    models: robots.map(modelView),
  });
  const cancel = new AbortController();
  const startup = serveLichtblick(
    layout,
    { bridge: ports.bridgePort },
    { port: ports.viewerPort, signal: cancel.signal },
  );
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
  /** Simulated seconds the host could not step in time, and dropped. */
  behind = 0;
  constructor(
    private readonly world: PhysicsWorld,
    private readonly halted: () => boolean,
  ) {}
  /** Steps `delta` seconds on, carrying at most `limit`, and at least 32 steps, of debt. */
  update(delta: number, limit = 0): void {
    const owed = this.accumulated + delta;
    this.accumulated = Math.min(
      owed,
      Math.max(limit, this.world.fixedDelta * 32),
    );
    this.behind += owed - this.accumulated;
    while (
      this.accumulated + 1e-12 >= this.world.fixedDelta &&
      !this.halted()
    ) {
      this.accumulated = Math.max(0, this.accumulated - this.world.fixedDelta);
      this.world.update(this.world.fixedDelta);
    }
  }
  /** Steps as many fixed steps as fit before `deadline`, in performance.now() milliseconds. */
  until(deadline: number): void {
    while (performance.now() < deadline && !this.halted())
      this.world.update(this.world.fixedDelta);
  }
  clear(): void {
    this.accumulated = 0;
  }
}
