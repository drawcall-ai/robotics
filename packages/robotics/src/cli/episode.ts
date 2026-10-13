import {
  spawn,
  spawnSync,
  type ChildProcess,
  type StdioOptions,
} from "node:child_process";
import { once } from "node:events";
import {
  closeSync,
  existsSync,
  openSync,
  readFileSync,
  writeFileSync,
} from "node:fs";
import { availableParallelism, setPriority } from "node:os";
import { dirname, join, resolve } from "node:path";
import { setTimeout as sleep } from "node:timers/promises";
import { fileURLToPath } from "node:url";
import { RclConnection } from "../connections/rcl.js";
import type { EvaluationState } from "../evaluation.js";
import { alive } from "./domain.js";
import { UserError } from "./errors.js";
import {
  ALREADY_IN_TARGET_STATE,
  FAILED,
  field,
  get,
  OK,
  PAUSED,
  PLAYING,
  QUITTING,
  SET_STATE,
  SET_STATE_TYPE,
  START,
  START_TYPE,
} from "./protocol.js";

/** The simulation launch this package ships. */
const LAUNCH = fileURLToPath(
  new URL("../../launch/simulation.launch.xml", import.meta.url),
);
/** Wall-clock milliseconds a turn may take before its whole launch is stopped. */
const WATCHDOG = 2 * 60 * 60 * 1000;

/** What every episode of a run shares. */
export interface Setup {
  scene: string;
  bringup: string;
  /** The robot's decision-making, started anew for each turn. */
  policy: Policy;
  realTimeFactor: number;
  /** The ports of Lichtblick and of the foxglove_bridge it connects to, if served. */
  viewer?: { port: number; bridge: number };
  recordAll: boolean;
  git: State["git"];
}

/** A policy launch file, or a single command that replaces it. */
export type Policy = { file: string } | { command: string[] };

/** An episode as the runner knows it: a single run's state.json, and part of every result.json. */
export interface State {
  domain: number;
  /** The simulation launch. */
  pid: number;
  scene: string;
  seed: number;
  /** Whether a later turn continued the episode. */
  continued: boolean;
  /** Wall-clock seconds of the episode's turns. */
  wall: number;
  argv: string[];
  git?: { commit: string; dirty: boolean };
}

/** One episode's line in the table. */
export interface Row {
  seed: number;
  status: string;
  reason: string;
  elapsed?: number;
  score?: number;
  wall: number;
}

/**
 * How a turn runs. A batch episode's policy writes to policy.log, and its turn ends the
 * episode; a single run's policy writes to the terminal, and its turn pauses the episode.
 */
export interface Mode {
  batch: boolean;
  /** Simulated seconds the turn may take. */
  timeout: number;
  /** Episodes side by side, which share the cores. */
  slots: number;
}

/** Why a turn ended. */
type Ending = "simulation" | "policy" | "timeout" | "interrupt" | "watchdog";

/** Aborted by SIGINT or SIGTERM, once `listen` has been called: the turns under way end. */
export const interrupted = new AbortController();

export function listen(): void {
  for (const signal of ["SIGINT", "SIGTERM"] as const)
    process.on(signal, () => interrupted.abort());
}

/** Process groups this process started, for a last resort if it fails. */
const running = new Set<number>();

/** Kills every process group this process started that is still running. */
export function kill(): void {
  for (const leader of running) signal(-leader, "SIGKILL");
}

/**
 * Starts the simulation launch of an episode in `folder`, detached in its own process group so
 * that it outlives a paused run, and returns its pid. Its output goes to launch.log.
 */
export async function launch(
  setup: Setup,
  folder: string,
  seed: number,
  domain: number,
  slots: number,
  keepPace: boolean,
): Promise<number> {
  const log = openSync(join(folder, "launch.log"), "a");
  try {
    const child = await start(
      [
        "ros2",
        "launch",
        LAUNCH,
        `scene:=${setup.scene}`,
        `bringup:=${setup.bringup}`,
        `record:=${resolve(folder, "bag")}`,
        `real_time_factor:=${setup.realTimeFactor}`,
        `keep_pace:=${keepPace}`,
        ...(setup.viewer
          ? [
              "viewer:=true",
              `viewer_port:=${setup.viewer.port}`,
              `bridge_port:=${setup.viewer.bridge}`,
            ]
          : []),
        ...(setup.recordAll ? ["record_all:=true"] : []),
      ],
      // The simulator runs in the scene's folder, which its relative paths expect.
      dirname(setup.scene),
      ["ignore", log, log],
      {
        ...environment(domain, seed, slots),
        DRAWCALL_RESULT: resolve(folder, "result.json"),
      },
    );
    child.unref();
    return pid(child);
  } finally {
    closeSync(log);
  }
}

/**
 * A turn of an episode: the policy runs until the turn ends, then a single run pauses, or the
 * episode ends and its row is returned. Whatever fails on the way stops the episode.
 */
export async function turn(
  state: State,
  folder: string,
  policy: Policy,
  mode: Mode,
): Promise<Row | undefined> {
  const ros = await RclConnection.create(
    `drawcall_robotics_${state.domain}`,
    state.domain,
  );
  let child: ChildProcess | undefined;
  try {
    // A continued episode is paused; a new one waits for its start.
    if (!state.continued) await ready(ros, state.pid, folder);
    child = await play(policy, state, folder, mode);
    if (state.continued) await setState(ros, PLAYING);
    else await begin(ros);
    const started = performance.now();
    const ending = await until(ros, state.pid, child, mode.timeout);
    state.wall += (performance.now() - started) / 1000;
    if (!mode.batch)
      console.error(
        ending === "policy"
          ? `Turn ended: the policy exited with ${child.exitCode ?? child.signalCode}`
          : `Turn ended: ${ending}`,
      );
    // Paused before the policy goes, so that its last commands do not expire into a limp robot.
    const paused =
      !mode.batch &&
      ending !== "watchdog" &&
      alive(state.pid) &&
      (await pause(ros, state.pid));
    // Done with ROS before the policy goes: once a peer has vanished, DDS waits a second for its
    // acknowledgement of each entity it deletes.
    if (paused) ros.dispose();
    await eject(child);
    if (!mode.batch)
      writeFileSync(
        join(folder, "state.json"),
        `${JSON.stringify(state, null, 2)}\n`,
      );
    if (paused) {
      console.error(
        "Paused: npx @drawcall/robotics continue runs another turn, npx @drawcall/robotics stop ends it",
      );
      console.log(folder);
      return undefined;
    }
    await quit(ros, state.pid);
    return finalize(folder, state);
  } catch (error) {
    if (child) await eject(child);
    await quit(ros, state.pid);
    throw error;
  } finally {
    ros.dispose();
  }
}

/**
 * Ends the episode as aborted and waits for its launch to exit; failing that, interrupts the
 * launch, which stops what it started, and at last kills its process group.
 */
export async function quit(
  ros: RclConnection,
  launched: number,
): Promise<void> {
  if (!alive(launched)) return;
  if (await ros.available(SET_STATE_TYPE, SET_STATE, 2000)) {
    await setState(ros, QUITTING);
    if (await exited(launched, 15_000)) return;
  }
  // The launch alone: the simulator stops its recorder itself, after the evaluation's end.
  signal(launched, "SIGINT");
  if (await exited(launched, 15_000)) return;
  signal(-launched, "SIGKILL");
  await exited(launched, 1000);
}

/**
 * Merges `state` into the result.json the simulator wrote, if the episode got that far, and
 * returns its row.
 */
export function finalize(folder: string, state: State): Row {
  const file = join(folder, "result.json");
  const { seed, wall } = state;
  if (!existsSync(file))
    return { seed, status: "missing", reason: "no result", wall };
  const result: EvaluationState = JSON.parse(readFileSync(file, "utf8"));
  // Gymnasium's terms: the task reached a terminal state, or was cut short.
  const ending =
    result.status === "success" || result.status === "failure"
      ? "terminated"
      : "truncated";
  writeFileSync(
    file,
    `${JSON.stringify({ ...result, ...state, ending }, null, 2)}\n`,
  );
  return {
    seed,
    status: result.status,
    reason: result.reason,
    elapsed: result.elapsed,
    score: result.metrics.score,
    wall,
  };
}

/** The commit the working directory is at, if it is in a git repository. */
export function git(): State["git"] {
  const commit = spawnSync("git", ["rev-parse", "HEAD"], { encoding: "utf8" });
  if (commit.status !== 0) return undefined;
  const status = spawnSync("git", ["status", "--porcelain"], {
    encoding: "utf8",
  });
  return { commit: commit.stdout.trim(), dirty: status.stdout.trim() !== "" };
}

/** Waits until the simulation can start its evaluation; it fails if the launch exits first. */
async function ready(
  ros: RclConnection,
  launched: number,
  folder: string,
): Promise<void> {
  while (!(await ros.available(START_TYPE, START, 100))) {
    if (!alive(launched))
      throw new UserError(
        `The simulation exited during startup; see ${join(folder, "launch.log")}`,
      );
    if (interrupted.signal.aborted)
      throw new UserError("Interrupted during startup");
  }
}

/**
 * Starts the policy in the scene's folder, in its own process group, below the simulator's
 * priority: the world never waits for the robot's computation.
 */
async function play(
  policy: Policy,
  state: State,
  folder: string,
  mode: Mode,
): Promise<ChildProcess> {
  const argv =
    "file" in policy
      ? ["ros2", "launch", policy.file, "use_sim_time:=true"]
      : [...policy.command, "--ros-args", "-p", "use_sim_time:=true"];
  // Episodes side by side would interleave their policies' output.
  const log = mode.batch ? openSync(join(folder, "policy.log"), "a") : 2;
  try {
    const child = await start(
      argv,
      dirname(state.scene),
      mode.batch ? ["ignore", log, log] : ["inherit", 2, 2],
      environment(state.domain, state.seed, mode.slots),
    );
    setPriority(pid(child), 10);
    return child;
  } finally {
    if (mode.batch) closeSync(log);
  }
}

/** Starts the episode's evaluation. */
async function begin(ros: RclConnection): Promise<void> {
  const response = await ros.call(START_TYPE, START, {});
  if (get(response, "success") !== true)
    throw new Error(
      `Could not start the evaluation: ${String(get(response, "message"))}`,
    );
}

/**
 * Sets a simulation_interfaces state and returns the result code: OK, ALREADY_IN_TARGET_STATE,
 * or FAILED once the episode has finished. Any other result throws.
 */
async function setState(ros: RclConnection, state: number): Promise<number> {
  const response = await ros.call(SET_STATE_TYPE, SET_STATE, {
    state: { state },
  });
  const code = field(response, "result", "result");
  if (code !== OK && code !== ALREADY_IN_TARGET_STATE && code !== FAILED)
    throw new Error(
      `Could not set the simulation state: ${String(get(response, "result", "error_message"))}`,
    );
  return code;
}

/**
 * Waits for the end of a turn: the policy exits, `timeout` simulated seconds pass, the
 * simulation launch exits, the runner is interrupted, or the watchdog fires.
 */
async function until(
  ros: RclConnection,
  launched: number,
  policy: ChildProcess,
  timeout: number,
): Promise<Ending> {
  const done = new AbortController();
  try {
    return await new Promise<Ending>((end) => {
      policy.once("exit", () => end("policy"));
      if (policy.exitCode !== null || policy.signalCode !== null) end("policy");
      const started = performance.now();
      const poll = setInterval(() => {
        if (!alive(launched)) end("simulation");
        if (interrupted.signal.aborted) end("interrupt");
        if (performance.now() - started > WATCHDOG) end("watchdog");
      }, 100);
      done.signal.addEventListener("abort", () => clearInterval(poll));
      if (timeout === Infinity) return;
      // Simulated time, which stands still while the simulation is paused.
      let start: number | undefined;
      const unsubscribe = ros.subscribe(
        "/clock",
        "rosgraph_msgs/msg/Clock",
        (message) => {
          const time =
            field(message, "clock", "sec") +
            field(message, "clock", "nanosec") / 1e9;
          start ??= time;
          if (time - start >= timeout) end("timeout");
        },
      );
      done.signal.addEventListener("abort", unsubscribe);
    });
  } finally {
    done.abort();
  }
}

/**
 * Pauses the simulation, which releases the robot's actuators and closes the recording so far.
 * False if the episode has finished instead, once its launch has exited.
 */
async function pause(ros: RclConnection, launched: number): Promise<boolean> {
  if ((await setState(ros, PAUSED)) !== FAILED) return true;
  if (!(await exited(launched, 30_000)))
    throw new Error("The simulation finished its episode but did not exit");
  return false;
}

/** Stops the policy as Ctrl-C would, then more firmly. Exit code 130 is a clean one. */
async function eject(policy: ChildProcess): Promise<void> {
  const leader = pid(policy);
  for (const [name, wait] of [
    ["SIGINT", 5000],
    ["SIGTERM", 5000],
    ["SIGKILL", 1000],
  ] as const) {
    signal(-leader, name);
    if (await exited(leader, wait)) return;
  }
}

/**
 * The environment of every process of an episode: its domain and its seed, and a share of the
 * cores, as joblib gives its workers, so that episodes side by side do not crowd each other out.
 */
function environment(
  domain: number,
  seed: number,
  slots: number,
): NodeJS.ProcessEnv {
  return {
    OMP_NUM_THREADS: String(
      Math.max(1, Math.floor(availableParallelism() / slots)),
    ),
    ...process.env,
    ROS_DOMAIN_ID: String(domain),
    SEED: String(seed),
  };
}

/** Spawns a command in a new process group and resolves once it runs. */
async function start(
  argv: string[],
  cwd: string,
  stdio: StdioOptions,
  env: NodeJS.ProcessEnv,
): Promise<ChildProcess> {
  const [command = "", ...args] = argv;
  const child = spawn(command, args, { cwd, stdio, env, detached: true });
  await once(child, "spawn");
  running.add(pid(child));
  child.once("exit", () => running.delete(pid(child)));
  return child;
}

function pid(child: ChildProcess): number {
  if (child.pid === undefined) throw new Error("Process did not start");
  return child.pid;
}

/** Signals a process, or with a negative pid a process group; it may already be gone. */
function signal(pid: number, name: NodeJS.Signals): void {
  try {
    process.kill(pid, name);
  } catch (error) {
    if (error instanceof Error && "code" in error && error.code === "ESRCH")
      return;
    throw error;
  }
}

/** Whether a process exits within `wait` milliseconds. */
async function exited(pid: number, wait: number): Promise<boolean> {
  const deadline = performance.now() + wait;
  while (alive(pid)) {
    if (performance.now() >= deadline) return false;
    await sleep(20);
  }
  return true;
}
