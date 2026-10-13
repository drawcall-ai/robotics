import { randomInt } from "node:crypto";
import {
  existsSync,
  mkdirSync,
  readFileSync,
  realpathSync,
  renameSync,
  rmSync,
  symlinkSync,
} from "node:fs";
import { once } from "node:events";
import { createServer } from "node:net";
import { availableParallelism } from "node:os";
import { dirname, join, relative, resolve } from "node:path";
import { Command, InvalidArgumentError } from "commander";
import { RclConnection } from "../connections/rcl.js";
import { alive, claim, hold } from "./domain.js";
import {
  finalize,
  git,
  interrupted,
  launch,
  listen,
  quit,
  turn,
  type Mode,
  type Policy,
  type Row,
  type Setup,
  type State,
} from "./episode.js";
import { lichtblickUrl } from "./lichtblick.js";
import { BEHIND } from "./protocol.js";

interface RunOptions {
  bringup?: string;
  policy?: string;
  count?: number;
  seed?: number[];
  timeout?: number;
  realTimeFactor: number;
  viewer?: boolean;
  recordAll?: boolean;
  minScore?: number;
}

/** A parser of numbers of at least `min`, whole ones if `integer`. */
export function number(min: number, integer = false) {
  return (value: string): number => {
    const parsed = Number(value);
    if (value.trim() === "" || Number.isNaN(parsed))
      throw new InvalidArgumentError("must be a number");
    if (integer && !Number.isInteger(parsed))
      throw new InvalidArgumentError("must be a whole number");
    if (parsed < min) throw new InvalidArgumentError(`must be at least ${min}`);
    return parsed;
  };
}

/** `run`; `command` is the policy command after `--`, which replaces the policy file. */
export function run(command: string[] | undefined): Command {
  return new Command("run")
    .description(
      "run evaluation episodes of a scene; a single episode pauses when its turn ends",
    )
    .argument("[scene]", "the .glts scene", "scene.glts")
    .option(
      "--bringup <file>",
      "the robot's persistent stack (default: bringup.launch.xml beside the scene)",
    )
    .option(
      "--policy <file>",
      "the robot's decision-making (default: policy.launch.xml beside the scene)",
    )
    .option("--count <n>", "number of episodes (default: 1)", number(1, true))
    .option(
      "--seed <n|a,b,c>",
      "seed of the first episode, the others counting up, or the list of seeds (default: random)",
      (value) => value.split(",").map(number(0, true)),
    )
    .option(
      "--timeout <s>",
      "simulated seconds a single episode's turn may take",
      number(0),
    )
    .option(
      "--real-time-factor <k>",
      "simulated seconds per wall-clock second; 0 runs as fast as the host allows",
      number(0),
      1,
    )
    .option("--viewer", "serve Lichtblick and open it (single episodes only)")
    .option("--record-all", "also record raw images")
    .option(
      "--min-score <x>",
      "exit 1 when the mean score is below this",
      number(-Infinity),
    )
    .action(async function (
      this: Command,
      sceneFile: string,
      options: RunOptions,
    ) {
      if (options.seed && options.seed.length > 1 && options.count)
        this.error("error: --count does not combine with a list of seeds");
      const list = seeds(options.seed, options.count);
      const batch = list.length > 1;
      if (batch && options.viewer)
        this.error("error: --viewer needs a single episode");
      if (batch && options.timeout !== undefined)
        this.error("error: --timeout needs a single episode");
      const scene = existing(this, resolve(sceneFile));
      const setup: Setup = {
        scene,
        bringup: existing(
          this,
          resolve(
            options.bringup ?? join(dirname(scene), "bringup.launch.xml"),
          ),
        ),
        policy: policy(this, scene, options.policy, command),
        realTimeFactor: options.realTimeFactor,
        viewer: options.viewer ? await viewerPorts() : undefined,
        recordAll: options.recordAll ?? false,
        git: git(),
      };
      const mode: Mode = {
        batch,
        timeout: options.timeout ?? Infinity,
        slots: Math.min(
          list.length,
          Math.max(1, Math.floor(availableParallelism() / 2)),
        ),
      };
      const folder = create();
      const place = (seed: number) =>
        batch ? join(folder, String(seed)) : folder;
      listen();
      if (batch) console.error(header());
      const rows = await schedule(
        list,
        mode.slots,
        (seed) => episode(setup, place(seed), seed, mode),
        (line) => console.error(line),
        (r) => print(r, place(r.seed), !batch),
      );
      if (batch) console.error(total(rows));
      const below =
        options.minScore !== undefined &&
        rows.length > 0 &&
        mean(rows) < options.minScore;
      process.exit(below ? 1 : 0);
    });
}

/** `continue`; `command` is the policy command after `--`, which replaces the policy file. */
export function resume(command: string[] | undefined): Command {
  return new Command("continue")
    .description("run another turn of a paused single run")
    .option("--run <folder>", "the paused run", "runs/latest")
    .option(
      "--policy <file>",
      "the robot's decision-making (default: policy.launch.xml beside the scene)",
    )
    .option("--timeout <s>", "simulated seconds the turn may take", number(0))
    .action(async function (
      this: Command,
      options: { run: string; policy?: string; timeout?: number },
    ) {
      const folder = paused(this, options.run);
      const state = read(folder);
      if (!alive(state.pid)) this.error(`error: ${folder} has ended`);
      const chosen = policy(this, state.scene, options.policy, command);
      state.continued = true;
      listen();
      const r = await turn(state, folder, chosen, {
        batch: false,
        timeout: options.timeout ?? Infinity,
        slots: 1,
      });
      if (r) print(r, folder, true);
      process.exit(0);
    });
}

export const stop = new Command("stop")
  .description("end a paused single run: its episode ends as aborted")
  .option("--run <folder>", "the paused run", "runs/latest")
  .action(async function (this: Command, options: { run: string }) {
    const folder = paused(this, options.run);
    const state = read(folder);
    const ros = await RclConnection.create(
      `drawcall_robotics_${state.domain}`,
      state.domain,
    );
    try {
      await quit(ros, state.pid);
    } finally {
      ros.dispose();
    }
    print(finalize(folder, state), folder, true);
    process.exit(0);
  });

/** The episodes' seeds: the listed ones, or `count` from a base seed, random by default. */
export function seeds(listed: number[] | undefined, count = 1): number[] {
  if (listed && listed.length > 1) return listed;
  const base = listed?.[0] ?? randomInt(1_000_000);
  return Array.from({ length: count }, (_, i) => base + i);
}

/**
 * Runs `episode` for each seed, `slots` at a time, until interrupted. An episode that could not
 * keep its pace runs again, and the first one of an overload retires a slot; alone, its row
 * stands. A paused episode has no row yet.
 */
export async function schedule(
  seeds: number[],
  slots: number,
  episode: (seed: number) => Promise<Row | undefined>,
  report: (line: string) => void,
  done: (row: Row) => void,
): Promise<Row[]> {
  const queue = [...seeds];
  const rows: Row[] = [];
  let active = slots;
  const slot = async () => {
    // A slot stops taking seeds while more run than the machine keeps at pace.
    while (active <= slots && !interrupted.signal.aborted) {
      const seed = queue.shift();
      if (seed === undefined) break;
      const before = slots;
      const row = await episode(seed);
      if (!row) continue;
      if (row.reason.startsWith(BEHIND) && before > 1) {
        queue.unshift(seed);
        // Episodes started before the back-off fall behind from the same overload.
        if (slots === before) {
          slots = before - 1;
          report(`${seed} fell behind; ${slots} at a time from now`);
        } else report(`${seed} fell behind; running it again`);
        continue;
      }
      rows.push(row);
      done(row);
    }
    active--;
  };
  await Promise.all(Array.from({ length: slots }, slot));
  return rows.sort((a, b) => a.seed - b.seed);
}

export function header(): string {
  return `${"seed".padEnd(8)}${"status".padEnd(10)}${"reason".padEnd(22)}${"score".padStart(6)}${"sim s".padStart(8)}${"wall s".padStart(8)}`;
}

export function row(r: Row): string {
  return (
    `${String(r.seed).padEnd(8)}${r.status.padEnd(10)}${r.reason.padEnd(22)}` +
    `${(r.score === undefined ? "" : r.score.toFixed(0)).padStart(6)}` +
    `${(r.elapsed === undefined ? "" : r.elapsed.toFixed(0)).padStart(8)}` +
    `${r.wall.toFixed(0).padStart(8)}`
  );
}

/** The mean score; an episode without one, such as a missing one, counts as 0. */
export function mean(rows: Row[]): number {
  return rows.reduce((sum, r) => sum + (r.score ?? 0), 0) / rows.length;
}

/** The share of episodes that succeeded, and their mean score. */
export function total(rows: Row[]): string {
  const succeeded = rows.filter((r) => r.status === "success").length;
  return `${succeeded}/${rows.length} succeeded, mean score ${(rows.length ? mean(rows) : 0).toFixed(1)}`;
}

/**
 * One episode, from its launch to its first turn's end. In a batch, a failure becomes its
 * error row.
 */
async function episode(
  setup: Setup,
  folder: string,
  seed: number,
  mode: Mode,
): Promise<Row | undefined> {
  if (mode.batch) {
    rmSync(folder, { recursive: true, force: true });
    mkdirSync(folder);
  }
  try {
    const domain = claim();
    if (!mode.batch)
      console.error(
        `Simulating ${setup.scene} with seed ${seed} in ROS domain ${domain}`,
      );
    const pid = await launch(
      setup,
      folder,
      seed,
      domain,
      mode.slots,
      mode.batch,
    );
    hold(domain, pid);
    // The simulator serves Lichtblick on this port once it is up, and opens it.
    if (setup.viewer)
      console.error(
        `Lichtblick: ${lichtblickUrl(setup.viewer.port, { bridge: setup.viewer.bridge })}`,
      );
    const state: State = {
      domain,
      pid,
      scene: setup.scene,
      seed,
      continued: false,
      wall: 0,
      argv: process.argv,
      git: setup.git,
    };
    return await turn(state, folder, setup.policy, mode);
  } catch (error) {
    if (!mode.batch) throw error;
    const message = error instanceof Error ? error.message : String(error);
    console.error(`${seed}: ${message}`);
    return { seed, status: "error", reason: message, wall: 0 };
  }
}

/** Prints an episode's row on stderr, and its folder, now final, on stdout. */
function print(r: Row, folder: string, titled: boolean): void {
  if (titled) console.error(header());
  console.error(row(r));
  console.log(folder);
}

/** Two free ports, for Lichtblick and the foxglove_bridge, so that runs side by side keep apart. */
async function viewerPorts(): Promise<{ port: number; bridge: number }> {
  // Both stay bound until both are known, so that they differ.
  await using servers = new AsyncDisposableStack();
  return { port: await freePort(servers), bridge: await freePort(servers) };
}

async function freePort(servers: AsyncDisposableStack): Promise<number> {
  const server = servers.use(createServer());
  server.listen(0, "127.0.0.1");
  await once(server, "listening");
  const address = server.address();
  if (!address || typeof address === "string")
    throw new Error("Port probe has no TCP address");
  return address.port;
}

/** The policy to run: a command after `--`, or a launch file. */
function policy(
  command: Command,
  scene: string,
  file: string | undefined,
  argv: string[] | undefined,
): Policy {
  if (argv && file)
    command.error("error: --policy does not combine with a command after --");
  if (argv && !argv.length) command.error("error: no command after --");
  if (argv) return { command: argv };
  return {
    file: existing(
      command,
      resolve(file ?? join(dirname(scene), "policy.launch.xml")),
    ),
  };
}

function existing(command: Command, path: string): string {
  if (!existsSync(path)) command.error(`error: ${path} does not exist`);
  return path;
}

/**
 * A new run folder, runs/<local date and time>, with a suffix if another run started in the
 * same second, which runs/latest then points to.
 */
function create(): string {
  // Sweden writes dates as ISO 8601 does: 2026-10-02 03:35:31.
  const stamp = new Date()
    .toLocaleString("sv")
    .replace(" ", "-")
    .replaceAll(":", "");
  let name = stamp;
  // Only the call that creates the folder gets its path back.
  for (let n = 2; !mkdirSync(join("runs", name), { recursive: true }); n++)
    name = `${stamp}-${n}`;
  const latest = join("runs", `.latest-${process.pid}`);
  symlinkSync(name, latest);
  renameSync(latest, join("runs", "latest"));
  return join("runs", name);
}

/** The folder of a paused single run, relative to the working directory. */
function paused(command: Command, run: string): string {
  const folder = relative(".", realpathSync(existing(command, run)));
  if (!existsSync(join(folder, "state.json")))
    command.error(`error: ${folder} is not a single run: it has no state.json`);
  return folder;
}

function read(folder: string): State {
  return JSON.parse(readFileSync(join(folder, "state.json"), "utf8"));
}
