import { spawn, type ChildProcess } from "node:child_process";
import {
  closeSync,
  existsSync,
  mkdirSync,
  openSync,
  readFileSync,
  rmSync,
} from "node:fs";
import { availableParallelism } from "node:os";
import { join, resolve } from "node:path";
import { Command, InvalidArgumentError } from "commander";
import { BEHIND } from "./sim.js";

/** One episode's outcome, as the simulator wrote it, or missing if it never finished. */
export interface EpisodeResult {
  seed: number;
  status: string;
  reason: string;
  elapsed?: number;
  score?: number;
  wall: number;
}

/** What every episode of a run shares. */
export interface Run {
  launch: string;
  args: string[];
  timeout: number;
  out: string;
  /** Episodes at a time: fewer once the machine could not keep that many at pace. */
  slots: number;
  /** Inference threads per episode, fixed for the run: more would only crowd the others. */
  threads: string;
  running: Set<ChildProcess>;
}

const integer =
  (name: string, least = 1) =>
  (value: string) => {
    const parsed = Number(value);
    if (!Number.isInteger(parsed) || parsed < least)
      throw new InvalidArgumentError(
        `${name} must be a whole number of at least ${least}`,
      );
    return parsed;
  };

export const episodes = new Command("episodes")
  .description(
    "run evaluation episodes of a launch file in parallel, each in a ROS domain of its own",
  )
  .argument("<launch>", "launch file that runs a simulator with an evaluator")
  .option("--count <n>", "number of episodes", integer("count"), 1)
  .option(
    "--seed <n>",
    "seed of the first episode; the others count up",
    integer("seed", 0),
    1,
  )
  .option(
    "--parallel <n>",
    "episodes at a time",
    integer("parallel"),
    Math.max(1, Math.floor(availableParallelism() / 2)),
  )
  .option(
    "--arg <name:=value>",
    "launch argument, repeatable",
    (value: string, all: string[]) => [...all, value],
    [] as string[],
  )
  .option(
    "--timeout <seconds>",
    "wall-clock limit per episode",
    integer("timeout"),
    1800,
  )
  .option("--out <dir>", "folder for each episode's log and result", "episodes")
  .option(
    "--domain <id>",
    "ROS domain of the first slot, the others counting up: runs side by side need distinct ranges",
    integer("domain"),
    1,
  )
  .action(async function (this: Command, launch: string, options) {
    const parallel = Math.min(options.parallel, options.count);
    // Domains 0-101 are safe on every platform; 0, the default, is left to interactive runs.
    if (options.domain + parallel > 102)
      this.error("error: ROS domains must stay within 1-101");
    const seeds: number[] = Array.from(
      { length: options.count },
      (_, i) => options.seed + i,
    );
    const run: Run = {
      launch: resolve(launch),
      args: options.arg,
      timeout: options.timeout,
      out: resolve(options.out),
      slots: parallel,
      // A share of the cores each, as joblib gives its workers, so that parallel episodes'
      // inference threads do not crowd each other out while simulated time runs on.
      threads: String(
        Math.max(1, Math.floor(availableParallelism() / parallel)),
      ),
      running: new Set(),
    };
    for (const [signal, code] of [
      ["SIGINT", 130],
      ["SIGTERM", 143],
    ] as const)
      process.on(signal, () => {
        for (const child of run.running) group(child, "SIGKILL");
        process.exit(code);
      });
    console.log(header());
    const results = await runEpisodes(run, seeds, options.domain, console.log);
    console.log(total(results));
    process.exit(results.every((r) => r.status === "success") ? 0 : 1);
  });

/**
 * Runs the seeds `run.slots` at a time, each slot in a ROS domain of its own from `domain` on,
 * and returns their results by seed. An episode that could not keep its pace runs again, and
 * the first one of an overload retires a slot; alone, its result stands.
 */
export async function runEpisodes(
  run: Run,
  seeds: number[],
  domain: number,
  report: (line: string) => void,
): Promise<EpisodeResult[]> {
  const results: EpisodeResult[] = [];
  let alive = run.slots;
  const slot = async (domain: number) => {
    // A slot stops taking seeds while more run than the machine keeps at pace.
    while (alive <= run.slots) {
      const seed = seeds.shift();
      if (seed === undefined) break;
      const slots = run.slots;
      const result = await episode(run, seed, domain);
      if (result.reason.startsWith(BEHIND) && slots > 1) {
        seeds.unshift(seed);
        // Episodes started before the back-off fall behind from the same overload.
        if (run.slots === slots) {
          run.slots = slots - 1;
          report(`${seed} fell behind; ${run.slots} at a time from now`);
        } else report(`${seed} fell behind; running it again`);
        continue;
      }
      results.push(result);
      report(row(result));
    }
    alive--;
  };
  await Promise.all(
    Array.from({ length: run.slots }, (_, i) => slot(domain + i)),
  );
  return results.sort((a, b) => a.seed - b.seed);
}

async function episode(
  run: Run,
  seed: number,
  domain: number,
): Promise<EpisodeResult> {
  const folder = join(run.out, String(seed));
  mkdirSync(folder, { recursive: true });
  const file = join(folder, "result.json");
  rmSync(file, { force: true });
  const start = performance.now();
  const log = openSync(join(folder, "launch.log"), "w");
  // Its own process group, so the whole launch can be stopped at once; discovery kept to
  // this machine.
  const child = spawn("ros2", ["launch", run.launch, ...run.args], {
    detached: true,
    stdio: ["ignore", log, log],
    env: {
      OMP_NUM_THREADS: run.threads,
      ...process.env,
      ROS_DOMAIN_ID: String(domain),
      ROS_AUTOMATIC_DISCOVERY_RANGE: "LOCALHOST",
      SEED: String(seed),
      DRAWCALL_RESULT: file,
    },
  });
  closeSync(log);
  run.running.add(child);
  let timedOut = false;
  const limit = setTimeout(() => {
    timedOut = true;
    stop(child);
  }, run.timeout * 1000);
  await new Promise((done) => child.on("exit", done));
  clearTimeout(limit);
  run.running.delete(child);
  const wall = (performance.now() - start) / 1000;
  if (timedOut)
    return { seed, status: "timeout", reason: "wall-clock limit", wall };
  if (!existsSync(file))
    return { seed, status: "missing", reason: "no result", wall };
  const state = JSON.parse(readFileSync(file, "utf8"));
  return {
    seed,
    status: state.status,
    reason: state.reason,
    elapsed: state.elapsed,
    score: state.metrics?.score,
    wall,
  };
}

/** Signal a launch's whole process group; it may already be gone. */
function group(child: ChildProcess, signal: NodeJS.Signals): void {
  try {
    process.kill(-child.pid!, signal);
  } catch {
    // Already gone.
  }
}

/** Interrupt a launch and everything it started, as Ctrl-C would, then kill what is left. */
function stop(child: ChildProcess): void {
  group(child, "SIGINT");
  setTimeout(() => group(child, "SIGKILL"), 15_000).unref();
}

export function header(): string {
  return `${"seed".padEnd(8)}${"status".padEnd(10)}${"reason".padEnd(22)}${"score".padStart(6)}${"sim s".padStart(8)}${"wall s".padStart(8)}`;
}

export function row(r: EpisodeResult): string {
  return (
    `${String(r.seed).padEnd(8)}${r.status.padEnd(10)}${r.reason.padEnd(22)}` +
    `${(r.score === undefined ? "" : r.score.toFixed(0)).padStart(6)}` +
    `${(r.elapsed === undefined ? "" : r.elapsed.toFixed(0)).padStart(8)}` +
    `${r.wall.toFixed(0).padStart(8)}`
  );
}

/** The share of episodes that succeeded, and their mean score. */
export function total(results: EpisodeResult[]): string {
  const succeeded = results.filter((r) => r.status === "success").length;
  const scores = results.flatMap((r) =>
    r.score === undefined ? [] : [r.score],
  );
  const mean = scores.length
    ? scores.reduce((a, b) => a + b, 0) / scores.length
    : 0;
  return `${succeeded}/${results.length} succeeded, mean score ${mean.toFixed(1)}`;
}
