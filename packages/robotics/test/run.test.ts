import { spawn } from "node:child_process";
import { mkdtempSync, readFileSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { afterEach, beforeEach, expect, it } from "vitest";
import { alive, claim, hold } from "../src/cli/domain.js";
import { finalize, type Row } from "../src/cli/episode.js";
import { BEHIND } from "../src/cli/protocol.js";
import { header, number, row, schedule, seeds, total } from "../src/cli/run.js";

const temporary = process.env.TMPDIR;
beforeEach(() => {
  process.env.TMPDIR = mkdtempSync(join(tmpdir(), "domains-"));
});
afterEach(() => {
  process.env.TMPDIR = temporary;
});

it("tabulates episodes and counts the ones that succeeded", () => {
  const results = [
    {
      seed: 1,
      status: "success",
      reason: "piecePlaced",
      elapsed: 231.4,
      score: 100,
      wall: 118,
    },
    {
      seed: 2,
      status: "timeout",
      reason: "timeLimit",
      elapsed: 600,
      score: 0,
      wall: 297,
    },
    { seed: 3, status: "missing", reason: "no result", wall: 3 },
  ];
  expect(header()).toMatch(/^seed\s+status\s+reason\s+score\s+sim s\s+wall s$/);
  expect(row(results[0]!)).toMatch(
    /^1\s+success\s+piecePlaced\s+100\s+231\s+118$/,
  );
  expect(row(results[2]!)).toMatch(/^3\s+missing\s+no result\s+3$/);
  // A missing episode scores 0.
  expect(total(results)).toBe("1/3 succeeded, mean score 33.3");
});

it("parses numbers within bounds", () => {
  expect(number(0)("2.5")).toBe(2.5);
  expect(number(-Infinity)("-3")).toBe(-3);
  expect(() => number(-Infinity)("abc")).toThrow("must be a number");
  expect(() => number(0)("")).toThrow("must be a number");
  expect(() => number(1, true)("1.5")).toThrow("must be a whole number");
  expect(() => number(1, true)("0")).toThrow("must be at least 1");
});

it("counts seeds up from a base, keeps a list as given, and picks a random base", () => {
  expect(seeds([100], 3)).toEqual([100, 101, 102]);
  expect(seeds([3, 17, 42])).toEqual([3, 17, 42]);
  const [base, next] = seeds(undefined, 2);
  expect(next).toBe(base! + 1);
  expect(seeds(undefined)).toHaveLength(1);
});

it("claims free domains and reclaims the ones whose process is gone", async () => {
  expect(claim()).toBe(1);
  expect(claim()).toBe(2);
  const child = spawn("true");
  await new Promise((done) => child.on("exit", done));
  hold(1, child.pid!);
  expect(alive(child.pid!)).toBe(false);
  expect(claim()).toBe(1);
  expect(
    readFileSync(
      join(process.env.TMPDIR!, "drawcall-robotics", "domain-1"),
      "utf8",
    ),
  ).toBe(String(process.pid));
  expect(claim()).toBe(3);
});

it("leaves a lock alone while its creator is still writing it", () => {
  expect(claim()).toBe(1);
  writeFileSync(join(process.env.TMPDIR!, "drawcall-robotics", "domain-1"), "");
  expect(claim()).toBe(2);
});

/** An episode that falls behind on its first try for the seeds in `behind`. */
function episodes(behind: number[]) {
  const tried = new Set<number>();
  return async (seed: number): Promise<Row> => {
    await new Promise((done) => setTimeout(done, 5));
    if (behind.includes(seed) && !tried.has(seed)) {
      tried.add(seed);
      return { seed, status: "error", reason: `${BEHIND} 1: 2.5 s`, wall: 1 };
    }
    return { seed, status: "success", reason: "done", wall: 1 };
  };
}

it("runs a seed that fell behind again, with one slot fewer", async () => {
  const lines: string[] = [];
  const done: number[] = [];
  const results = await schedule(
    [1, 2, 3, 4],
    3,
    episodes([2]),
    (line) => lines.push(line),
    (result) => done.push(result.seed),
  );
  expect(results.map((r) => [r.seed, r.status])).toEqual([
    [1, "success"],
    [2, "success"],
    [3, "success"],
    [4, "success"],
  ]);
  expect(done.sort()).toEqual([1, 2, 3, 4]);
  expect(lines).toContain("2 fell behind; 2 at a time from now");
});

it("backs off once for episodes that fell behind in the same overload", async () => {
  const lines: string[] = [];
  const results = await schedule(
    [1, 2, 3, 4, 5, 6],
    4,
    episodes([1, 2]),
    (line) => lines.push(line),
    () => {},
  );
  expect(results.map((r) => r.status)).toEqual(Array(6).fill("success"));
  expect(lines.filter((line) => line.includes("at a time"))).toHaveLength(1);
  expect(
    lines.filter((line) => line.includes("running it again")),
  ).toHaveLength(1);
});

it("keeps the result of an episode that falls behind on its own", async () => {
  const results = await schedule(
    [5],
    1,
    episodes([5]),
    () => {},
    () => {},
  );
  expect(results).toMatchObject([{ seed: 5, status: "error" }]);
});

it("merges the episode's state into the simulator's result", () => {
  const folder = mkdtempSync(join(tmpdir(), "episode-"));
  const state = {
    domain: 3,
    pid: 42,
    scene: "scene.ts",
    seed: 7,
    continued: true,
    wall: 12,
    argv: ["run"],
  };
  expect(finalize(folder, state)).toEqual({
    seed: 7,
    status: "missing",
    reason: "no result",
    wall: 12,
  });
  writeFileSync(
    join(folder, "result.json"),
    JSON.stringify({
      stamp: { sec: 3, nanosec: 0 },
      elapsed: 3,
      status: "aborted",
      reason: "quit",
      metrics: { score: 40 },
    }),
  );
  expect(finalize(folder, state)).toEqual({
    seed: 7,
    status: "aborted",
    reason: "quit",
    elapsed: 3,
    score: 40,
    wall: 12,
  });
  expect(
    JSON.parse(readFileSync(join(folder, "result.json"), "utf8")),
  ).toMatchObject({ ...state, status: "aborted", ending: "truncated" });
});
