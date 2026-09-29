import { chmodSync, mkdtempSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { expect, it } from "vitest";
import {
  episodes,
  header,
  row,
  runEpisodes,
  total,
  type Run,
} from "../src/cli/episodes.js";
import { BEHIND } from "../src/cli/sim.js";

/** A `ros2` on PATH whose launches write a result at once; `behind` seeds fall behind on their first try. */
function fakeLaunches(behind: string): Run {
  const bin = mkdtempSync(join(tmpdir(), "ros2-"));
  writeFileSync(
    join(bin, "ros2"),
    `#!/bin/sh
case " ${behind} " in
  *" $SEED "*) [ -e "$DRAWCALL_RESULT.tried" ] || { touch "$DRAWCALL_RESULT.tried"; echo '{"status":"error","reason":"${BEHIND} 1: 2.5 s behind"}' > "$DRAWCALL_RESULT"; exit; } ;;
esac
echo '{"status":"success","reason":"done","elapsed":1}' > "$DRAWCALL_RESULT"
`,
  );
  chmodSync(join(bin, "ros2"), 0o755);
  process.env.PATH = `${bin}:${process.env.PATH}`;
  const out = mkdtempSync(join(tmpdir(), "episodes-"));
  return {
    launch: "fake.launch.xml",
    args: [],
    timeout: 30,
    out,
    slots: 3,
    threads: "1",
    running: new Set(),
  };
}

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
  expect(total(results)).toBe("1/3 succeeded, mean score 50.0");
});

it("rejects running no episodes at a time before starting any", async () => {
  const command = episodes
    .exitOverride()
    .configureOutput({ writeErr: () => {} });
  await expect(
    command.parseAsync(["launch.xml", "--parallel", "0"], { from: "user" }),
  ).rejects.toThrow(/parallel must be a whole number of at least 1/);
});

it("runs a seed that fell behind again, with one slot fewer", async () => {
  const run = fakeLaunches("2");
  const lines: string[] = [];
  const results = await runEpisodes(run, [1, 2, 3, 4], 90, (line) =>
    lines.push(line),
  );
  expect(results.map((r) => [r.seed, r.status])).toEqual([
    [1, "success"],
    [2, "success"],
    [3, "success"],
    [4, "success"],
  ]);
  expect(run.slots).toBe(2);
  expect(lines).toContain("2 fell behind; 2 at a time from now");
});

it("backs off once for episodes that fell behind in the same overload", async () => {
  const run = { ...fakeLaunches("1 2"), slots: 4 };
  const lines: string[] = [];
  const results = await runEpisodes(run, [1, 2, 3, 4, 5, 6], 90, (line) =>
    lines.push(line),
  );
  expect(results.map((r) => r.status)).toEqual(Array(6).fill("success"));
  expect(run.slots).toBe(3);
  expect(lines.filter((line) => line.includes("at a time"))).toHaveLength(1);
  expect(
    lines.filter((line) => line.includes("running it again")),
  ).toHaveLength(1);
});

it("keeps the result of an episode that falls behind on its own", async () => {
  const run = { ...fakeLaunches("5"), slots: 1 };
  const results = await runEpisodes(run, [5], 90, () => {});
  expect(results).toMatchObject([{ seed: 5, status: "error" }]);
  expect(run.slots).toBe(1);
});
