import { expect, it } from "vitest";
import { episodes, header, row, total } from "../src/cli/episodes.js";

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
