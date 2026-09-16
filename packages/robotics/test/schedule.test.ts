import { expect, it } from "vitest";
import { Schedule } from "../src/devices/schedule.js";

it("fires once per period of simulation time without a burst after a gap", () => {
  const schedule = new Schedule(10);
  expect(schedule.period).toBe(0.1);
  expect(schedule.due(0.01)).toBe(true);
  expect(schedule.due(0.09)).toBe(false);
  expect(schedule.due(0.1)).toBe(true);
  expect(schedule.due(0.45)).toBe(true);
  expect(schedule.due(0.46)).toBe(false);
  expect(schedule.due(0.5)).toBe(true);
  expect(schedule.due(0)).toBe(true); // the clock reset
  expect(schedule.due(0.05)).toBe(false);
});

it("needs a positive finite rate", () => {
  for (const rate of [0, -5, Infinity, NaN])
    expect(() => new Schedule(rate)).toThrow("Rate must be positive");
});
