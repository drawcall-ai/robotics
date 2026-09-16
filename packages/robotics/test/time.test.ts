import { expect, it } from "vitest";
import { Schedule, stamp } from "../src/time.js";

it("normalizes nanosecond rounding across second boundaries", () => {
  expect(stamp(1.9999999998)).toEqual({ sec: 2, nanosec: 0 });
  expect(stamp(0.0125)).toEqual({ sec: 0, nanosec: 12_500_000 });
  expect(() => stamp(-1)).toThrow();
  expect(() => stamp(Infinity)).toThrow();
});

it("schedules by simulation time without a burst after skipped intervals", () => {
  const schedule = new Schedule(10);
  expect(schedule.due(0.01)).toBe(true);
  expect(schedule.due(0.09)).toBe(false);
  expect(schedule.due(0.1)).toBe(true);
  expect(schedule.due(0.45)).toBe(true);
  expect(schedule.due(0.46)).toBe(false);
  expect(schedule.due(0.5)).toBe(true);
  expect(schedule.due(0)).toBe(true); // the clock reset
  expect(schedule.due(0.05)).toBe(false);
});
