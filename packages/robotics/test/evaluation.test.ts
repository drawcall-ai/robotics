import { expect, it } from "vitest";
import { BoxCollider, RigidBody, Trigger } from "@drawcall/physics";
import { Evaluator, EVALUATION_TOPIC } from "../src/evaluation.js";
import { Stepping } from "../src/cli/sim.js";
import { simulation } from "./fixture.js";

it("publishes initial, coalesced absolute metrics, and an immediate frozen terminal snapshot", async () => {
  const { world, connection } = await simulation();
  let score = 0;
  const evaluation = new Evaluator({
    timeout: 10,
    success: { complete: () => score === 3 },
    score: () => score,
  });
  evaluation.start();
  const messages = () =>
    connection.messages.filter(({ name }) => name === EVALUATION_TOPIC);
  expect(messages()).toHaveLength(1);
  score = 1;
  for (let i = 0; i < 9; i++) world.update(0.01);
  expect(messages()).toHaveLength(1);
  world.update(0.01);
  expect(messages()).toHaveLength(2);
  score = 3;
  world.update(0.01);
  expect(messages()).toHaveLength(3);
  expect(evaluation.result).toMatchObject({
    status: "success",
    reason: "complete",
    metrics: { score: 3 },
  });
  expect(messages().at(-1)).toMatchObject({
    type: "std_msgs/msg/String",
    message: { data: JSON.stringify(evaluation.result) },
  });
  expect(
    connection.messages
      .filter(({ name }) => name === "/sim/score")
      .map(({ message }) => message),
  ).toEqual([{ data: 0 }, { data: 1 }, { data: 3 }]);
  const result = evaluation.result;
  score = 4;
  world.update(0.01);
  expect(evaluation.result).toBe(result);
  expect(messages()).toHaveLength(3);
  expect(() => evaluation.start()).toThrow("only start once");
});

it("checks failure before success and records timeout even when no metric changes", async () => {
  const { world } = await simulation();
  const evaluation = new Evaluator({
    timeout: 0.01,
    success: { done: () => true },
    failure: { unsafe: () => true },
    score: ({ status }) => (status === "success" ? 100 : 0),
  });
  evaluation.start();
  world.update(0.01);
  expect(evaluation.result).toMatchObject({
    status: "failure",
    reason: "unsafe",
  });
  evaluation.dispose();
  const timeout = new Evaluator({
    timeout: 0.01,
    success: {},
    score: () => 0,
  });
  timeout.start();
  world.update(0.01);
  expect(timeout.result).toMatchObject({
    status: "timeout",
    reason: "timeLimit",
  });
});

it("samples after synchronous devices regardless of registration order", async () => {
  const { world, ros } = await simulation();
  let value = 0;
  const evaluation = new Evaluator({
    timeout: 1,
    success: { updated: () => value === 1 },
    score: () => value,
  });
  ros.register({
    afterStep: () => {
      value = 1;
    },
    dispose() {},
  });
  evaluation.start();
  world.update(0.01);
  expect(evaluation.result?.status).toBe("success");
});

it("uses real trigger samples and stops catch-up immediately at a terminal step", async () => {
  const { world } = await simulation();
  const body = new RigidBody();
  body.add(new BoxCollider({ size: [1, 1, 1] }));
  const goal = new Trigger();
  goal.add(new BoxCollider({ size: [2, 2, 2] }));
  const evaluation = new Evaluator({
    timeout: 1,
    success: { reached: () => goal.overlaps(body) },
    score: ({ status }) => (status === "success" ? 100 : 0),
  });
  evaluation.start();
  new Stepping(world, () => evaluation.finished).update(0.1);
  expect(world.time).toBeCloseTo(0.01);
  expect(evaluation.result?.status).toBe("success");
});

it("does not evaluate before start or publish unchanged values", async () => {
  const { world, connection } = await simulation();
  const evaluation = new Evaluator({
    timeout: 1,
    success: {},
    score: () => 0,
  });
  world.update(0.01);
  expect(evaluation.result).toBeUndefined();
  evaluation.start();
  for (let i = 0; i < 30; i++) world.update(0.01);
  expect(
    connection.messages.filter(({ name }) => name === EVALUATION_TOPIC),
  ).toHaveLength(1);
  evaluation.end("aborted", "interrupted");
  expect(evaluation.result).toMatchObject({
    status: "aborted",
    reason: "interrupted",
    elapsed: expect.closeTo(0.3),
  });
});

it("rejects invalid values and multiple evaluators", async () => {
  await simulation();
  expect(
    () => new Evaluator({ timeout: -1, success: {}, score: () => 0 }),
  ).toThrow("timeout");
  const evaluation = new Evaluator({
    timeout: 1,
    success: {},
    score: () => NaN,
  });
  expect(() => evaluation.start()).toThrow("finite");
  expect(evaluation.active).toBe(false);
  expect(evaluation.result).toBeUndefined();
  expect(() => evaluation.end("error", "initial sample failed")).not.toThrow();
  expect(
    () => new Evaluator({ timeout: 1, success: {}, score: () => 0 }),
  ).toThrow("one evaluator");
});

it("preserves a final diagnostic snapshot after a runtime fault", async () => {
  const { ros, connection } = await simulation();
  const evaluation = new Evaluator({
    timeout: 1,
    success: {},
    score: () => 25,
  });
  evaluation.start();
  ros.fail(new Error("device failed"));
  evaluation.end("error", "device failed");
  expect(evaluation.result).toMatchObject({
    status: "error",
    reason: "device failed",
  });
  expect(
    connection.messages.filter(({ name }) => name === EVALUATION_TOPIC),
  ).toHaveLength(2);
});

it("preserves the last valid metrics when a condition throws", async () => {
  const { world, ros } = await simulation();
  const error = new Error("condition failed");
  const evaluation = new Evaluator({
    timeout: 1,
    success: {
      complete: () => {
        throw error;
      },
    },
    score: () => 25,
  });
  evaluation.start();
  expect(() => world.update(0.01)).toThrow(error);
  expect(ros.error).toBe(error);
  evaluation.end("error", error.message);
  expect(evaluation.result).toMatchObject({
    status: "error",
    metrics: { score: 25 },
  });
});

it("steps until a deadline, or until halted", async () => {
  const { world } = await simulation();
  new Stepping(world, () => false).until(performance.now() + 20);
  expect(world.time).toBeGreaterThan(0);
  const before = world.time;
  new Stepping(world, () => true).until(performance.now() + 20);
  expect(world.time).toBe(before);
});

it("counts the time it could not step as behind, and none while it keeps up", async () => {
  const { world } = await simulation();
  const keeping = new Stepping(world, () => false);
  for (let i = 0; i < 10; i++) keeping.update(0.1);
  expect(keeping.behind).toBe(0);
  // Held, it carries at most 32 steps of debt and drops the rest.
  const held = new Stepping(world, () => true);
  for (let i = 0; i < 10; i++) held.update(0.1);
  expect(held.behind).toBeCloseTo(1 - 32 * world.fixedDelta);
});

it("slows down when it falls behind, unless it must keep pace", async () => {
  const { world } = await simulation();
  const held = new Stepping(world, () => true);
  for (let i = 0; i < 30; i++) held.update(0.1);
  expect(() => held.pace(true, 1)).toThrow(
    "could not keep the real-time factor 1",
  );
  held.pace(false, 1);
  expect(held.behind).toBe(0);
  for (let i = 0; i < 30; i++) held.update(0.1);
  expect(() => held.pace(false, 1)).not.toThrow();
});
