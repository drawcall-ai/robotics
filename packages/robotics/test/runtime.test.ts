import { expect, it, vi } from "vitest";
import { LaserSensor } from "../src/devices/laser.js";
import { cart, jointCommand, offline, simulation } from "./fixture.js";

/** Every message inside an error, however it is nested. */
function messages(error: unknown): string[] {
  if (!(error instanceof Error)) return [String(error)];
  const nested = error instanceof AggregateError ? error.errors : [];
  return [error.message, ...[...nested, error.cause].flatMap(messages)];
}

it("steps a plain device until disposed", async () => {
  const { ros, world } = await simulation();
  const afterStep = vi.fn();
  const dispose = vi.fn();
  ros.register({ afterStep, dispose });
  world.update(0.01);
  expect(afterStep).toHaveBeenCalledWith(0.01, 0.01);
  ros.dispose();
  expect(dispose).toHaveBeenCalledOnce();
  world.update(0.01);
  expect(afterStep).toHaveBeenCalledOnce();
});

it("stops every device and stays faulted after a device throws in a step", async () => {
  const { ros, world } = await simulation();
  ros.register({
    afterStep() {
      throw new Error("Sensor failed");
    },
    stop() {
      throw new Error("Extension stop failed");
    },
    dispose() {},
  });
  const stop = vi.fn();
  ros.register({ stop, dispose() {} });
  expect(() => world.update(0.01)).toThrow("Sensor failed");
  expect(stop).toHaveBeenCalledOnce();
  expect(ros.connected).toBe(false);
  expect(ros.error?.message).toContain("Sensor failed");
  expect(messages(ros.error)).toContain("Extension stop failed");
});

it("stops every device once after the connection is lost", async () => {
  const { ros, world, connection } = await simulation();
  ros.register({
    stop() {
      throw new Error("Stop failed");
    },
    dispose() {},
  });
  const stop = vi.fn();
  ros.register({ stop, dispose() {} });
  connection.connected = false;
  expect(() => world.update(0.01)).not.toThrow();
  expect(ros.connected).toBe(false);
  expect(ros.error?.message).toContain("ROS connection lost");
  expect(messages(ros.error)).toContain("Stop failed");
  world.update(0.01);
  expect(stop).toHaveBeenCalledOnce();
});

it("releases everything else when a device fails to dispose", async () => {
  const { ros, world, connection } = await simulation();
  ros.register({
    dispose() {
      throw new Error("Extension disposal failed");
    },
  });
  const dispose = vi.fn();
  ros.register({ dispose });
  const release = ros.subscribe("status", "std_msgs/msg/String", () => {});
  expect(() => ros.dispose()).toThrow();
  expect(dispose).toHaveBeenCalledOnce();
  expect(connection.disposed).toBe(true);
  expect(ros.connected).toBe(false);
  expect(() => ros.dispose()).not.toThrow();
  expect(() => release()).not.toThrow();
  world.update(0.01);
  expect(connection.messages).toHaveLength(0);
});

it("does not register a sensor with invalid options", async () => {
  const { ros, world } = await simulation();
  expect(() => new LaserSensor({ ros, samples: 1 })).toThrow("at least two");
  expect(ros.find(LaserSensor)).toEqual([]);
  expect(() => world.update(0.01)).not.toThrow();
});

it("puts topics under its namespace and passes absolute topics through", async () => {
  const { world, ros, connection } = await simulation({ namespace: "/bot" });
  const { robot, actuator } = cart(ros);
  world.scene.add(robot);
  connection.receive("/bot/joint_commands", jointCommand("velocity", 1));
  world.update(0.01);
  expect(actuator.target?.velocity).toBe(1);
  expect(connection.messages.map(({ name }) => name)).toContain(
    "/bot/joint_states",
  );
  expect(ros.topic("/abs")).toBe("/abs");
  expect(() => ros.topic("bad name")).toThrow("Invalid ROS topic");
});

it("publishes at the root with the root namespace", async () => {
  const { world, ros, connection } = await simulation({ namespace: "/" });
  world.scene.add(cart(ros).robot);
  world.update(0.01);
  expect(connection.messages.map(({ name }) => name)).toContain(
    "/joint_states",
  );
});

it("rejects an invalid namespace", () => {
  expect(() => offline("bot")).toThrow("Invalid ROS namespace");
});

function deferred() {
  let resolve: (() => void) | undefined;
  let reject: ((error: unknown) => void) | undefined;
  const promise = new Promise<void>((yes, no) => {
    resolve = yes;
    reject = no;
  });
  if (!resolve || !reject) throw new Error("Promise executor did not run");
  return { promise, resolve, reject };
}

it("disconnects immediately and joins all device work after synchronous disposal", async () => {
  const { ros, world, connection } = await simulation();
  const first = deferred();
  const second = deferred();
  const afterStep = vi.fn();
  const dispose = vi.fn(() => first.promise);
  ros.register({ afterStep, dispose });
  ros.register({ dispose: () => second.promise });
  ros.dispose();
  expect(ros.connected).toBe(false);
  expect(connection.disposed).toBe(true);
  world.update(0.01);
  expect(afterStep).not.toHaveBeenCalled();
  const finished = vi.fn();
  const completion = ros.disposeAsync().then(finished);
  first.resolve();
  await Promise.resolve();
  expect(finished).not.toHaveBeenCalled();
  second.resolve();
  await completion;
  await ros.disposeAsync();
  expect(finished).toHaveBeenCalledOnce();
  expect(dispose).toHaveBeenCalledOnce();
});

it("retains synchronous and asynchronous release failures for every async join", async () => {
  const { ros, connection } = await simulation();
  const readback = deferred();
  const syncFailure = new Error("Device release failed");
  const asyncFailure = new Error("Readback failed");
  ros.register({
    dispose() {
      throw syncFailure;
    },
  });
  ros.register({ dispose: () => readback.promise });
  expect(() => ros.dispose()).toThrow("Failed to dispose ROS runtime");
  expect(connection.disposed).toBe(true);
  readback.reject(asyncFailure);
  // A later owner can join a synchronous shutdown without an unhandled rejection in between.
  await new Promise((resolve) => setTimeout(resolve, 0));
  for (let attempt = 0; attempt < 2; attempt++)
    await expect(ros.disposeAsync()).rejects.toMatchObject({
      errors: [syncFailure, asyncFailure],
    });
  expect(() => ros.dispose()).not.toThrow();
});

it("async disposal starts every release before waiting, even when a device throws", async () => {
  const { ros, connection } = await simulation();
  const delayed = deferred();
  const failure = new Error("Device release failed");
  ros.register({ dispose: () => delayed.promise });
  ros.register({
    dispose() {
      throw failure;
    },
  });
  const last = vi.fn();
  ros.register({ dispose: last });
  const completion = ros.disposeAsync();
  expect(last).toHaveBeenCalledOnce();
  expect(connection.disposed).toBe(true);
  delayed.resolve();
  await expect(completion).rejects.toMatchObject({ errors: [failure] });
});

it("logs through the connection, or to stderr where it cannot", async () => {
  const { ros } = await simulation();
  const error = vi.spyOn(console, "error").mockImplementation(() => {});
  ros.log("to stderr");
  expect(error).toHaveBeenCalledWith("to stderr");
  const log = vi.fn();
  Object.assign(ros.connection, { log });
  ros.log("to rosout");
  expect(log).toHaveBeenCalledWith("to rosout");
  error.mockRestore();
});

it("tells devices how long a pause lasted when the simulation plays again", async () => {
  const { ros } = await simulation();
  const resumed = vi.fn();
  ros.register({ resumed, dispose() {} });
  ros.resume(42);
  expect(resumed).toHaveBeenCalledWith(42);
});
