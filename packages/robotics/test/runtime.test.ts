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
  const { actuator } = cart(ros);
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
  cart(ros);
  world.update(0.01);
  expect(connection.messages.map(({ name }) => name)).toContain(
    "/joint_states",
  );
});

it("rejects an invalid namespace", () => {
  expect(() => offline("bot")).toThrow("Invalid ROS namespace");
});
