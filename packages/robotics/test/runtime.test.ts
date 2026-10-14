import { expect, it, vi } from "vitest";
import { Object3D, Scene } from "three";
import { CameraSensor } from "../src/devices/camera.js";
import { LaserSensor } from "../src/devices/laser.js";
import { Robot } from "../src/devices/robot.js";
import { Evaluator } from "../src/evaluation.js";
import type { Device } from "../src/runtime.js";
import { exportURDF } from "../src/urdf/export.js";
import { cart, jointCommand, simulation } from "./fixture.js";

/** Every message inside an error, however it is nested. */
function messages(error: unknown): string[] {
  if (!(error instanceof Error)) return [String(error)];
  const nested = error instanceof AggregateError ? error.errors : [];
  return [error.message, ...[...nested, error.cause].flatMap(messages)];
}

/** A scene object the runtime drives, with the given hooks. */
function device(
  hooks: Partial<Omit<Device, "isDevice" | "busy">> = {},
): Object3D & Device {
  return Object.assign(new Object3D(), {
    isDevice: true as const,
    join() {},
    leave() {},
    ...hooks,
  });
}

it("joins a device that enters the scene at the next step and steps it until disposed", async () => {
  const { ros, world } = await simulation();
  const join = vi.fn();
  const afterStep = vi.fn();
  const leave = vi.fn();
  const plain = device({ join, afterStep, leave });
  world.scene.add(plain);
  expect(ros.find(Object3D)).toEqual([]);
  world.update(0.01);
  expect(join).toHaveBeenCalledWith(ros);
  expect(ros.find(Object3D)).toEqual([plain]);
  expect(afterStep).toHaveBeenCalledWith(0.01, 0.01);
  ros.dispose();
  expect(leave).toHaveBeenCalledOnce();
  world.update(0.01);
  expect(afterStep).toHaveBeenCalledOnce();
});

it("has a device that left the scene leave at the next step, and rejoins it when put back", async () => {
  const { ros, world } = await simulation();
  const join = vi.fn();
  const afterStep = vi.fn();
  const leave = vi.fn();
  const parent = new Object3D();
  parent.add(device({ join, afterStep, leave }));
  world.scene.add(parent);
  world.update(0.01);
  parent.removeFromParent();
  expect(leave).not.toHaveBeenCalled();
  world.update(0.01);
  expect(leave).toHaveBeenCalledOnce();
  expect(afterStep).toHaveBeenCalledOnce();
  expect(ros.find(Object3D)).toEqual([]);
  world.scene.add(parent);
  ros.sync();
  expect(join).toHaveBeenCalledTimes(2);
  world.update(0.01);
  expect(afterStep).toHaveBeenCalledTimes(2);
});

it("stops every device and stays faulted after a device throws in a step", async () => {
  const { ros, world } = await simulation();
  const stop = vi.fn();
  world.scene.add(
    device({
      afterStep() {
        throw new Error("Sensor failed");
      },
      stop() {
        throw new Error("Extension stop failed");
      },
    }),
    device({ stop }),
  );
  expect(() => world.update(0.01)).toThrow("Sensor failed");
  expect(stop).toHaveBeenCalledOnce();
  expect(ros.connected).toBe(false);
  expect(ros.error?.message).toContain("Sensor failed");
  expect(messages(ros.error)).toContain("Extension stop failed");
});

it("stops every device once after the connection is lost", async () => {
  const { ros, world, connection } = await simulation();
  const stop = vi.fn();
  world.scene.add(
    device({
      stop() {
        throw new Error("Stop failed");
      },
    }),
    device({ stop }),
  );
  ros.sync();
  connection.connected = false;
  expect(() => world.update(0.01)).not.toThrow();
  expect(ros.connected).toBe(false);
  expect(ros.error?.message).toContain("ROS connection lost");
  expect(messages(ros.error)).toContain("Stop failed");
  world.update(0.01);
  expect(stop).toHaveBeenCalledOnce();
});

it("releases everything else when a device fails to leave", async () => {
  const { ros, world, connection } = await simulation();
  const leave = vi.fn();
  world.scene.add(
    device({
      leave() {
        throw new Error("Extension disposal failed");
      },
    }),
    device({ leave }),
  );
  ros.sync();
  const release = ros.subscribe("/status", "std_msgs/msg/String", () => {});
  expect(() => ros.dispose()).toThrow();
  expect(leave).toHaveBeenCalledOnce();
  expect(connection.disposed).toBe(true);
  expect(ros.connected).toBe(false);
  expect(() => ros.dispose()).not.toThrow();
  expect(() => release()).not.toThrow();
  world.update(0.01);
  expect(connection.messages).toHaveLength(0);
});

it("takes only absolute topics", async () => {
  const { ros } = await simulation();
  expect(() => ros.publish("status", "std_msgs/msg/String", {})).toThrow(
    "must be absolute",
  );
  expect(() =>
    ros.subscribe("status", "std_msgs/msg/String", () => {}),
  ).toThrow("must be absolute");
  expect(() => ros.publish("/bad name", "std_msgs/msg/String", {})).toThrow(
    "Invalid ROS topic",
  );
});

it("drives several runtimes at once, each over its own scene", async () => {
  const first = await simulation();
  const second = await simulation();
  const one = cart();
  const two = cart();
  first.world.scene.add(one.robot);
  second.world.scene.add(two.robot);
  first.ros.sync();
  second.ros.sync();
  first.connection.receive(
    "/robot/joint_commands",
    jointCommand("velocity", 1),
  );
  second.connection.receive(
    "/robot/joint_commands",
    jointCommand("velocity", 2),
  );
  first.world.update(0.01);
  second.world.update(0.01);
  expect(one.actuator.target?.velocity).toBe(1);
  expect(two.actuator.target?.velocity).toBe(2);
  expect(first.ros.find(Robot)).toEqual([one.robot]);
});

it("builds devices and exports a robot's URDF with no runtime", () => {
  const { robot, base } = cart();
  base.add(new CameraSensor(), new LaserSensor());
  const evaluation = new Evaluator({ timeout: 1, success: {}, score: () => 0 });
  const scene = new Scene().add(robot, evaluation);
  expect(exportURDF(scene)).toContain('<robot name="cart">');
  expect(() => evaluation.start()).toThrow("not under a ROS runtime");
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
  const leave = vi.fn(() => first.promise);
  world.scene.add(
    device({ afterStep, leave }),
    device({ leave: () => second.promise }),
  );
  ros.sync();
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
  expect(leave).toHaveBeenCalledOnce();
});

it("retains synchronous and asynchronous release failures for every async join", async () => {
  const { ros, world, connection } = await simulation();
  const readback = deferred();
  const syncFailure = new Error("Device release failed");
  const asyncFailure = new Error("Readback failed");
  world.scene.add(
    device({
      leave() {
        throw syncFailure;
      },
    }),
    device({ leave: () => readback.promise }),
  );
  ros.sync();
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
  const { ros, world, connection } = await simulation();
  const delayed = deferred();
  const failure = new Error("Device release failed");
  const last = vi.fn();
  world.scene.add(
    device({ leave: () => delayed.promise }),
    device({
      leave() {
        throw failure;
      },
    }),
    device({ leave: last }),
  );
  ros.sync();
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
  const { ros, world } = await simulation();
  const resumed = vi.fn();
  world.scene.add(device({ resumed }));
  ros.sync();
  ros.resume(42);
  expect(resumed).toHaveBeenCalledWith(42);
});
