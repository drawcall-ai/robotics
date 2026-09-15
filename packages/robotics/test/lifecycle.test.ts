import { afterEach, expect, it, vi } from "vitest";
import { Scene, WebGLRenderer } from "three";
import { setupWorld } from "@drawcall/physics-rapier";
import { ROS2 } from "../src/runtime.js";

vi.mock("three", async (importOriginal) => {
  const three = await importOriginal<typeof import("three")>();
  return { ...three, WebGLRenderer: class {} };
});

const cleanup: (() => void)[] = [];
afterEach(() => {
  for (const release of cleanup.splice(0).reverse()) release();
});

async function fixture() {
  const world = await setupWorld({ gravity: [0, 0, 0], fixedDelta: 0.01 });
  cleanup.push(() => world.dispose());
  const unsubscribe = vi.fn();
  const connection = {
    connected: true,
    dropped: 0,
    publish: vi.fn(),
    subscribe: () => unsubscribe,
    dispose: vi.fn(),
  };
  const ros = new ROS2(connection, {
    world,
    scene: new Scene(),
    renderer: new WebGLRenderer(),
  });
  cleanup.push(() => ros.dispose());
  return { world, ros, connection, unsubscribe };
}

it("dispatches structural devices without requiring a runtime reference", async () => {
  const { ros, world } = await fixture();
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

it("stops every device after a fault even when an earlier stop fails", async () => {
  const { ros, world } = await fixture();
  const fault = new Error("Sensor failed");
  const stopFailure = new Error("Extension stop failed");
  ros.register({
    afterStep() {
      throw fault;
    },
    stop() {
      throw stopFailure;
    },
    dispose() {},
  });
  const stop = vi.fn();
  ros.register({ stop, dispose() {} });
  expect(() => world.update(0.01)).toThrow("Sensor failed");
  expect(stop).toHaveBeenCalledOnce();
  expect(ros.connected).toBe(false);
  expect(ros.error).toBeInstanceOf(AggregateError);
  expect(ros.error).toMatchObject({
    cause: fault,
    errors: [fault, stopFailure],
  });
});

it("stops every device after disconnection and surfaces stop failures", async () => {
  const { ros, world, connection } = await fixture();
  ros.register({
    stop() {
      throw new Error("Stop failed");
    },
    dispose() {},
  });
  const stop = vi.fn();
  ros.register({ stop, dispose() {} });
  connection.connected = false;
  expect(() => world.update(0.01)).toThrow("Failed to stop ROS devices");
  expect(stop).toHaveBeenCalledOnce();
});

it("releases remaining devices, subscriptions, hooks and connection after disposal fails", async () => {
  const { ros, world, connection, unsubscribe } = await fixture();
  const failure = new Error("Extension disposal failed");
  ros.register({
    dispose() {
      throw failure;
    },
  });
  const dispose = vi.fn();
  ros.register({ dispose });
  const release = ros.subscribe("status", "std_msgs/msg/String", () => {});
  expect(() => ros.dispose()).toThrow("Failed to dispose ROS runtime");
  expect(dispose).toHaveBeenCalledOnce();
  expect(unsubscribe).toHaveBeenCalledOnce();
  expect(connection.dispose).toHaveBeenCalledOnce();
  expect(ros.connected).toBe(false);
  expect(() => ros.dispose()).not.toThrow();
  release();
  expect(unsubscribe).toHaveBeenCalledOnce();
  world.update(0.01);
  expect(connection.publish).not.toHaveBeenCalled();
});
