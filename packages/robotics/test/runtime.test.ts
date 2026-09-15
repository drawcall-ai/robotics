import { Device, withDevice } from "../src/device.js";
import { LaserSensor } from "../src/sensors/laser.js";
import { afterEach, expect, it, vi } from "vitest";
import { PerspectiveCamera, Scene, WebGLRenderer } from "three";
import { JointMotor, RevoluteJoint, RigidBody } from "@drawcall/physics";
import { setupWorld } from "@drawcall/physics-rapier";
import type { Connection } from "../src/connection.js";
import { ROS2 } from "../src/runtime.js";
import { MotorBinding } from "../src/motor.js";

vi.mock("three", async (importOriginal) => {
  const three = await importOriginal<typeof import("three")>();
  return { ...three, WebGLRenderer: class {} };
});

class Transport implements Connection {
  connected = true;
  dropped = 0;
  readonly messages: { name: string; type: string; message: unknown }[] = [];
  readonly receivers = new Map<string, (message: unknown) => void>();
  publish(name: string, type: string, message: unknown): void {
    this.messages.push({ name, type, message });
  }
  subscribe(
    name: string,
    _type: string,
    receive: (message: unknown) => void,
  ): () => void {
    this.receivers.set(name, receive);
    return () => {
      this.receivers.delete(name);
    };
  }
  receive(name: string, message: unknown): void {
    const receive = this.receivers.get(name);
    if (!receive) throw new Error(`No subscription for ${name}`);
    receive(message);
  }
  dispose(): void {
    this.connected = false;
    this.receivers.clear();
  }
}

const cleanup: (() => void)[] = [];
afterEach(() => {
  for (const release of cleanup.splice(0).reverse()) release();
  vi.restoreAllMocks();
});

async function fixture() {
  const world = await setupWorld({ gravity: [0, 0, 0], fixedDelta: 0.01 });
  cleanup.push(() => world.dispose());
  const transport = new Transport();
  const ros = new ROS2(transport, {
    world,
    scene: new Scene(),
    renderer: new WebGLRenderer(),
  });
  cleanup.push(() => ros.dispose());
  const body = new RigidBody({
    mass: 1,
    centerOfMass: [0, 0, 0],
    diagonalInertia: [1, 1, 1],
  });
  const joint = new RevoluteJoint({ body0: null, body1: body, axis: "Z" });
  const motor = new JointMotor({ joint, damping: 10, maxForce: 20 });
  const binding = new MotorBinding({
    motor,
    name: "wheel",
    mode: "velocity",
    timeout: 0.05,
    ros,
  });
  return { world, ros, transport, motor, binding, joint };
}

it("selects the latest received command before the solver and publishes completed time", async () => {
  const { world, transport, motor, joint } = await fixture();
  expect(motor.enabled).toBe(false);
  transport.receive("/robot/wheel/command", { data: -2 });
  transport.receive("/robot/wheel/command", { data: 3 });
  expect(motor.enabled).toBe(false);
  world.update(0.01);
  expect(motor.target?.velocity).toBe(3);
  expect(joint.getState().velocity).toBeGreaterThan(0);
  expect(transport.messages).toContainEqual({
    name: "/clock",
    type: "rosgraph_msgs/msg/Clock",
    message: { clock: { sec: 0, nanosec: 10_000_000 } },
  });
});

it("expires persistent actuation by simulation time and accepts a fresh repeated target", async () => {
  vi.spyOn(performance, "now").mockReturnValue(0);
  const { world, transport, motor } = await fixture();
  transport.receive("/robot/wheel/command", { data: 3 });
  for (let step = 0; step < 8; step++) world.update(0.01);
  expect(motor.enabled).toBe(false);
  transport.receive("/robot/wheel/command", { data: 3 });
  world.update(0.01);
  expect(motor.enabled).toBe(true);
});

it("expires commands during a display pause before resuming the next physics step", async () => {
  const clock = vi.spyOn(performance, "now").mockReturnValue(0);
  const { world, transport, motor } = await fixture();
  transport.receive("/robot/wheel/command", { data: 2 });
  world.update(0.01);
  clock.mockReturnValue(100);
  world.update(0.01);
  expect(motor.enabled).toBe(false);
});

it("disables motors before another step when disconnected, even with a pending nonzero command", async () => {
  const { world, transport, motor, joint } = await fixture();
  transport.receive("/robot/wheel/command", { data: 3 });
  world.update(0.01);
  const velocity = joint.getState().velocity;
  transport.receive("/robot/wheel/command", { data: 10 });
  transport.connected = false;
  world.update(0.01);
  expect(motor.enabled).toBe(false);
  expect(joint.getState().velocity).toBeCloseTo(velocity, 5);
});

it("surfaces malformed ROS commands and stops an already active actuator", async () => {
  const { world, ros, transport, motor } = await fixture();
  transport.receive("/robot/wheel/command", { data: 3 });
  world.update(0.01);
  transport.receive("/robot/wheel/command", { data: "3" });
  expect(ros.error?.message).toContain("Invalid wheel motor command");
  expect(ros.connected).toBe(false);
  expect(motor.enabled).toBe(false);
});

it("disposes a binding without disposing its motor and releases subscriptions and hooks", async () => {
  const { world, ros, transport, motor, binding } = await fixture();
  transport.receive("/robot/wheel/command", { data: 2 });
  world.update(0.01);
  binding.dispose();
  binding.dispose();
  expect(motor.disposed).toBe(false);
  expect(motor.enabled).toBe(false);
  expect(transport.receivers.size).toBe(0);
  ros.dispose();
  const count = transport.messages.length;
  world.update(0.01);
  expect(transport.messages).toHaveLength(count);
});

it("inherits device registration and disposal without a custom constructor", async () => {
  const { ros, world } = await fixture();
  class Diagnostic extends Device {
    samples = 0;
    afterStep() {
      this.samples++;
    }
  }
  const diagnostic = new Diagnostic({ ros });
  world.update(0.01);
  expect(diagnostic.samples).toBe(1);
  diagnostic.dispose();
  diagnostic.dispose();
  world.update(0.01);
  expect(diagnostic.samples).toBe(1);
});

it("preserves Three.js inheritance and constructor arguments while dispatching to the final instance", async () => {
  const { ros, world } = await fixture();
  class Camera extends withDevice(PerspectiveCamera) {
    samples = 0;
    afterStep() {
      this.samples++;
    }
  }
  const camera = new Camera({ ros, args: [42, 2, 0.2, 50] });
  expect(camera).toBeInstanceOf(PerspectiveCamera);
  expect(camera.fov).toBe(42);
  expect(camera.aspect).toBe(2);
  expect(camera.near).toBe(0.2);
  expect(camera.far).toBe(50);
  world.update(0.01);
  expect(camera.samples).toBe(1);
  ros.dispose();
  world.update(0.01);
  expect(camera.samples).toBe(1);
});

it("does not register invalid sensors or leave a duplicate motor binding behind", async () => {
  const { ros, world, motor } = await fixture();
  const register = vi.spyOn(ros, "register");
  expect(() => new LaserSensor({ ros, samples: 1 })).toThrow("at least two");
  expect(
    () => new MotorBinding({ ros, motor, name: "duplicate", mode: "velocity" }),
  ).toThrow("already bound");
  expect(register).not.toHaveBeenCalled();
  expect(() => world.update(0.01)).not.toThrow();
});

it("keeps unchanged native motor settings and restores externally changed target components", async () => {
  const { world, transport, motor } = await fixture();
  vi.spyOn(performance, "now").mockReturnValue(0);
  transport.receive("/robot/wheel/command", { data: 3 });
  world.update(0.01);
  const version = motor.settingsVersion;
  world.update(0.01);
  expect(motor.settingsVersion).toBe(version);
  motor.setTarget({ position: 2, velocity: 3 });
  world.update(0.01);
  expect(motor.target).toEqual({ position: 0, velocity: 3 });
  motor.setEnabled(false);
  world.update(0.01);
  expect(motor.enabled).toBe(true);
});

it("allows rebinding an application motor after subscription cleanup fails", async () => {
  const { world, ros, transport, motor } = await fixture();
  vi.spyOn(transport.receivers, "delete").mockImplementationOnce(() => {
    throw new Error("Unsubscribe failed");
  });
  expect(() => ros.dispose()).toThrow("Failed to dispose ROS runtime");
  expect(motor.enabled).toBe(false);
  const nextTransport = new Transport();
  const next = new ROS2(nextTransport, {
    world,
    scene: new Scene(),
    renderer: new WebGLRenderer(),
  });
  cleanup.push(() => next.dispose());
  new MotorBinding({ ros: next, motor, name: "wheel", mode: "velocity" });
  nextTransport.receive("/robot/wheel/command", { data: 2 });
  world.update(0.01);
  expect(motor.enabled).toBe(true);
  expect(motor.target?.velocity).toBe(2);
});
