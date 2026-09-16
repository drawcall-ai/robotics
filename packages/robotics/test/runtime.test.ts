import { Device, withDevice } from "../src/device.js";
import { LaserSensor } from "../src/sensors/laser.js";
import { afterEach, expect, it, vi } from "vitest";
import { PerspectiveCamera } from "three";
import { JointMotor, RevoluteJoint, RigidBody } from "@drawcall/physics";
import { setupWorld } from "@drawcall/physics-rapier";
import { ConnectionStub } from "./connection.js";
import { ROS2 } from "../src/runtime.js";
import { Robot } from "../src/robot.js";
import { RendererStub } from "./renderer.js";

const command = (velocity: number) => ({
  name: ["wheel"],
  position: [],
  velocity: [velocity],
  effort: [],
});

const cleanup: (() => void)[] = [];
afterEach(() => {
  for (const release of cleanup.splice(0).reverse()) release();
  vi.restoreAllMocks();
});

async function fixture() {
  const world = await setupWorld({ gravity: [0, 0, 0], fixedDelta: 0.01 });
  cleanup.push(() => world.dispose());
  const transport = new ConnectionStub();
  const ros = new ROS2(transport, {
    world,
    renderer: new RendererStub(),
  });
  cleanup.push(() => ros.dispose());
  const base = new RigidBody({ world, type: "static" });
  base.name = "base_link";
  const body = new RigidBody({
    world,
    mass: 1,
    centerOfMass: [0, 0, 0],
    diagonalInertia: [1, 1, 1],
  });
  body.name = "wheel_link";
  const joint = new RevoluteJoint({ body0: base, body1: body, axis: "Z" });
  joint.name = "wheel";
  const motor = new JointMotor({ joint, damping: 10, maxForce: 20 });
  const robot = new Robot({ ros });
  robot.name = "cart";
  robot.add(base, body, joint);
  return { world, ros, transport, motor, robot, joint };
}

it("selects the latest received command before the solver and publishes completed time", async () => {
  const { world, transport, motor, joint } = await fixture();
  transport.receive("/robot/joint_commands", command(-2));
  transport.receive("/robot/joint_commands", command(3));
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
  transport.receive("/robot/joint_commands", command(3));
  for (let step = 0; step < 60; step++) world.update(0.01);
  expect(motor.enabled).toBe(false);
  transport.receive("/robot/joint_commands", command(3));
  world.update(0.01);
  expect(motor.enabled).toBe(true);
});

it("expires commands during a display pause before resuming the next physics step", async () => {
  const clock = vi.spyOn(performance, "now").mockReturnValue(0);
  const { world, transport, motor } = await fixture();
  transport.receive("/robot/joint_commands", command(2));
  world.update(0.01);
  clock.mockReturnValue(1000);
  world.update(0.01);
  expect(motor.enabled).toBe(false);
});

it("disables motors before another step when disconnected, even with a pending nonzero command", async () => {
  const { world, transport, motor, joint } = await fixture();
  transport.receive("/robot/joint_commands", command(3));
  world.update(0.01);
  const velocity = joint.getState().velocity;
  transport.receive("/robot/joint_commands", command(10));
  transport.connected = false;
  world.update(0.01);
  expect(motor.enabled).toBe(false);
  expect(joint.getState().velocity).toBeCloseTo(velocity, 5);
});

it("surfaces malformed or foreign joint commands and stops an already active actuator", async () => {
  const { world, ros, transport, motor } = await fixture();
  transport.receive("/robot/joint_commands", command(3));
  world.update(0.01);
  transport.receive("/robot/joint_commands", {
    name: ["wheel"],
    velocity: ["3"],
  });
  expect(ros.error?.message).toContain("needs a finite velocity");
  expect(ros.connected).toBe(false);
  expect(motor.enabled).toBe(false);
});

it("rejects commands for joints it does not own", async () => {
  const { ros, transport } = await fixture();
  transport.receive("/robot/joint_commands", {
    name: ["wheel", "arm"],
    velocity: [1, 2],
  });
  expect(ros.error?.message).toContain("unknown joint arm");
});

it("disposes a robot without disposing its motor and releases subscriptions and hooks", async () => {
  const { world, ros, transport, motor, robot } = await fixture();
  transport.receive("/robot/joint_commands", command(2));
  world.update(0.01);
  robot.dispose();
  robot.dispose();
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

it("does not register invalid sensors", async () => {
  const { ros, world } = await fixture();
  const register = vi.spyOn(ros, "register");
  expect(() => new LaserSensor({ ros, samples: 1 })).toThrow("at least two");
  expect(register).not.toHaveBeenCalled();
  expect(() => world.update(0.01)).not.toThrow();
});

it("keeps unchanged native motor settings and restores externally changed target components", async () => {
  const { world, transport, motor } = await fixture();
  vi.spyOn(performance, "now").mockReturnValue(0);
  transport.receive("/robot/joint_commands", command(3));
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

it("allows moving a joint to a robot of a new runtime after subscription cleanup fails", async () => {
  const { world, ros, transport, motor, robot } = await fixture();
  transport.receive("/robot/joint_commands", command(2));
  world.update(0.01);
  expect(motor.enabled).toBe(true);
  vi.spyOn(transport.receivers, "delete").mockImplementationOnce(() => {
    throw new Error("Unsubscribe failed");
  });
  expect(() => ros.dispose()).toThrow("Failed to dispose ROS runtime");
  expect(motor.enabled).toBe(false);
  const nextTransport = new ConnectionStub();
  const next = new ROS2(nextTransport, {
    world,
    renderer: new RendererStub(),
  });
  cleanup.push(() => next.dispose());
  const moved = new Robot({ ros: next });
  moved.name = "cart";
  moved.add(...robot.children);
  nextTransport.receive("/robot/joint_commands", command(2));
  world.update(0.01);
  expect(motor.enabled).toBe(true);
  expect(motor.target?.velocity).toBe(2);
});
