import { expect, it, vi } from "vitest";
import { JointActuator } from "../src/devices/actuator.js";
import { findRobot, Robot } from "../src/devices/robot.js";
import { ROS2 } from "../src/runtime.js";
import {
  cart,
  ConnectionStub,
  defer,
  jointCommand,
  offline,
  RendererStub,
  simulation,
} from "./fixture.js";

const COMMANDS = "/robot/joint_commands";

it("drives the joint with the latest command", async () => {
  const { world, ros, connection } = await simulation();
  const { joint, actuator } = cart(ros);
  connection.receive(COMMANDS, jointCommand("velocity", -2));
  connection.receive(COMMANDS, jointCommand("velocity", 3));
  world.update(0.01);
  expect(actuator.target?.velocity).toBe(3);
  expect(joint.getState().velocity).toBeGreaterThan(0);
});

it("commands a position through a stiff actuator", async () => {
  const { world, ros, connection } = await simulation();
  const { actuator } = cart(ros, { stiffness: 50 });
  connection.receive(COMMANDS, jointCommand("position", 1.2));
  world.update(0.01);
  expect(actuator.target).toEqual({ position: 1.2, velocity: 0, effort: 0 });
  connection.receive(COMMANDS, jointCommand("velocity", 1));
  expect(ros.error?.message).toContain("needs a finite position");
});

it("commands a torque through an effort actuator", async () => {
  const { world, ros, connection } = await simulation();
  const { joint, actuator } = cart(ros, { interface: "effort" });
  connection.receive(COMMANDS, jointCommand("effort", 2));
  world.update(0.01);
  expect(actuator.target).toEqual({ position: 0, velocity: 0, effort: 2 });
  expect(joint.getState().velocity).toBeCloseTo(0.02, 5);
});

it("requires the gains its interface acts through", async () => {
  const { ros } = await simulation();
  expect(() => new JointActuator({ ros, interface: "position" })).toThrow(
    "needs stiffness",
  );
  expect(
    () => new JointActuator({ ros, interface: "velocity", stiffness: 1 }),
  ).toThrow("no stiffness");
  expect(
    () => new JointActuator({ ros, interface: "effort", damping: 1 }),
  ).toThrow("no gains");
  expect(
    () => new JointActuator({ ros, interface: "effort", maxVelocity: -1 }),
  ).toThrow("maximum velocity must be finite and positive");
});

it("holds its joint up until commanded, then releases once a command expires", async () => {
  const { world, ros, connection } = await simulation({
    gravity: [0, -9.81, 0],
  });
  const { joint, actuator } = cart(ros, { stiffness: 100 });
  // An arm whose controller has not connected yet is still powered, so it holds.
  for (let step = 0; step < 100; step++) world.update(0.01);
  expect(actuator.target).toBeDefined();
  expect(Math.abs(joint.getState().position)).toBeLessThan(0.05);
  connection.receive(COMMANDS, jointCommand("position", 0.5));
  for (let step = 0; step < 20; step++) {
    connection.receive(COMMANDS, jointCommand("position", 0.5));
    world.update(0.01);
  }
  expect(joint.getState().position).toBeGreaterThan(0.3);
  // A controller that stops talking lets go, rather than reverting to the old pose.
  for (let step = 0; step < 100; step++) world.update(0.01);
  expect(actuator.target).toBeUndefined();
});

it("lets a command expire by simulation time and accepts a repeated one", async () => {
  const { world, ros, connection } = await simulation();
  const { actuator } = cart(ros);
  connection.receive(COMMANDS, jointCommand("velocity", 3));
  for (let step = 0; step < 60; step++) world.update(0.01);
  expect(actuator.target).toBeUndefined();
  connection.receive(COMMANDS, jointCommand("velocity", 3));
  world.update(0.01);
  expect(actuator.target?.velocity).toBe(3);
});

it("keeps a command across a pause, as a sim-time controller stops publishing while paused", async () => {
  const clock = vi.spyOn(performance, "now").mockReturnValue(0);
  const { world, ros, connection } = await simulation();
  const { actuator } = cart(ros);
  connection.receive(COMMANDS, jointCommand("velocity", 2));
  world.update(0.01);
  clock.mockReturnValue(1000);
  world.update(0.01);
  expect(actuator.target?.velocity).toBe(2);
});

it("goes passive before the next step once disconnected", async () => {
  const { world, ros, connection } = await simulation();
  const { joint, actuator } = cart(ros);
  connection.receive(COMMANDS, jointCommand("velocity", 3));
  world.update(0.01);
  const velocity = joint.getState().velocity;
  connection.receive(COMMANDS, jointCommand("velocity", 10));
  connection.connected = false;
  world.update(0.01);
  expect(actuator.target).toBeUndefined();
  expect(joint.getState().velocity).toBeCloseTo(velocity, 5);
});

it("faults on a malformed command and goes passive", async () => {
  const { world, ros, connection } = await simulation();
  const { actuator } = cart(ros);
  connection.receive(COMMANDS, jointCommand("velocity", 3));
  world.update(0.01);
  connection.receive(COMMANDS, { name: ["wheel"], velocity: ["3"] });
  expect(ros.error?.message).toContain("needs a finite velocity");
  expect(ros.connected).toBe(false);
  expect(actuator.target).toBeUndefined();
});

it("ignores commands for other joints and rejects targets from the scene", async () => {
  const { world, ros, connection } = await simulation();
  const { joint, actuator } = cart(ros);
  const spare = new JointActuator({ ros, interface: "velocity", damping: 1 });
  connection.receive(COMMANDS, jointCommand("velocity", 3));
  connection.receive(COMMANDS, jointCommand("velocity", 5, "arm"));
  world.update(0.01);
  expect(spare.target).toBeUndefined();
  expect(actuator.target?.velocity).toBe(3);
  expect(() => actuator.setTarget()).toThrow("joint_commands");
  const version = actuator.settingsVersion;
  world.update(0.01);
  expect(actuator.settingsVersion).toBe(version);
  expect(joint.drive).toBe(actuator);
});

it("publishes the joint state", async () => {
  const { world, ros, connection } = await simulation();
  const { joint } = cart(ros);
  world.update(0.01);
  const state = joint.getState();
  expect(connection.messages).toContainEqual({
    name: "/robot/joint_states",
    type: "sensor_msgs/msg/JointState",
    message: {
      header: { stamp: { sec: 0, nanosec: 10_000_000 }, frame_id: "" },
      name: ["wheel"],
      position: [state.position],
      velocity: [state.velocity],
      effort: [],
    },
  });
});

it("detaches and stops listening on dispose", async () => {
  const { world, ros, connection } = await simulation();
  const { robot, joint, actuator } = cart(ros);
  connection.receive(COMMANDS, jointCommand("velocity", 2));
  world.update(0.01);
  actuator.dispose();
  actuator.dispose();
  expect(actuator.target).toBeUndefined();
  expect(joint.drive).toBeUndefined();
  expect(() =>
    connection.receive(COMMANDS, jointCommand("velocity", 2)),
  ).toThrow("No subscription");
  robot.dispose();
  robot.dispose();
  ros.dispose();
  const count = connection.messages.length;
  world.update(0.01);
  expect(connection.messages).toHaveLength(count);
});

it("drives the joint again from a new runtime with a new actuator", async () => {
  const { world, ros, connection } = await simulation();
  const { robot, joint, actuator } = cart(ros);
  connection.receive(COMMANDS, jointCommand("velocity", 2));
  world.update(0.01);
  expect(actuator.target?.velocity).toBe(2);
  ros.dispose();
  expect(actuator.target).toBeUndefined();
  expect(joint.drive).toBeUndefined();
  const nextConnection = new ConnectionStub();
  const next = new ROS2(nextConnection, {
    world,
    renderer: new RendererStub(),
  });
  defer(() => next.dispose());
  const moved = new Robot({ ros: next });
  moved.name = "cart";
  moved.add(...robot.children);
  const replacement = new JointActuator({
    ros: next,
    interface: "velocity",
    damping: 10,
  });
  joint.setDrive(replacement);
  nextConnection.receive(COMMANDS, jointCommand("velocity", 2));
  world.update(0.01);
  expect(replacement.target?.velocity).toBe(2);
});

it("finds the only robot, or one by name when a scene has several", () => {
  const ros = offline();
  expect(() => findRobot(ros)).toThrow("has 0 robots");
  const { robot } = cart(ros);
  robot.name = "one";
  expect(findRobot(ros)).toBe(robot);
  cart(ros).robot.name = "two";
  expect(() => findRobot(ros)).toThrow("pick one by name");
  expect(findRobot(ros, "two").name).toBe("two");
  expect(() => findRobot(ros, "three")).toThrow("no robot named three");
});

it("stops and detaches an actuator even when releasing its subscription fails", async () => {
  const { world, ros, connection } = await simulation();
  const subscribe = connection.subscribe.bind(connection);
  const failure = new Error("Subscription release failed");
  vi.spyOn(connection, "subscribe").mockImplementation((...args) => {
    const release = subscribe(...args);
    return () => {
      release();
      throw failure;
    };
  });
  const { actuator, joint } = cart(ros);
  connection.receive(COMMANDS, jointCommand("velocity", 2));
  world.update(0.01);
  expect(actuator.target?.velocity).toBe(2);
  expect(() => actuator.dispose()).toThrow(
    expect.objectContaining({ errors: [failure] }),
  );
  expect(actuator.target).toBeUndefined();
  expect(joint.drive).toBeUndefined();
  expect(ros.find(JointActuator)).toEqual([]);
  expect(() => ros.dispose()).not.toThrow();
});
