import { expect, it, vi } from "vitest";
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
  const { joint, motor } = cart(ros);
  connection.receive(COMMANDS, jointCommand("velocity", -2));
  connection.receive(COMMANDS, jointCommand("velocity", 3));
  world.update(0.01);
  expect(motor.target?.velocity).toBe(3);
  expect(joint.getState().velocity).toBeGreaterThan(0);
});

it("commands a position through a stiff motor", async () => {
  const { world, ros, connection } = await simulation();
  const { motor } = cart(ros, { stiffness: 50 });
  connection.receive(COMMANDS, jointCommand("position", 1.2));
  world.update(0.01);
  expect(motor.target).toEqual({ position: 1.2, velocity: 0 });
  connection.receive(COMMANDS, jointCommand("velocity", 1));
  expect(ros.error?.message).toContain("needs a finite position");
});

it("lets a command expire by simulation time and accepts a repeated one", async () => {
  vi.spyOn(performance, "now").mockReturnValue(0);
  const { world, ros, connection } = await simulation();
  const { motor } = cart(ros);
  connection.receive(COMMANDS, jointCommand("velocity", 3));
  for (let step = 0; step < 60; step++) world.update(0.01);
  expect(motor.enabled).toBe(false);
  connection.receive(COMMANDS, jointCommand("velocity", 3));
  world.update(0.01);
  expect(motor.enabled).toBe(true);
});

it("lets a command expire by wall time across a pause", async () => {
  const clock = vi.spyOn(performance, "now").mockReturnValue(0);
  const { world, ros, connection } = await simulation();
  const { motor } = cart(ros);
  connection.receive(COMMANDS, jointCommand("velocity", 2));
  world.update(0.01);
  clock.mockReturnValue(1000);
  world.update(0.01);
  expect(motor.enabled).toBe(false);
});

it("stops the motor before the next step once disconnected", async () => {
  const { world, ros, connection } = await simulation();
  const { joint, motor } = cart(ros);
  connection.receive(COMMANDS, jointCommand("velocity", 3));
  world.update(0.01);
  const velocity = joint.getState().velocity;
  connection.receive(COMMANDS, jointCommand("velocity", 10));
  connection.connected = false;
  world.update(0.01);
  expect(motor.enabled).toBe(false);
  expect(joint.getState().velocity).toBeCloseTo(velocity, 5);
});

it("faults on a malformed command and stops the motor", async () => {
  const { world, ros, connection } = await simulation();
  const { motor } = cart(ros);
  connection.receive(COMMANDS, jointCommand("velocity", 3));
  world.update(0.01);
  connection.receive(COMMANDS, { name: ["wheel"], velocity: ["3"] });
  expect(ros.error?.message).toContain("needs a finite velocity");
  expect(ros.connected).toBe(false);
  expect(motor.enabled).toBe(false);
});

it("faults on a command for a joint it does not have", async () => {
  const { ros, connection } = await simulation();
  cart(ros);
  connection.receive(COMMANDS, { name: ["wheel", "arm"], velocity: [1, 2] });
  expect(ros.error?.message).toContain("unknown joint arm");
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

it("stops listening and driving on dispose without disposing the motor", async () => {
  const { world, ros, connection } = await simulation();
  const { robot, motor } = cart(ros);
  connection.receive(COMMANDS, jointCommand("velocity", 2));
  world.update(0.01);
  robot.dispose();
  robot.dispose();
  expect(motor.disposed).toBe(false);
  expect(motor.enabled).toBe(false);
  expect(() =>
    connection.receive(COMMANDS, jointCommand("velocity", 2)),
  ).toThrow("No subscription");
  ros.dispose();
  const count = connection.messages.length;
  world.update(0.01);
  expect(connection.messages).toHaveLength(count);
});

it("leaves an unchanged target alone and restores an externally changed motor", async () => {
  vi.spyOn(performance, "now").mockReturnValue(0);
  const { world, ros, connection } = await simulation();
  const { motor } = cart(ros);
  connection.receive(COMMANDS, jointCommand("velocity", 3));
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

it("drives the motor again from a robot of a new runtime over the same joint", async () => {
  const { world, ros, connection } = await simulation();
  const { robot, motor } = cart(ros);
  connection.receive(COMMANDS, jointCommand("velocity", 2));
  world.update(0.01);
  expect(motor.enabled).toBe(true);
  ros.dispose();
  expect(motor.enabled).toBe(false);
  const nextConnection = new ConnectionStub();
  const next = new ROS2(nextConnection, {
    world,
    renderer: new RendererStub(),
  });
  defer(() => next.dispose());
  const moved = new Robot({ ros: next });
  moved.name = "cart";
  moved.add(...robot.children);
  nextConnection.receive(COMMANDS, jointCommand("velocity", 2));
  world.update(0.01);
  expect(motor.enabled).toBe(true);
  expect(motor.target?.velocity).toBe(2);
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
