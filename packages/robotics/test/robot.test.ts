import { expect, it, vi } from "vitest";
import { RigidBody } from "@drawcall/physics";
import { Scene } from "three";
import { JointActuator } from "../src/devices/actuator.js";
import { CameraSensor } from "../src/devices/camera.js";
import { findRobot, Robot } from "../src/devices/robot.js";
import { ROS2 } from "../src/runtime.js";
import {
  cart,
  ConnectionStub,
  defer,
  jointCommand,
  RendererStub,
  simulation,
} from "./fixture.js";

const COMMANDS = "/robot/joint_commands";

it("drives the joint with the latest command", async () => {
  const { world, ros, connection } = await simulation();
  const { robot, joint, actuator } = cart();
  world.scene.add(robot);
  ros.sync();
  connection.receive(COMMANDS, jointCommand("velocity", -2));
  connection.receive(COMMANDS, jointCommand("velocity", 3));
  world.update(0.01);
  expect(actuator.target?.velocity).toBe(3);
  expect(joint.getState().velocity).toBeGreaterThan(0);
});
it("commands a position through a stiff actuator", async () => {
  const { world, ros, connection } = await simulation();
  const { robot, actuator } = cart({ stiffness: 50 });
  world.scene.add(robot);
  ros.sync();
  connection.receive(COMMANDS, jointCommand("position", 1.2));
  world.update(0.01);
  expect(actuator.target).toEqual({ position: 1.2, velocity: 0, effort: 0 });
  connection.receive(COMMANDS, jointCommand("velocity", 1));
  expect(ros.error?.message).toContain("needs a finite position");
});
it("commands a torque through an effort actuator", async () => {
  const { world, ros, connection } = await simulation();
  const { robot, joint, actuator } = cart({ interface: "effort" });
  world.scene.add(robot);
  ros.sync();
  connection.receive(COMMANDS, jointCommand("effort", 2));
  world.update(0.01);
  expect(actuator.target).toEqual({ position: 0, velocity: 0, effort: 2 });
  expect(joint.getState().velocity).toBeCloseTo(0.02, 5);
});
it("requires the gains its interface acts through", () => {
  expect(() => new JointActuator({ interface: "position" })).toThrow(
    "needs stiffness",
  );
  expect(
    () => new JointActuator({ interface: "velocity", stiffness: 1 }),
  ).toThrow("no stiffness");
  expect(() => new JointActuator({ interface: "effort", damping: 1 })).toThrow(
    "no gains",
  );
  expect(
    () => new JointActuator({ interface: "effort", maxVelocity: -1 }),
  ).toThrow("maximum velocity must be finite and positive");
});
it("holds its joint up until commanded, then releases once a command expires", async () => {
  const { world, connection } = await simulation({
    gravity: [0, -9.81, 0],
  });
  const { robot, joint, actuator } = cart({ stiffness: 100 });
  world.scene.add(robot);
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
it("holds where a released joint is, as if never commanded", async () => {
  const { world, ros, connection } = await simulation({
    gravity: [0, -9.81, 0],
  });
  const { robot, joint, actuator } = cart({ stiffness: 100 });
  world.scene.add(robot);
  ros.sync();
  for (let step = 0; step < 30; step++) {
    connection.receive(COMMANDS, jointCommand("position", 0.5));
    world.update(0.01);
  }
  const position = joint.getState().position;
  actuator.release();
  // Long past the command's expiry, it still holds instead of letting go.
  for (let step = 0; step < 100; step++) world.update(0.01);
  expect(actuator.target?.position).toBeCloseTo(position, 1);
});
it("lets a command expire by simulation time and accepts a repeated one", async () => {
  const { world, ros, connection } = await simulation();
  const { robot, actuator } = cart();
  world.scene.add(robot);
  ros.sync();
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
  const { robot, actuator } = cart();
  world.scene.add(robot);
  ros.sync();
  connection.receive(COMMANDS, jointCommand("velocity", 2));
  world.update(0.01);
  clock.mockReturnValue(1000);
  world.update(0.01);
  expect(actuator.target?.velocity).toBe(2);
});
it("goes passive before the next step once disconnected", async () => {
  const { world, ros, connection } = await simulation();
  const { robot, joint, actuator } = cart();
  world.scene.add(robot);
  ros.sync();
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
  const { robot, actuator } = cart();
  world.scene.add(robot);
  ros.sync();
  connection.receive(COMMANDS, jointCommand("velocity", 3));
  world.update(0.01);
  connection.receive(COMMANDS, { name: ["wheel"], velocity: ["3"] });
  expect(ros.error?.message).toContain("needs a finite velocity");
  expect(ros.connected).toBe(false);
  expect(actuator.target).toBeUndefined();
});
it("ignores commands for other joints and rejects targets from the scene", async () => {
  const { world, ros, connection } = await simulation();
  const { robot, joint, actuator } = cart();
  world.scene.add(robot);
  ros.sync();
  const spare = new JointActuator({ interface: "velocity", damping: 1 });
  connection.receive(COMMANDS, jointCommand("velocity", 3));
  connection.receive(COMMANDS, jointCommand("velocity", 5, "arm"));
  world.update(0.01);
  expect(spare.target).toBeUndefined();
  expect(actuator.target?.velocity).toBe(3);
  expect(() => actuator.setTarget()).toThrow("joint_commands");
  const version = joint.version;
  world.update(0.01);
  expect(joint.version).toBe(version);
  expect(joint.drive).toBe(actuator);
});
it("publishes the joint state", async () => {
  const { world, connection } = await simulation();
  const { robot, joint } = cart();
  world.scene.add(robot);
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

it("joins when added to the scene of a running simulation, at the next step", async () => {
  const { world, connection } = await simulation();
  for (let step = 0; step < 5; step++) world.update(0.01);
  const { robot, actuator } = cart();
  world.scene.add(robot);
  expect(() =>
    connection.receive(COMMANDS, jointCommand("velocity", 2)),
  ).toThrow("No subscription");
  world.update(0.01);
  expect(connection.messages.map(({ name }) => name)).toContain(
    "/robot/joint_states",
  );
  connection.receive(COMMANDS, jointCommand("velocity", 2));
  world.update(0.01);
  expect(actuator.target?.velocity).toBe(2);
});

it("leaves at the next step once removed from the scene: stops listening and lets its joints go", async () => {
  const { world, ros, connection } = await simulation();
  const { robot, actuator } = cart();
  world.scene.add(robot);
  ros.sync();
  connection.receive(COMMANDS, jointCommand("velocity", 2));
  world.update(0.01);
  expect(actuator.target?.velocity).toBe(2);
  robot.removeFromParent();
  world.update(0.01);
  expect(ros.find(Robot)).toEqual([]);
  expect(actuator.target).toBeUndefined();
  expect(() =>
    connection.receive(COMMANDS, jointCommand("velocity", 2)),
  ).toThrow("No subscription");
  const count = connection.messages.length;
  world.update(0.01);
  expect(
    connection.messages.slice(count).map(({ name }) => name),
  ).not.toContain("/robot/joint_states");
  // Put back, it joins afresh and holds still, as if never commanded.
  world.scene.add(robot);
  world.update(0.01);
  expect(actuator.target).toEqual({ position: 0, velocity: 0, effort: 0 });
  connection.receive(COMMANDS, jointCommand("velocity", 3));
  world.update(0.01);
  expect(actuator.target?.velocity).toBe(3);
});

it("lets its joints go when the runtime ends, and is driven again by the next runtime", async () => {
  const { world, ros, connection } = await simulation();
  const { robot, actuator } = cart();
  world.scene.add(robot);
  ros.sync();
  connection.receive(COMMANDS, jointCommand("velocity", 2));
  world.update(0.01);
  expect(actuator.target?.velocity).toBe(2);
  ros.dispose();
  expect(actuator.target).toBeUndefined();
  const count = connection.messages.length;
  world.update(0.01);
  expect(connection.messages).toHaveLength(count);
  const nextConnection = new ConnectionStub();
  const next = new ROS2(nextConnection, {
    scene: world.scene,
    world,
    renderer: new RendererStub(),
  });
  defer(() => next.dispose());
  next.sync();
  nextConnection.receive(COMMANDS, jointCommand("velocity", 2));
  world.update(0.01);
  expect(actuator.target?.velocity).toBe(2);
});

it("finds the only robot under a scene, or one by name when it has several", () => {
  const scene = new Scene();
  expect(() => findRobot(scene)).toThrow("has 0 robots");
  const { robot } = cart();
  robot.name = "one";
  scene.add(robot);
  expect(findRobot(scene)).toBe(robot);
  const two = new Robot({ namespace: "/two" });
  two.name = "two";
  scene.add(two);
  expect(() => findRobot(scene)).toThrow("pick one by name");
  expect(findRobot(scene, "two")).toBe(two);
  expect(() => findRobot(scene, "three")).toThrow("no robot named three");
});

it("checks its namespace when built and prefixes frames only below the root", () => {
  expect(() => new Robot({ namespace: "bot" })).toThrow(
    "Invalid ROS namespace: bot",
  );
  const root = new Robot({ namespace: "/" });
  expect(root.topic("joint_states")).toBe("/joint_states");
  expect(root.frame("base_link")).toBe("base_link");
  expect(new Robot({ namespace: "/arm1" }).frame("base_link")).toBe(
    "arm1/base_link",
  );
});

it("refuses to join beside a robot of the same namespace", async () => {
  const { world, ros } = await simulation();
  const first = cart().robot;
  const second = cart().robot;
  world.scene.add(first, second);
  expect(() => ros.sync()).toThrow("share the namespace /robot");
  expect(ros.find(Robot)).toEqual([first]);
});

it("releases a robot even when releasing its command subscription fails", async () => {
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
  const { robot, actuator } = cart();
  world.scene.add(robot);
  ros.sync();
  connection.receive(COMMANDS, jointCommand("velocity", 2));
  world.update(0.01);
  expect(actuator.target?.velocity).toBe(2);
  robot.removeFromParent();
  expect(() => ros.sync()).toThrow(
    expect.objectContaining({ errors: [failure] }),
  );
  expect(ros.find(Robot)).toEqual([]);
  expect(actuator.target).toBeUndefined();
  expect(() => ros.dispose()).not.toThrow();
});

it("puts a namespaced robot's topics under its namespace and prefixes its frames", async () => {
  const { world, ros, connection } = await simulation();
  const arm = new Robot({ namespace: "/arm1" });
  arm.name = "arm";
  const base = new RigidBody({ bodyType: "static" });
  base.name = "base_link";
  const camera = new CameraSensor({ frameId: "camera_optical" });
  base.add(camera);
  arm.add(base);
  expect(arm.descriptionTopic).toBe("/arm1/robot_description");
  expect(camera.compressedTopic).toBe("/arm1/camera/image_raw/compressed");
  world.scene.add(arm);
  ros.sync();
  expect([...connection.receivers.keys()]).toEqual(["/arm1/joint_commands"]);
  world.update(0.01);
  const statics = connection.messages.find(({ name }) => name === "/tf_static");
  expect(statics?.message).toMatchObject({
    transforms: [
      {
        header: { frame_id: "arm1/base_footprint" },
        child_frame_id: "arm1/base_link",
      },
      {
        header: { frame_id: "arm1/base_link" },
        child_frame_id: "arm1/camera_optical",
      },
    ],
  });
});
