import { afterEach, expect, it, vi } from "vitest";
import { Object3D, Vector3 } from "three";
import { RigidBody, RevoluteJoint } from "@drawcall/physics";
import { Robot } from "../src/robot.js";
import { setupWorld } from "@drawcall/physics-rapier";
import { ROS2 } from "../src/runtime.js";
import { LaserSensor } from "../src/sensors/laser.js";
import { ConnectionStub } from "./connection.js";
import { RendererStub } from "./renderer.js";
const cleanup: (() => void)[] = [];
afterEach(() => {
  for (const release of cleanup.splice(0).reverse()) release();
  vi.restoreAllMocks();
});
async function fixture() {
  const world = await setupWorld({ gravity: [0, 0, 0], fixedDelta: 0.01 });
  cleanup.push(() => world.dispose());
  const connection = new ConnectionStub();
  const { messages } = connection;
  const ros = new ROS2(connection, { world, renderer: new RendererStub() });
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
  const robot = new Robot({ ros });
  robot.name = "cart";
  robot.add(base, body, joint);
  return { ros, world, messages, joint, robot };
}

it("publishes measured encoder feedback without a robot pose", async () => {
  const { world, messages, joint } = await fixture();
  world.update(0.01);
  const stamp = { sec: 0, nanosec: 10_000_000 };
  const state = joint.getState();
  expect(messages).toContainEqual({
    name: "/robot/joint_states",
    type: "sensor_msgs/msg/JointState",
    message: {
      header: { stamp, frame_id: "" },
      name: ["wheel"],
      position: [state.position],
      velocity: [state.velocity],
      effort: [],
    },
  });
});

it("uses world geometry for rays without publishing world pose", async () => {
  const { ros, world, messages } = await fixture();
  const laser = new LaserSensor({
    ros,
    samples: 3,
    range: 7,
    minRange: 0.1,
    frameId: "front_laser",
  });
  laser.position.set(2, 3, 4);
  laser.rotation.y = Math.PI / 2;
  const raycast = vi
    .spyOn(world, "raycast")
    .mockReturnValue(null)
    .mockReturnValueOnce({
      distance: 0.05,
      point: new Vector3(),
      normal: new Vector3(),
      body: new RigidBody({ world, type: "static" }),
      collider: new Object3D(),
    });
  world.update(0.01);
  expect(raycast).toHaveBeenCalledTimes(3);
  const directions = raycast.mock.calls.map((call) => call[1]);
  expect(directions[1]?.x).toBeCloseTo(-1);
  expect(directions[1]?.z).toBeCloseTo(0);
  expect(messages).toContainEqual({
    name: "/robot/scan",
    type: "sensor_msgs/msg/LaserScan",
    message: expect.objectContaining({
      header: {
        stamp: { sec: 0, nanosec: 10_000_000 },
        frame_id: "front_laser",
      },
      ranges: [8, 8, 8], // the first hit lies inside the dead zone
      angle_increment: Math.PI,
    }),
  });
  expect(messages.map(({ name }) => name)).toEqual([
    "/clock",
    "/robot/joint_states",
    "/robot/scan",
  ]);
});
