import { JointStateSensor } from "../src/sensors/joint-state.js";
import { afterEach, expect, it, vi } from "vitest";
import { Scene, WebGLRenderer } from "three";
import { RigidBody, RevoluteJoint } from "@drawcall/physics";
import { setupWorld } from "@drawcall/physics-rapier";
import { ROS2 } from "../src/runtime.js";
import { LaserSensor } from "../src/sensors/laser.js";

vi.mock("three", async (importOriginal) => {
  const three = await importOriginal<typeof import("three")>();
  return { ...three, WebGLRenderer: class {} };
});
const cleanup: (() => void)[] = [];
afterEach(() => {
  for (const release of cleanup.splice(0).reverse()) release();
  vi.restoreAllMocks();
});
async function fixture() {
  const world = await setupWorld({ gravity: [0, 0, 0], fixedDelta: 0.01 });
  cleanup.push(() => world.dispose());
  const messages: { name: string; type: string; message: unknown }[] = [];
  const ros = new ROS2(
    {
      connected: true,
      dropped: 0,
      publish(name, type, message) {
        messages.push({ name, type, message });
      },
      subscribe() {
        return () => {};
      },
      dispose() {},
    },
    { world, scene: new Scene(), renderer: new WebGLRenderer() },
  );
  cleanup.push(() => ros.dispose());
  const body = new RigidBody({
    mass: 1,
    centerOfMass: [0, 0, 0],
    diagonalInertia: [1, 1, 1],
  });
  const joint = new RevoluteJoint({ body0: null, body1: body, axis: "Z" });
  return { ros, world, messages, joint };
}

it("publishes measured encoder feedback without a robot pose", async () => {
  const { ros, world, messages, joint } = await fixture();
  new JointStateSensor({ ros, joints: { wheel: joint } });
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
    frameId: "front_laser",
  });
  laser.position.set(2, 3, 4);
  laser.rotation.y = Math.PI / 2;
  const raycast = vi.spyOn(world, "raycast").mockReturnValue(null);
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
      ranges: [8, 8, 8],
      angle_increment: Math.PI,
    }),
  });
  expect(messages.map(({ name }) => name)).toEqual(["/clock", "/robot/scan"]);
});
