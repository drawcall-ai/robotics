import { expect, it, vi } from "vitest";
import {
  BoxGeometry,
  Mesh,
  MeshStandardMaterial,
  Matrix4,
  Vector3,
} from "three";
import { BoxCollider, FixedJoint, RigidBody } from "@drawcall/physics";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { Robot } from "../src/devices/robot.js";
import { exportURDF } from "../src/urdf/export.js";
import { descriptionView } from "../src/urdf/view.js";
import { rosPosition } from "../src/coordinates.js";
import { cart, simulation } from "./fixture.js";

it("publishes a retained model and static frames once before motion, then live link transforms", async () => {
  const { world, connection } = await simulation();
  const { robot, base, wheel } = cart();
  world.scene.add(robot);
  base.add(new BoxCollider());
  const publish = vi.spyOn(connection, "publish");
  expect(connection.messages).toHaveLength(0);
  world.update(0.01);
  expect(publish).toHaveBeenCalledWith(
    "/robot/robot_description",
    "std_msgs/msg/String",
    { data: expect.stringContaining('<robot name="cart">') },
    { retain: true, private: false },
  );
  expect(publish).toHaveBeenCalledWith(
    "/tf_static",
    "tf2_msgs/msg/TFMessage",
    {
      transforms: expect.arrayContaining([
        expect.objectContaining({
          child_frame_id: "base_link",
          header: expect.objectContaining({ frame_id: "base_footprint" }),
        }),
      ]),
    },
    { retain: true, private: false },
  );
  const point = rosPosition(wheel.position.clone().sub(base.position));
  expect(connection.messages).toContainEqual({
    name: "/tf",
    type: "tf2_msgs/msg/TFMessage",
    message: {
      transforms: [
        expect.objectContaining({
          child_frame_id: "wheel_link",
          transform: expect.objectContaining({
            translation: { x: point.x, y: point.y, z: point.z },
          }),
        }),
      ],
    },
  });
  for (let i = 0; i < 20; i++) world.update(0.01);
  expect(
    connection.messages.filter(
      ({ name }) => name === "/robot/robot_description",
    ),
  ).toHaveLength(1);
  expect(
    connection.messages.filter(({ name }) => name === "/tf").length,
  ).toBeGreaterThan(1);
});

it("embeds actual colored visual geometry in ROS link coordinates, separately from collision proxies", () => {
  const { robot, base } = cart();
  base.add(new BoxCollider({ size: [0.2, 0.2, 0.2] }));
  const visual = new Mesh(
    new BoxGeometry(2, 4, 6),
    new MeshStandardMaterial({ color: "red" }),
  );
  visual.position.set(1, 2, 3);
  base.add(visual);
  const xml = exportURDF(robot);
  const encoded = /data:model\/stl;base64,([^"]+)/.exec(xml)?.[1];
  if (!encoded) throw new Error("No embedded visual");
  const data = Uint8Array.from(atob(encoded), (c) => c.charCodeAt(0));
  const geometry = new STLLoader().parse(data.buffer);
  geometry.computeBoundingBox();
  expect(geometry.boundingBox?.getCenter(new Vector3()).toArray()).toEqual([
    -3, -1, 2,
  ]);
  expect(geometry.boundingBox?.getSize(new Vector3()).toArray()).toEqual([
    6, 2, 4,
  ]);
  expect(xml).toMatch(/rgba="0\.9+ 0 0 1"/);
  expect(xml).toContain(
    '<collision>\n      <origin xyz="0 0 0" rpy="0 0 0"/>\n      <geometry><box size="0.2 0.2 0.2"/>',
  );
  expect(descriptionView(xml)).toMatchObject({
    topic: "/robot/robot_description",
    frame: "base_footprint",
  });
  geometry.dispose();
});

it("includes fixed bodies and publishes a model even without actuated joints", async () => {
  const { world, connection } = await simulation();
  const robot = new Robot();
  robot.name = "fixed";
  const base = new RigidBody({ bodyType: "static" });
  base.name = "base_link";
  base.add(new BoxCollider());
  const tool = new RigidBody({ mass: 1 });
  tool.name = "tool";
  tool.position.y = 1;
  tool.add(new BoxCollider());
  const joint = new FixedJoint({
    body0: base,
    body1: tool,
    frame0: new Matrix4().setPosition(0, 1, 0),
    frame1: new Matrix4(),
  });
  joint.name = "mount";
  robot.add(base, tool, joint);
  world.scene.add(robot);
  world.update(0.01);
  expect(connection.messages).toContainEqual({
    name: "/tf_static",
    type: "tf2_msgs/msg/TFMessage",
    message: {
      transforms: expect.arrayContaining([
        expect.objectContaining({
          child_frame_id: "tool",
          transform: expect.objectContaining({
            translation: { x: -0, y: -0, z: 1 },
          }),
        }),
      ]),
    },
  });
  expect(exportURDF(world.scene)).toContain(
    '<joint name="mount" type="fixed">',
  );
});
