import { expect, it } from "vitest";
import { BoxGeometry, Matrix4 } from "three";
import {
  BoxCollider,
  CylinderCollider,
  MeshCollider,
  RevoluteJoint,
  RigidBody,
  SphericalJoint,
} from "@drawcall/physics";
import { JointActuator } from "../src/devices/actuator.js";
import { CameraSensor } from "../src/devices/camera.js";
import { LaserSensor } from "../src/devices/laser.js";
import { Robot } from "../src/devices/robot.js";
import { exportURDF } from "../src/urdf/export.js";
import { mkdtemp, readFile, rm } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { separateMeshes } from "../src/cli/urdf.js";
import { defer } from "./fixture.js";

/** A rover standing somewhere: box chassis, one actuated wheel on a cylinder, a camera and a lidar. */
function rover(
  name = "rover",
  {
    at = [5, 1, -2],
    stiffness,
    limits,
  }: {
    at?: number[];
    stiffness?: number;
    limits?: [number, number];
  } = {},
) {
  const [x = 0, y = 0, z = 0] = at;
  const robot = new Robot();
  robot.name = name;
  const base = new RigidBody({ mass: 2 });
  base.name = "base_link";
  base.position.set(x, y, z);
  base.add(new BoxCollider({ size: [0.4, 0.2, 0.6] }));
  const wheel = new RigidBody({ mass: 1 });
  wheel.name = "wheel";
  wheel.position.set(x + 0.3, y - 0.05, z);
  const tire = new CylinderCollider({ radius: 0.1, height: 0.05 });
  tire.rotation.z = Math.PI / 2;
  wheel.add(tire);
  const axle = new RevoluteJoint({
    body0: base,
    body1: wheel,
    axis: "X",
    limits,
    frame0: new Matrix4().setPosition(0.3, -0.05, 0),
    frame1: new Matrix4(),
  });
  axle.name = "axle";
  axle.setDrive(
    new JointActuator({
      interface: stiffness ? "position" : "velocity",
      damping: 1,
      maxForce: 1,
      maxVelocity: 3,
      stiffness,
    }),
  );
  const camera = new CameraSensor();
  camera.position.set(0, 0.1, -0.2);
  const laser = new LaserSensor();
  laser.position.set(0, 0.2, 0);
  laser.rotation.y = Math.PI / 2;
  base.add(camera, laser);
  robot.add(base, wheel, axle);
  return { robot, base, wheel, axle };
}
/** A free-rolling support ball: URDF has no ball joint to describe it. */
function withCaster({ robot, base }: { robot: Robot; base: RigidBody }) {
  const caster = new RigidBody({ mass: 0.1 });
  caster.name = "caster";
  robot.add(caster, new SphericalJoint({ body0: base, body1: caster }));
  return caster;
}

it("reads links, joints, colliders, frames and control from its children in ROS coordinates", () => {
  const { robot, base } = rover();
  const urdf = exportURDF(robot);
  expect(urdf).toContain('<robot name="rover">');
  expect(urdf).toContain('<box size="0.6 0.4 0.2"/>');
  expect(urdf).toContain('<cylinder radius="0.1" length="0.05"/>');
  expect(urdf).toContain('<origin xyz="0 0 0" rpy="-1.570796 0 0"/>');
  expect(urdf).toContain('<joint name="axle" type="continuous">');
  expect(urdf).toContain('<origin xyz="0 -0.3 -0.05" rpy="0 0 0"/>');
  expect(urdf).toContain('<axis xyz="0 -1 0"/>');
  expect(urdf).toContain(
    '<origin xyz="0.2 0 0.1" rpy="-1.570796 0 -1.570796"/>',
  );
  expect(urdf).toContain('<origin xyz="0 0 0.2" rpy="0 0 1.570796"/>');
  expect(urdf).toContain('<origin xyz="0 0 0.15" rpy="0 0 0"/>');
  expect(urdf).toContain(
    '<param name="joint_commands_topic">/robot/joint_commands</param>',
  );
  expect(urdf).toContain('<command_interface name="velocity"/>');
  expect(urdf).not.toContain("caster");
  expect(robot.base).toBe(base);
});

it("is deterministic and independent of where the robot stands", () => {
  const urdf = exportURDF(rover().robot);
  expect(exportURDF(rover("rover", { at: [-3, 2, 8] }).robot)).toBe(urdf);
});

it("commands a stiff actuator through the position interface", () => {
  const { robot } = rover("stiff", { stiffness: 50 });
  expect(exportURDF(robot)).toContain('<command_interface name="position"/>');
});

it("writes the actuator's force and velocity limits on a limited joint", () => {
  const { robot } = rover("limited", { stiffness: 50, limits: [-1, 1] });
  expect(exportURDF(robot)).toContain(
    '<limit lower="-1" upper="1" effort="1" velocity="3"/>',
  );
});

it("has no control block without actuators", () => {
  const free = new Robot();
  free.name = "free";
  const base = new RigidBody({ mass: 1 });
  base.name = "base_link";
  base.add(new BoxCollider({ size: [1, 1, 1] }));
  const wheel = new RigidBody({ mass: 1 });
  wheel.name = "wheel";
  const axle = new RevoluteJoint({ body0: base, body1: wheel });
  axle.name = "axle";
  free.add(base, wheel, axle);
  expect(exportURDF(free)).not.toContain("ros2_control");
});

it("rejects a robot without a name, an ambiguous base and an unnamed joint", () => {
  expect(() => exportURDF(new Robot())).toThrow("Invalid robot name");
  const two = rover("two");
  two.robot.add(new RigidBody({ mass: 1 }));
  expect(() => exportURDF(two.robot)).toThrow("found 2");
  const unnamed = rover("unnamed");
  unnamed.axle.name = "";
  expect(() => exportURDF(unnamed.robot)).toThrow("Invalid joint name");
});

it("rejects disconnected joints and duplicate frames, and exports mesh colliders", () => {
  const orphaned = rover("orphaned");
  const loose = new RigidBody({ mass: 1 });
  loose.name = "loose";
  const other = new RigidBody({ mass: 1 });
  other.name = "other";
  const orphan = new RevoluteJoint({ body0: loose, body1: other });
  orphan.name = "orphan";
  orphaned.robot.add(loose, other, orphan);
  expect(() => exportURDF(orphaned.robot)).toThrow("found 2");
  const duplicate = rover("duplicate");
  duplicate.base.add(new LaserSensor({ name: "rear", frameId: "wheel" }));
  expect(() => exportURDF(duplicate.robot)).toThrow(
    "Duplicate URDF link name: wheel",
  );
  const meshed = rover("meshed");
  const mesh = new MeshCollider();
  mesh.setGeometry(new BoxGeometry(1, 1, 1));
  meshed.base.add(mesh);
  expect(exportURDF(meshed.robot)).toContain("data:model/stl;base64,");
});

it("rejects two joints or two frames with one name, and a frame off a link", () => {
  const twin = rover("twin");
  const extra = new RigidBody({ mass: 1 });
  extra.name = "extra";
  const axle = new RevoluteJoint({ body0: twin.base, body1: extra });
  axle.name = "axle";
  twin.robot.add(extra, axle);
  expect(() => exportURDF(twin.robot)).toThrow("two joints named axle");
  const eyes = rover("eyes");
  eyes.base.add(new CameraSensor({ name: "rear", frameId: "camera_optical" }));
  expect(() => exportURDF(eyes.robot)).toThrow(
    "two frames named camera_optical",
  );
  const rolling = rover("rolling");
  withCaster(rolling);
  expect(() => exportURDF(rolling.robot)).toThrow(
    "Body caster of rolling hangs from no named fixed or axis joint",
  );
});

it("separates embedded meshes into shared STL files beside the output", async () => {
  const { robot, base } = rover();
  for (const x of [0, 1]) {
    const mesh = new MeshCollider();
    mesh.setGeometry(new BoxGeometry(1, 1, 1));
    mesh.position.x = x;
    base.add(mesh);
  }
  const dir = await mkdtemp(join(tmpdir(), "urdf-"));
  defer(() => void rm(dir, { recursive: true }));
  const output = join(dir, "rover.urdf");
  const urdf = await separateMeshes(exportURDF(robot), output);
  expect(urdf).not.toContain("data:");
  const files = [...urdf.matchAll(/filename="(file:[^"]+)"/g)].map(
    ([, url]) => url ?? "",
  );
  // Two colliders, each drawn as a visual too, all one shape.
  expect(files).toHaveLength(4);
  expect(new Set(files).size).toBe(1);
  const [file = ""] = files;
  expect(file).toMatch(/\/rover-[0-9a-f]{12}\.stl$/);
  expect((await readFile(new URL(file))).byteLength).toBeGreaterThan(84);
});
