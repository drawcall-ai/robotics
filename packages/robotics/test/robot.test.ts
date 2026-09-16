import { afterEach, expect, it } from "vitest";
import { BoxGeometry, Matrix4 } from "three";
import {
  AuthoringWorld,
  BoxCollider,
  CylinderCollider,
  JointMotor,
  MeshCollider,
  RevoluteJoint,
  RigidBody,
  SphericalJoint,
  setDefaultWorld,
} from "@drawcall/physics";
import { OfflineConnection } from "../src/offline.js";
import { Robot, findRobot } from "../src/robot.js";
import { ROS2 } from "../src/runtime.js";
import { CameraSensor } from "../src/sensors/camera.js";
import { LaserSensor } from "../src/sensors/laser.js";

const cleanup: (() => void)[] = [];
afterEach(() => {
  for (const release of cleanup.splice(0).reverse()) release();
});

function runtime() {
  setDefaultWorld(new AuthoringWorld());
  const ros = new ROS2(new OfflineConnection());
  cleanup.push(() => ros.dispose());
  return ros;
}

/** A rover standing somewhere: box chassis, one motorized wheel on a cylinder, a camera and a lidar. */
function rover(name = "rover", at: [number, number, number] = [5, 1, -2]) {
  const robot = new Robot();
  robot.name = name;
  const base = new RigidBody({ mass: 2 });
  base.name = "base_link";
  base.position.set(...at);
  base.add(new BoxCollider({ size: [0.4, 0.2, 0.6] }));
  const wheel = new RigidBody({ mass: 1 });
  wheel.name = "wheel";
  wheel.position.set(at[0] + 0.3, at[1] - 0.05, at[2]);
  const tire = new CylinderCollider({ radius: 0.1, height: 0.05 });
  tire.rotation.z = Math.PI / 2;
  wheel.add(tire);
  const axle = new RevoluteJoint({
    body0: base,
    body1: wheel,
    axis: "X",
    frame0: new Matrix4().setPosition(0.3, -0.05, 0),
    frame1: new Matrix4(),
  });
  axle.name = "axle";
  new JointMotor({ joint: axle, damping: 1, maxForce: 1 });
  const camera = new CameraSensor();
  camera.position.set(0, 0.1, -0.2);
  const laser = new LaserSensor();
  laser.position.set(0, 0.2, 0);
  laser.rotation.y = Math.PI / 2;
  base.add(camera, laser);
  robot.add(base, wheel, axle);
  return { robot, base, wheel, axle };
}

it("reads links, joints, colliders, frames and control from its children in ROS coordinates", () => {
  runtime();
  const { robot, base } = rover();
  const caster = new RigidBody({ mass: 0.1 });
  robot.add(caster, new SphericalJoint({ body0: base, body1: caster }));
  const urdf = robot.urdf();
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
  expect(urdf).not.toContain("5 ");
  expect(robot.base).toBe(base);
});

it("is deterministic and independent of where the robot stands", () => {
  runtime();
  expect(rover().robot.urdf()).toBe(rover("rover", [-3, 2, 8]).robot.urdf());
});

it("has no control block without motors and rejects missing names and ambiguous bases", () => {
  runtime();
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
  expect(free.urdf()).not.toContain("ros2_control");
  expect(() => new Robot().urdf()).toThrow("needs a name");
  const two = rover("two");
  two.robot.add(new RigidBody({ mass: 1 }));
  expect(() => two.robot.urdf()).toThrow("found 2");
  const unnamed = rover("unnamed");
  unnamed.axle.name = "";
  expect(() => unnamed.robot.urdf()).toThrow("needs a name");
});

it("rejects disconnected joints, mesh colliders and duplicate names", () => {
  runtime();
  const orphaned = rover("orphaned");
  const loose = new RigidBody({ mass: 1 });
  loose.name = "loose";
  const other = new RigidBody({ mass: 1 });
  other.name = "other";
  const orphan = new RevoluteJoint({ body0: loose, body1: other });
  orphan.name = "orphan";
  orphaned.robot.add(loose, other, orphan);
  expect(() => orphaned.robot.urdf()).toThrow("found 2");
  const duplicate = rover("duplicate");
  duplicate.base.add(new LaserSensor({ name: "rear", frameId: "wheel" }));
  expect(() => duplicate.robot.urdf()).toThrow(
    "Duplicate URDF link name: wheel",
  );
  const meshed = rover("meshed");
  const mesh = new MeshCollider();
  mesh.setGeometry(new BoxGeometry(1, 1, 1));
  meshed.base.add(mesh);
  expect(() => meshed.robot.urdf()).toThrow("mesh");
});

it("rejects two joints or two frames with one name", () => {
  runtime();
  const twin = rover("twin");
  const extra = new RigidBody({ mass: 1 });
  extra.name = "extra";
  const axle = new RevoluteJoint({ body0: twin.base, body1: extra });
  axle.name = "axle";
  twin.robot.add(extra, axle);
  expect(() => twin.robot.urdf()).toThrow("two joints named axle");
  const eyes = rover("eyes");
  eyes.base.add(new CameraSensor({ name: "rear", frameId: "camera_optical" }));
  expect(() => eyes.robot.urdf()).toThrow("two frames named camera_optical");
});

it("finds a robot by name when a scene has several", () => {
  const ros = runtime();
  expect(() => findRobot(ros)).toThrow("has 0 robots");
  const { robot } = rover("one");
  expect(findRobot(ros)).toBe(robot);
  rover("two");
  expect(() => findRobot(ros)).toThrow("pick one with --robot");
  expect(findRobot(ros, "two").name).toBe("two");
  expect(() => findRobot(ros, "three")).toThrow("no robot named three");
});
