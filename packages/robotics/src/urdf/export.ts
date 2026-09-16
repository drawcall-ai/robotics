import { Camera, Matrix4, Object3D, Quaternion, Vector3 } from "three";
import { PrismaticJoint, RigidBody, type Shape } from "@drawcall/physics";
import {
  rollPitchYaw,
  rosOpticalRotation,
  rosPosition,
  rosRotation,
} from "../coordinates.js";
import { findRobot, type Robot } from "../devices/robot.js";
import { getROS2, type ROS2 } from "../runtime.js";
import {
  inLinkFrame,
  lowestPoint,
  resolveLinks,
  type Link,
  type LinkJoint,
} from "./links.js";

const FOOTPRINT = "base_footprint";
const COLOR = "0.603922 0.647059 0.670588 1";

/**
 * Serializes the runtime's robot, or the one named `name`, as URDF in ROS coordinates: bodies become
 * links, axis joints keep their axes and limits, sensors become fixed frames with a three.js `Camera` in
 * the ROS optical convention, and motors produce the topic-based ros2_control block.
 */
export function exportURDF({
  ros = getROS2(),
  name,
}: { ros?: ROS2; name?: string } = {}): string {
  const robot = findRobot(ros, name);
  robot.updateWorldMatrix(true, true);
  const { base, links, joints } = resolveLinks(robot.base, robot.joints);
  const frames = [...robot.frames];
  unique("link", [
    ...[...links.values()].map((link) => link.name),
    FOOTPRINT,
    ...frames.map(([name]) => name),
  ]);
  unique("joint", [
    ...joints.map((joint) => joint.name),
    `${FOOTPRINT}_joint`,
    ...frames.map(([name]) => `${name}_joint`),
  ]);
  const material = `${robot.name}_material`;
  return [
    `<?xml version="1.0"?>`,
    `<robot name="${robot.name}">`,
    `  <material name="${material}">`,
    `    <color rgba="${COLOR}"/>`,
    `  </material>`,
    ...footprintXML(base, [...links.values()]),
    ...[...links.values()].flatMap((link) => linkXML(link, material)),
    ...joints.flatMap(jointXML),
    ...frames.flatMap(([name, object]) =>
      frameXML(name, object, links, robot.name),
    ),
    ...(robot.drives.size ? controlXML(robot, joints) : []),
    "</robot>",
    "",
  ].join("\n");
}

function unique(kind: string, names: string[]): void {
  const seen = new Set<string>();
  for (const name of names) {
    if (seen.has(name)) throw new Error(`Duplicate URDF ${kind} name: ${name}`);
    seen.add(name);
  }
}

function linkXML(link: Link, material: string): string[] {
  return [
    `  <link name="${link.name}">`,
    ...link.colliders.flatMap(({ shape, matrix, where }) => {
      const geometry = geometryXML(shape, where);
      const origin = originXML("      ", matrix);
      return [
        `    <visual>`,
        origin,
        `      <geometry>${geometry}</geometry>`,
        `      <material name="${material}"/>`,
        `    </visual>`,
        `    <collision>`,
        origin,
        `      <geometry>${geometry}</geometry>`,
        `    </collision>`,
      ];
    }),
    `  </link>`,
  ];
}

function jointXML({ name, joint, parent, child, origin }: LinkJoint): string[] {
  const limits = joint.limits;
  const prismatic = joint instanceof PrismaticJoint;
  if (prismatic && !limits)
    throw new Error(`Prismatic joint ${name} needs limits for URDF`);
  const type = prismatic ? "prismatic" : limits ? "revolute" : "continuous";
  const axis = rosPosition(axisVector(joint.options.axis ?? "Y"));
  return [
    `  <joint name="${name}" type="${type}">`,
    `    <parent link="${parent.name}"/>`,
    `    <child link="${child.name}"/>`,
    originXML("    ", origin),
    `    <axis xyz="${vec(axis.x, axis.y, axis.z)}"/>`,
    ...(limits
      ? [
          `    <limit lower="${num(limits[0])}" upper="${num(limits[1])}" effort="0" velocity="0"/>`,
        ]
      : []),
    `  </joint>`,
  ];
}

function frameXML(
  name: string,
  object: Object3D,
  links: Map<RigidBody, Link>,
  robotName: string,
): string[] {
  let ancestor = object.parent;
  while (ancestor && !(ancestor instanceof RigidBody))
    ancestor = ancestor.parent;
  if (!ancestor)
    throw new Error(`Frame ${name} does not descend from a robot body`);
  const link = links.get(ancestor);
  if (!link)
    throw new Error(
      `Frame ${name} is on ${ancestor.name}, which is not a link of ${robotName}`,
    );
  const bodyLocal = link.body.matrixWorld
    .clone()
    .invert()
    .multiply(object.matrixWorld);
  return [
    `  <link name="${name}"/>`,
    `  <joint name="${name}_joint" type="fixed">`,
    `    <parent link="${link.name}"/>`,
    `    <child link="${name}"/>`,
    originXML(
      "    ",
      inLinkFrame(link.origin, bodyLocal),
      object instanceof Camera,
    ),
    `  </joint>`,
  ];
}

function footprintXML(base: Link, links: Link[]): string[] {
  const lowest = Math.min(
    ...links.flatMap((link) =>
      link.colliders.map(({ shape, matrix, where }) =>
        lowestPoint(shape, link.restPose.clone().multiply(matrix), where),
      ),
    ),
  );
  if (!Number.isFinite(lowest))
    throw new Error("A footprint needs at least one collider");
  return [
    `  <link name="${FOOTPRINT}"/>`,
    `  <joint name="${FOOTPRINT}_joint" type="fixed">`,
    `    <parent link="${FOOTPRINT}"/>`,
    `    <child link="${base.name}"/>`,
    `    <origin xyz="${vec(0, 0, -lowest)}" rpy="0 0 0"/>`,
    `  </joint>`,
  ];
}

function controlXML(robot: Robot, joints: LinkJoint[]): string[] {
  return [
    `  <ros2_control name="${robot.name}" type="system">`,
    `    <hardware>`,
    `      <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>`,
    `      <param name="joint_commands_topic">${robot.ros.topic("joint_commands")}</param>`,
    `      <param name="joint_states_topic">${robot.ros.topic("joint_states")}</param>`,
    `      <param name="trigger_joint_command_threshold">-1</param>`,
    `    </hardware>`,
    ...joints.flatMap(({ name }) => {
      const command = robot.drives.get(name)?.command;
      return [
        `    <joint name="${name}">`,
        ...(command ? [`      <command_interface name="${command}"/>`] : []),
        `      <state_interface name="position"/>`,
        `      <state_interface name="velocity"/>`,
        `    </joint>`,
      ];
    }),
    `  </ros2_control>`,
  ];
}

function originXML(
  indent: string,
  transform: Matrix4,
  optical = false,
): string {
  const position = new Vector3();
  const rotation = new Quaternion();
  const scale = new Vector3();
  transform.decompose(position, rotation, scale);
  if ([scale.x, scale.y, scale.z].some((value) => Math.abs(value - 1) > 1e-6))
    throw new Error("URDF origins must not be scaled");
  const p = rosPosition(position);
  const [roll, pitch, yaw] = rollPitchYaw(
    optical ? rosOpticalRotation(rotation) : rosRotation(rotation),
  );
  return `${indent}<origin xyz="${vec(p.x, p.y, p.z)}" rpy="${vec(roll, pitch, yaw)}"/>`;
}

function geometryXML(shape: Shape, where: string): string {
  switch (shape.kind) {
    case "box": {
      const size = rosPosition(new Vector3(...shape.size));
      return `<box size="${vec(Math.abs(size.x), Math.abs(size.y), Math.abs(size.z))}"/>`;
    }
    case "sphere":
      return `<sphere radius="${num(shape.radius)}"/>`;
    case "cylinder":
      return `<cylinder radius="${num(shape.radius)}" length="${num(shape.height)}"/>`;
    default:
      throw new Error(
        `URDF export supports box, sphere and cylinder colliders; ${where} is a ${shape.kind}`,
      );
  }
}

function axisVector(axis: "X" | "Y" | "Z"): Vector3 {
  return new Vector3(
    axis === "X" ? 1 : 0,
    axis === "Y" ? 1 : 0,
    axis === "Z" ? 1 : 0,
  );
}

function num(value: number): string {
  const rounded = Number(value.toFixed(6));
  return (Object.is(rounded, -0) ? 0 : rounded).toString();
}
function vec(x: number, y: number, z: number): string {
  return `${num(x)} ${num(y)} ${num(z)}`;
}
