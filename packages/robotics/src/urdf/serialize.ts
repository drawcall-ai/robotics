import { CapsuleGeometry, Matrix4, Vector3 } from "three";
import { FixedJoint, PrismaticJoint, type Shape } from "@drawcall/physics";
import { rollPitchYaw, rosPose, rosPosition } from "../coordinates.js";
import { modelView } from "./view.js";
import { meshXML, visualsXML } from "./meshes.js";
import type { JointActuator } from "../devices/actuator.js";
import type { Robot } from "../devices/robot.js";
import type { Link, LinkFrame, LinkJoint } from "./links.js";

const FOOTPRINT = "base_footprint";
const COLOR = "0.603922 0.647059 0.670588 1";

/**
 * Serializes the runtime's robot, or the one named `name`, as URDF in ROS coordinates: bodies become
 * links, axis joints keep their axes and limits, sensors become fixed frames with a three.js `Camera` in
 * the ROS optical convention, and motors produce the topic-based ros2_control block.
 */
export function robotURDF(robot: Robot): string {
  robot.updateWorldMatrix(true, true);
  const { base, links, joints, frames, footprint } = robot.tree;
  unique("link", [
    ...[...links.values()].map((link) => link.name),
    FOOTPRINT,
    ...frames.map(({ name }) => name),
  ]);
  unique("joint", [
    ...joints.map((joint) => joint.name),
    `${FOOTPRINT}_joint`,
    ...frames.map(({ name }) => `${name}_joint`),
  ]);
  const material = `${robot.name}_material`;
  return [
    `<?xml version="1.0"?>`,
    `<robot name="${robot.name}">`,
    `  <!-- drawcall-view: ${JSON.stringify(modelView(robot))} -->`,
    `  <material name="${material}">`,
    `    <color rgba="${COLOR}"/>`,
    `  </material>`,
    ...footprintXML(base, footprint),
    ...[...links.values()].flatMap((link) => linkXML(link, material)),
    ...joints.flatMap((joint) =>
      jointXML(joint, robot.actuators.get(joint.name)),
    ),
    ...frames.flatMap(frameXML),
    ...(robot.actuators.size ? controlXML(robot, joints) : []),
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
  const visuals = visualsXML(link);
  return [
    `  <link name="${link.name}">`,
    ...visuals,
    ...link.colliders.flatMap(({ shape, matrix }) => {
      const geometry = geometryXML(shape);
      const origin = originXML("      ", matrix);
      return [
        ...(visuals.length
          ? []
          : [
              `    <visual>`,
              origin,
              `      <geometry>${geometry}</geometry>`,
              `      <material name="${material}"/>`,
              `    </visual>`,
            ]),
        `    <collision>`,
        origin,
        `      <geometry>${geometry}</geometry>`,
        `    </collision>`,
      ];
    }),
    `  </link>`,
  ];
}

function jointXML(
  { name, joint, parent, child, origin }: LinkJoint,
  actuator: JointActuator | undefined,
): string[] {
  if (joint instanceof FixedJoint)
    return [
      `  <joint name="${name}" type="fixed">`,
      `    <parent link="${parent.name}"/>`,
      `    <child link="${child.name}"/>`,
      originXML("    ", origin),
      `  </joint>`,
    ];
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
          `    <limit lower="${num(limits[0])}" upper="${num(limits[1])}" effort="${num(actuator?.options.maxForce ?? 0)}" velocity="${num(actuator?.options.maxVelocity ?? 0)}"/>`,
        ]
      : []),
    `  </joint>`,
  ];
}

function frameXML({ name, link, origin, optical }: LinkFrame): string[] {
  return [
    `  <link name="${name}"/>`,
    `  <joint name="${name}_joint" type="fixed">`,
    `    <parent link="${link.name}"/>`,
    `    <child link="${name}"/>`,
    originXML("    ", origin, optical),
    `  </joint>`,
  ];
}

function footprintXML(base: Link, footprint: Matrix4): string[] {
  return [
    `  <link name="${FOOTPRINT}"/>`,
    `  <joint name="${FOOTPRINT}_joint" type="fixed">`,
    `    <parent link="${FOOTPRINT}"/>`,
    `    <child link="${base.name}"/>`,
    originXML("    ", footprint),
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
    ...joints
      .filter(({ joint }) => !(joint instanceof FixedJoint))
      .flatMap(({ name }) => {
        const command = robot.actuators.get(name)?.options.interface;
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
  const { position: p, rotation } = rosPose(transform, optical);
  const [roll, pitch, yaw] = rollPitchYaw(rotation);
  return `${indent}<origin xyz="${vec(p.x, p.y, p.z)}" rpy="${vec(roll, pitch, yaw)}"/>`;
}

function geometryXML(shape: Shape): string {
  switch (shape.kind) {
    case "box": {
      const size = rosPosition(new Vector3(...shape.size));
      return `<box size="${vec(Math.abs(size.x), Math.abs(size.y), Math.abs(size.z))}"/>`;
    }
    case "sphere":
      return `<sphere radius="${num(shape.radius)}"/>`;
    case "cylinder":
      return `<cylinder radius="${num(shape.radius)}" length="${num(shape.height)}"/>`;
    case "capsule": {
      const geometry = new CapsuleGeometry(shape.radius, shape.height);
      try {
        return meshXML(geometry);
      } finally {
        geometry.dispose();
      }
    }
    case "mesh":
      return meshXML(
        shape.geometry,
        new Matrix4(),
        shape.approximation === "convexHull",
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
