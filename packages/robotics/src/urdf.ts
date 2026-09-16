import { Matrix4, Object3D, Quaternion, Vector3 } from "three";
import {
  PrismaticJoint,
  RigidBody,
  resolveCollider,
  type AxisJoint,
  type Shape,
} from "@drawcall/physics";
import {
  rollPitchYaw,
  rosOpticalRotation,
  rosPosition,
  rosRotation,
} from "./coordinates.js";

/** A fixed frame on a link, such as a sensor. Optical frames follow the ROS camera convention: right, down, forward. */
export interface Frame {
  object: Object3D;
  optical: boolean;
}
export interface RobotModel {
  name: string;
  /** Root body; its name becomes the base link. */
  base: RigidBody;
  /** Joint names to the physics joints connecting the bodies. */
  joints: Record<string, AxisJoint>;
  /** Frames by id; objects must descend from a body. */
  frames: Record<string, Frame>;
  /** Joints ROS commands and the interface each is commanded through; none means no ros2_control block. */
  commands: Record<string, "position" | "velocity">;
  commandTopic: string;
  stateTopic: string;
}

interface Collider {
  shape: Shape;
  /** Collider pose in link coordinates. */
  matrix: Matrix4;
  where: string;
}
interface Link {
  name: string;
  body: RigidBody;
  /** Link frame in body-local coordinates: identity for the base, the joint's child frame otherwise. */
  origin: Matrix4;
  /** Link frame relative to the base link at zero joint displacement. */
  pose: Matrix4;
  colliders: Collider[];
}
interface JointEntry {
  name: string;
  joint: AxisJoint;
  parent: Link;
  child: Link;
  /** Joint frame in parent link coordinates. */
  origin: Matrix4;
}

const NAME = /^[A-Za-z_][A-Za-z0-9_]*$/;
const FOOTPRINT = "base_footprint";
const COLOR = "0.603922 0.647059 0.670588 1";

/** Serializes a robot as URDF in ROS coordinates. World matrices must be current. */
export function exportURDF(model: RobotModel): string {
  const { base, links, joints } = resolveTree(model);
  const frames = Object.entries(model.frames);
  unique("robot", [model.name]);
  unique("link", [
    ...links.map((link) => link.name),
    FOOTPRINT,
    ...frames.map(([name]) => name),
  ]);
  unique("joint", [
    ...joints.map((joint) => joint.name),
    `${FOOTPRINT}_joint`,
    ...frames.map(([name]) => `${name}_joint`),
  ]);
  const material = `${model.name}_material`;
  return [
    `<?xml version="1.0"?>`,
    `<robot name="${model.name}">`,
    `  <material name="${material}">`,
    `    <color rgba="${COLOR}"/>`,
    `  </material>`,
    ...footprintXML(base, links),
    ...links.flatMap((link) => linkXML(link, material)),
    ...joints.flatMap(jointXML),
    ...frames.flatMap(([name, frame]) => frameXML(name, frame, links)),
    ...(Object.keys(model.commands).length ? controlXML(model, joints) : []),
    "</robot>",
    "",
  ].join("\n");
}

function unique(kind: string, names: string[]): void {
  const seen = new Set<string>();
  for (const name of names) {
    if (!NAME.test(name)) throw new Error(`Invalid URDF ${kind} name: ${name}`);
    if (seen.has(name)) throw new Error(`Duplicate URDF ${kind} name: ${name}`);
    seen.add(name);
  }
}

function resolveTree(model: RobotModel) {
  const base = link(model.base, new Matrix4(), new Matrix4());
  const links = new Map<RigidBody, Link>([[model.base, base]]);
  const pending = new Map(Object.entries(model.joints));
  const joints: JointEntry[] = [];
  while (pending.size) {
    const ready = nextJoint(pending, links);
    if (!ready)
      throw new Error(
        `Joints ${[...pending.keys()].join(", ")} do not connect to ${model.base.name}`,
      );
    const [name, joint, parent] = ready;
    pending.delete(name);
    const body = joint.options.body1;
    if (links.has(body))
      throw new Error(`Body ${body.name} has more than one parent joint`);
    const origin = parent.origin
      .clone()
      .invert()
      .multiply(joint.getFrame(0, new Matrix4()));
    const child = link(
      body,
      joint.getFrame(1, new Matrix4()),
      parent.pose.clone().multiply(origin),
    );
    links.set(body, child);
    joints.push({ name, joint, parent, child, origin });
  }
  return { base, links: [...links.values()], joints };
}

function link(body: RigidBody, origin: Matrix4, pose: Matrix4): Link {
  const inverse = origin.clone().invert();
  const colliders = body
    .getColliders()
    .filter((collider) => !collider.sensor)
    .map((collider) => {
      const { matrix, shape } = resolveCollider(body, collider);
      return {
        shape,
        matrix: inverse.clone().multiply(matrix),
        where: `${body.name}/${collider.name || collider.type}`,
      };
    });
  return { name: body.name, body, origin, pose, colliders };
}

function nextJoint(
  pending: Map<string, AxisJoint>,
  links: Map<RigidBody, Link>,
): [string, AxisJoint, Link] | undefined {
  for (const [name, joint] of pending) {
    const body0 = joint.options.body0;
    if (body0 === null)
      throw new Error(
        `Joint ${name} attaches to the world; URDF needs a parent body`,
      );
    const parent = links.get(body0);
    if (parent) return [name, joint, parent];
  }
  return undefined;
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

function jointXML({
  name,
  joint,
  parent,
  child,
  origin,
}: JointEntry): string[] {
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
  { object, optical }: Frame,
  links: Link[],
): string[] {
  let ancestor = object.parent;
  let link = links.find((candidate) => candidate.body === ancestor);
  while (ancestor && !link) {
    ancestor = ancestor.parent;
    link = links.find((candidate) => candidate.body === ancestor);
  }
  if (!link)
    throw new Error(`Frame ${name} does not descend from a robot body`);
  const relative = link.body.matrixWorld
    .clone()
    .invert()
    .multiply(object.matrixWorld);
  return [
    `  <link name="${name}"/>`,
    `  <joint name="${name}_joint" type="fixed">`,
    `    <parent link="${link.name}"/>`,
    `    <child link="${name}"/>`,
    originXML("    ", link.origin.clone().invert().multiply(relative), optical),
    `  </joint>`,
  ];
}

function footprintXML(base: Link, links: Link[]): string[] {
  const lowest = Math.min(
    ...links.flatMap((link) =>
      link.colliders.map(({ shape, matrix, where }) =>
        lowestPoint(shape, link.pose.clone().multiply(matrix), where),
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

function controlXML(model: RobotModel, joints: JointEntry[]): string[] {
  return [
    `  <ros2_control name="${model.name}" type="system">`,
    `    <hardware>`,
    `      <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>`,
    `      <param name="joint_commands_topic">${model.commandTopic}</param>`,
    `      <param name="joint_states_topic">${model.stateTopic}</param>`,
    `      <param name="trigger_joint_command_threshold">-1</param>`,
    `    </hardware>`,
    ...joints.flatMap(({ name }) => {
      const command = model.commands[name];
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
    case "box":
      return `<box size="${vec(shape.size[2], shape.size[0], shape.size[1])}"/>`;
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

/** Lowest point of a collider along Three.js up, with `pose` relative to the base link. */
function lowestPoint(shape: Shape, pose: Matrix4, where: string): number {
  const center = new Vector3().setFromMatrixPosition(pose);
  const rotation = new Quaternion().setFromRotationMatrix(pose);
  switch (shape.kind) {
    case "box": {
      const [x, y, z] = shape.size;
      const corners = [-1, 1].flatMap((sx) =>
        [-1, 1].flatMap((sy) =>
          [-1, 1].map(
            (sz) => new Vector3((sx * x) / 2, (sy * y) / 2, (sz * z) / 2),
          ),
        ),
      );
      return (
        center.y +
        Math.min(...corners.map((corner) => corner.applyQuaternion(rotation).y))
      );
    }
    case "sphere":
      return center.y - shape.radius;
    case "cylinder": {
      const along = Math.abs(new Vector3(0, 1, 0).applyQuaternion(rotation).y);
      const radial = Math.sqrt(Math.max(0, 1 - along * along));
      return center.y - (along * shape.height) / 2 - radial * shape.radius;
    }
    default:
      throw new Error(
        `Footprint needs primitive colliders; ${where} is a ${shape.kind}`,
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
