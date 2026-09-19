import { Matrix4, Quaternion, Vector3 } from "three";
import { identifier } from "../messages.js";
import {
  RigidBody,
  resolveCollider,
  type AxisJoint,
  type FixedJoint,
  type Shape,
} from "@drawcall/physics";

export interface LinkCollider {
  shape: Shape;
  /** Collider pose in link coordinates. */
  matrix: Matrix4;
  where: string;
}

export interface Link {
  name: string;
  body: RigidBody;
  /** Link frame in body-local coordinates: identity for the base, the joint's child frame otherwise. */
  origin: Matrix4;
  /** Link frame relative to the base link at zero joint displacement. */
  restPose: Matrix4;
  colliders: LinkCollider[];
}
export interface LinkJoint {
  name: string;
  joint: AxisJoint | FixedJoint;
  parent: Link;
  child: Link;
  /** Joint frame in parent link coordinates. */
  origin: Matrix4;
}

/**
 * The kinematic tree in link frames: the base and every body an axis joint hangs from, parents before
 * children. Every pose is read relative to its own body; only `restPose` composes across bodies.
 */
export function buildLinkTree(
  root: RigidBody,
  axes: ReadonlyMap<string, AxisJoint | FixedJoint>,
): { base: Link; links: Map<RigidBody, Link>; joints: LinkJoint[] } {
  const base = link(root, new Matrix4(), new Matrix4());
  const links = new Map<RigidBody, Link>([[root, base]]);
  const joints: LinkJoint[] = [];
  for (const [name, joint] of axes)
    if (joint.options.body0 === null)
      throw new Error(
        `Joint ${name} attaches to the world; URDF needs a parent body`,
      );
  const descend = (parent: Link): void => {
    for (const [name, joint] of axes) {
      if (joint.options.body0 !== parent.body) continue;
      const body = joint.options.body1;
      if (links.has(body))
        throw new Error(`Body ${body.name} has more than one parent joint`);
      const origin = inLinkFrame(
        parent.origin,
        joint.getFrame(0, new Matrix4()),
      );
      const child = link(
        body,
        joint.getFrame(1, new Matrix4()),
        parent.restPose.clone().multiply(origin),
      );
      links.set(body, child);
      joints.push({ name, joint, parent, child, origin });
      descend(child);
    }
  };
  descend(base);
  const unreached = [...axes.keys()].filter(
    (name) => !joints.some((j) => j.name === name),
  );
  if (unreached.length)
    throw new Error(
      `Joints ${unreached.join(", ")} do not connect to ${root.name}`,
    );
  return { base, links, joints };
}

function link(body: RigidBody, origin: Matrix4, restPose: Matrix4): Link {
  identifier("link", body.name);
  const colliders = body.getColliders().map((collider) => {
    const { matrix, shape } = resolveCollider(body, collider);
    return {
      shape,
      matrix: inLinkFrame(origin, matrix),
      where: `${body.name}/${collider.name || collider.type}`,
    };
  });
  return { name: body.name, body, origin, restPose, colliders };
}

/** Lowest point of a collider along Three.js up, with `pose` relative to the base link. */
export function lowestPoint(
  shape: Shape,
  pose: Matrix4,
  where: string,
): number {
  const center = new Vector3().setFromMatrixPosition(pose);
  const rotation = new Quaternion().setFromRotationMatrix(pose);
  // World up in shape coordinates; the shape's support point against it is its lowest.
  const up = new Vector3(0, 1, 0).applyQuaternion(rotation.invert());
  switch (shape.kind) {
    case "box": {
      const [x, y, z] = shape.size;
      const extent =
        Math.abs(up.x) * x + Math.abs(up.y) * y + Math.abs(up.z) * z;
      return center.y - extent / 2;
    }
    case "sphere":
      return center.y - shape.radius;
    case "cylinder":
      return (
        center.y -
        (Math.abs(up.y) * shape.height) / 2 -
        Math.hypot(up.x, up.z) * shape.radius
      );
    case "mesh": {
      const vertices = shape.geometry.getAttribute("position");
      let lowest = Infinity;
      for (let i = 0; i < vertices.count; i++)
        lowest = Math.min(
          lowest,
          new Vector3().fromBufferAttribute(vertices, i).applyMatrix4(pose).y,
        );
      if (!Number.isFinite(lowest))
        throw new Error(`Empty collision mesh ${where}`);
      return lowest;
    }
    case "capsule":
      return center.y - (Math.abs(up.y) * shape.height) / 2 - shape.radius;
  }
}

/** A body-local pose expressed in the frame of the link with `origin`. */
export function inLinkFrame(origin: Matrix4, bodyLocal: Matrix4): Matrix4 {
  return origin.clone().invert().multiply(bodyLocal);
}

export function footprintPose(base: Link, links: Link[]): Matrix4 {
  const rotation = new Matrix4().makeRotationFromQuaternion(
    base.body.getWorldQuaternion(new Quaternion()),
  );
  const lowest = Math.min(
    0,
    ...links.flatMap((link) =>
      link.colliders.map(({ shape, matrix, where }) =>
        lowestPoint(
          shape,
          rotation.clone().multiply(link.restPose).multiply(matrix),
          where,
        ),
      ),
    ),
  );
  return rotation.setPosition(0, -lowest, 0);
}
