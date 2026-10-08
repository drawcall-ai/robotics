// The torque each joint passes on to its link: what holds up everything beyond it against
// gravity and against the table, board or queen pushing on it. At rest it equals the motor's
// torque plus the gear friction's, so it is what bends the gears and printed parts between the
// servo's encoder and the link (see `calibration.ts`).
//
// Ground truth: a policy must not read this file.
import type { RevoluteJoint, RigidBody } from "@drawcall/physics";
import { getROS2 } from "@drawcall/robotics";
import { Matrix4, Vector3 } from "three";

const GRAVITY = new Vector3(0, -9.81, 0);

export interface Link {
  body: RigidBody;
  mass: number;
  /** Centre of mass in body coordinates, from its collision shapes. */
  centre: Vector3;
}

/**
 * For each joint, the torque about its axis, N·m, the joint exerts on its link to hold `beyond`
 * (that joint's link and every link after it) still against gravity and outside contacts, and
 * the part of it that holds them against gravity alone.
 */
export function linkTorques(
  joints: readonly RevoluteJoint[],
  beyond: readonly (readonly Link[])[],
  robot: object,
): { total: number[]; gravity: number[] } {
  const world = getROS2().world;
  const frame = new Matrix4();
  const axis = new Vector3();
  const origin = new Vector3();
  const arm = new Vector3();
  const point = new Vector3();
  const gravity: number[] = [];
  const total = joints.map((joint, i) => {
    const parent = joint.options.body0!;
    parent.updateWorldMatrix(true, false);
    frame.multiplyMatrices(parent.matrixWorld, joint.options.frame0!);
    axis.setFromMatrixColumn(frame, 2).normalize();
    origin.setFromMatrixPosition(frame);
    let load = 0;
    for (const { body, mass, centre } of beyond[i]!) {
      body.updateWorldMatrix(true, false);
      point.copy(centre).applyMatrix4(body.matrixWorld);
      load += arm.subVectors(point, origin).cross(GRAVITY).multiplyScalar(mass).dot(axis);
    }
    gravity.push(-load);
    for (const { body } of beyond[i]!) {
      for (const contact of world.contacts(body)) {
        if (contact.otherBody.parent === robot) continue;
        const force = contact.normal.clone().multiplyScalar(contact.force);
        load += arm.subVectors(contact.point, origin).cross(force).dot(axis);
      }
    }
    return -load;
  });
  return { total, gravity };
}
