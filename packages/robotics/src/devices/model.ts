import { Camera, Matrix4, Quaternion, Vector3 } from "three";
import { FixedJoint } from "@drawcall/physics";
import {
  rosOpticalRotation,
  rosPosition,
  rosRotation,
} from "../coordinates.js";
import { header } from "../messages.js";
import { robotURDF } from "../urdf/serialize.js";
import { buildLinkTree, footprintPose } from "../urdf/links.js";
import type { Robot } from "./robot.js";

/** Frozen model metadata, with live transforms measured in exactly the same link frames as URDF. */
export class Model {
  private readonly tree;
  constructor(
    private readonly robot: Robot,
    time: number,
  ) {
    robot.updateWorldMatrix(true, true);
    this.tree = buildLinkTree(robot.base, robot.linkJoints);
    const { base, links, joints } = this.tree;
    const fixed = [
      transform(
        "base_footprint",
        base.name,
        footprintPose(base, [...links.values()]),
        time,
      ),
    ];
    for (const { name, joint, parent, child, origin } of joints) {
      if (joint instanceof FixedJoint)
        fixed.push(transform(parent.name, child.name, origin, time));
      if (!name) throw new Error("Unnamed robot joint");
    }
    for (const [name, object] of robot.frames) {
      const link = [...links.values()].find(({ body }) => {
        let ancestor = object.parent;
        while (ancestor && ancestor !== body) ancestor = ancestor.parent;
        return ancestor === body;
      });
      if (!link) throw new Error(`Frame ${name} does not belong to a link`);
      const local = link.origin
        .clone()
        .invert()
        .multiply(link.body.matrixWorld.clone().invert())
        .multiply(object.matrixWorld);
      fixed.push(
        transform(link.name, name, local, time, object instanceof Camera),
      );
    }
    robot.ros.publish(
      robot.descriptionTopic,
      "std_msgs/msg/String",
      { data: robotURDF(robot) },
      { retain: true },
    );
    robot.ros.publish(
      "/tf_static",
      "tf2_msgs/msg/TFMessage",
      { transforms: fixed },
      { retain: true },
    );
  }
  publish(time: number): void {
    this.robot.updateWorldMatrix(true, true);
    const transforms = this.tree.joints
      .filter(({ joint }) => !(joint instanceof FixedJoint))
      .map(({ parent, child }) => {
        const parentPose = parent.body.matrixWorld
          .clone()
          .multiply(parent.origin);
        const childPose = child.body.matrixWorld.clone().multiply(child.origin);
        return transform(
          parent.name,
          child.name,
          parentPose.invert().multiply(childPose),
          time,
        );
      });
    if (transforms.length)
      this.robot.ros.publish("/tf", "tf2_msgs/msg/TFMessage", { transforms });
  }
}

function transform(
  parent: string,
  child: string,
  matrix: Matrix4,
  time: number,
  optical = false,
) {
  const position = new Vector3(),
    quaternion = new Quaternion(),
    scale = new Vector3();
  matrix.decompose(position, quaternion, scale);
  const p = rosPosition(position);
  const q = optical ? rosOpticalRotation(quaternion) : rosRotation(quaternion);
  return {
    header: header(time, parent),
    child_frame_id: child,
    transform: {
      translation: { x: p.x, y: p.y, z: p.z },
      rotation: { x: q.x, y: q.y, z: q.z, w: q.w },
    },
  };
}
