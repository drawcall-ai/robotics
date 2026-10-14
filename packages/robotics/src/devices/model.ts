import { Matrix4 } from "three";
import { FixedJoint } from "@drawcall/physics";
import { rosPose } from "../coordinates.js";
import { header } from "../messages.js";
import { robotURDF } from "../urdf/serialize.js";
import type { ROS2 } from "../runtime.js";
import type { Robot } from "./robot.js";

/** Frozen model metadata, with live transforms measured in exactly the same link frames as URDF. */
export class Model {
  constructor(
    private readonly robot: Robot,
    private readonly ros: ROS2,
    time: number,
  ) {
    const { base, joints, frames, footprint } = robot.tree;
    const frame = (name: string) => robot.frame(name);
    const fixed = [
      transform(frame("base_footprint"), frame(base.name), footprint, time),
      ...joints
        .filter(({ joint }) => joint instanceof FixedJoint)
        .map(({ parent, child, origin }) =>
          transform(frame(parent.name), frame(child.name), origin, time),
        ),
      ...frames.map(({ name, link, origin, optical }) =>
        transform(frame(link.name), frame(name), origin, time, optical),
      ),
    ];
    ros.publish(
      robot.descriptionTopic,
      "std_msgs/msg/String",
      { data: robotURDF(robot) },
      { retain: true, private: robot.private },
    );
    ros.publish(
      "/tf_static",
      "tf2_msgs/msg/TFMessage",
      { transforms: fixed },
      { retain: true, private: robot.private },
    );
  }
  publish(time: number): void {
    this.robot.updateWorldMatrix(true, true);
    const transforms = this.robot.tree.joints
      .filter(({ joint }) => !(joint instanceof FixedJoint))
      .map(({ parent, child }) => {
        const parentPose = parent.body.matrixWorld
          .clone()
          .multiply(parent.origin);
        const childPose = child.body.matrixWorld.clone().multiply(child.origin);
        return transform(
          this.robot.frame(parent.name),
          this.robot.frame(child.name),
          parentPose.invert().multiply(childPose),
          time,
        );
      });
    if (transforms.length)
      this.ros.publish(
        "/tf",
        "tf2_msgs/msg/TFMessage",
        { transforms },
        { private: this.robot.private },
      );
  }
}

function transform(
  parent: string,
  child: string,
  matrix: Matrix4,
  time: number,
  optical = false,
) {
  const { position: p, rotation: q } = rosPose(matrix, optical);
  return {
    header: header(time, parent),
    child_frame_id: child,
    transform: {
      translation: { x: p.x, y: p.y, z: p.z },
      rotation: { x: q.x, y: q.y, z: q.z, w: q.w },
    },
  };
}
