import type { AxisJoint } from "@drawcall/physics";
import { Object3D, Quaternion, Vector3 } from "three";
import { getROS2, type ROS2 } from "./runtime.js";
import { Device } from "./device.js";
import { rosPosition, rosRotation } from "./frames.js";
import { Schedule, stamp } from "./time.js";

export function header(time: number, frameId: string) {
  if (!frameId || frameId.startsWith("/"))
    throw new Error("ROS frame IDs must be nonempty and relative");
  return { stamp: stamp(time), frame_id: frameId };
}
export function transform(
  ros: ROS2,
  object: Object3D,
  frameId: string,
  time: number,
): void {
  object.updateWorldMatrix(true, false);
  const translation = rosPosition(object.getWorldPosition(new Vector3()));
  const rotation = rosRotation(object.getWorldQuaternion(new Quaternion()));
  ros.publish("/tf", "tf2_msgs/msg/TFMessage", {
    transforms: [
      {
        header: header(time, "map"),
        child_frame_id: frameId,
        transform: { translation, rotation },
      },
    ],
  });
}
export class JointStateSensor extends Device {
  private readonly joints: [string, AxisJoint][];
  private readonly schedule: Schedule;
  constructor(options: {
    joints: Record<string, AxisJoint>;
    rate?: number;
    ros?: ROS2;
  }) {
    const ros = options.ros ?? getROS2();
    const joints = Object.entries(options.joints);
    if (!joints.length)
      throw new Error("JointStateSensor needs at least one joint");
    if (joints.some(([, joint]) => joint.world !== ros.world))
      throw new Error("Joint and ROS worlds differ");
    const schedule = new Schedule(options.rate ?? 30);
    super({ ros });
    this.joints = joints;
    this.schedule = schedule;
  }
  afterStep(time: number): void {
    if (!this.schedule.due(time)) return;
    const states = this.joints.map(([, joint]) => joint.getState());
    this.ros.publish("joint_states", "sensor_msgs/msg/JointState", {
      header: { stamp: stamp(time), frame_id: "" },
      name: this.joints.map(([name]) => name),
      position: states.map((s) => s.position),
      velocity: states.map((s) => s.velocity),
      effort: [],
    });
  }
}
export class PoseSensor extends Device {
  private readonly object: Object3D;
  private readonly frameId: string;
  private readonly childFrameId: string;
  private readonly schedule: Schedule;
  constructor(options: {
    object: Object3D;
    frameId?: string;
    childFrameId?: string;
    rate?: number;
    ros?: ROS2;
  }) {
    const ros = options.ros ?? getROS2();
    const object = options.object;
    const frameId = options.frameId ?? "map";
    if (frameId !== "map")
      throw new Error("PoseSensor reports ground truth in the map frame");
    const childFrameId = options.childFrameId ?? "base_link";
    header(0, childFrameId);
    const schedule = new Schedule(options.rate ?? 30);
    super({ ros });
    this.object = object;
    this.frameId = frameId;
    this.childFrameId = childFrameId;
    this.schedule = schedule;
  }
  afterStep(time: number): void {
    if (!this.schedule.due(time)) return;
    this.object.updateWorldMatrix(true, false);
    const position = rosPosition(this.object.getWorldPosition(new Vector3()));
    const orientation = rosRotation(
      this.object.getWorldQuaternion(new Quaternion()),
    );
    this.ros.publish("pose", "geometry_msgs/msg/PoseStamped", {
      header: header(time, this.frameId),
      pose: { position, orientation },
    });
    transform(this.ros, this.object, this.childFrameId, time);
  }
}
