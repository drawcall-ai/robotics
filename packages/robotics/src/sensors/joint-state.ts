import type { AxisJoint } from "@drawcall/physics";
import { getROS2, type ROS2 } from "../runtime.js";
import { Device } from "../device.js";
import { Schedule, stamp } from "../time.js";
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
