import { Object3D, Quaternion, Vector3 } from "three";
import type { RigidBody } from "@drawcall/physics";
import { getROS2, type ROS2 } from "./runtime.js";
import { withDevice } from "./device.js";
import { Schedule } from "./time.js";
import { header, transform } from "./sensors.js";

export class LaserSensor extends withDevice(Object3D) {
  private readonly schedule: Schedule;
  private readonly samples: number;
  private readonly range: number;
  private readonly frameId: string;
  private readonly topic: string;
  private readonly excludeBodies: readonly RigidBody[];
  constructor(
    options: {
      name?: string;
      rate?: number;
      samples?: number;
      range?: number;
      frameId?: string;
      excludeBodies?: readonly RigidBody[];
      ros?: ROS2;
    } = {},
  ) {
    const ros = options.ros ?? getROS2();
    const schedule = new Schedule(options.rate ?? 10);
    const samples = options.samples ?? 61;
    const range = options.range ?? 6;
    const frameId = options.frameId ?? "laser";
    const topic = options.name ?? "scan";
    ros.topic(topic);
    const excludeBodies = [...(options.excludeBodies ?? [])];
    if (!Number.isInteger(samples) || samples < 2)
      throw new Error("Laser needs at least two samples");
    if (!Number.isFinite(range) || range <= 0)
      throw new Error("Laser range must be positive");
    header(0, frameId);
    super({ ros });
    this.schedule = schedule;
    this.samples = samples;
    this.range = range;
    this.frameId = frameId;
    this.topic = topic;
    this.excludeBodies = excludeBodies;
  }
  afterStep(time: number): void {
    if (!this.schedule.due(time)) return;
    this.updateWorldMatrix(true, false);
    const origin = this.getWorldPosition(new Vector3());
    const rotation = this.getWorldQuaternion(new Quaternion());
    const increment = Math.PI / (this.samples - 1);
    const ranges = Array.from({ length: this.samples }, (_, i) => {
      const angle = -Math.PI / 2 + i * increment;
      const direction = new Vector3(
        -Math.sin(angle),
        0,
        -Math.cos(angle),
      ).applyQuaternion(rotation);
      return (
        this.ros.world.raycast(origin, direction, this.range, {
          excludeBodies: this.excludeBodies,
        })?.distance ?? this.range
      );
    });
    this.ros.publish(this.topic, "sensor_msgs/msg/LaserScan", {
      header: header(time, this.frameId),
      angle_min: -Math.PI / 2,
      angle_max: Math.PI / 2,
      angle_increment: increment,
      time_increment: 0,
      scan_time: this.schedule.period,
      range_min: 0,
      range_max: this.range,
      ranges,
      intensities: [],
    });
    transform(this.ros, this, this.frameId, time);
  }
  dispose(): void {
    super.dispose();
    this.removeFromParent();
  }
}
