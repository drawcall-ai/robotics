import { Object3D, Quaternion, Vector3 } from "three";
import { getROS2, type ROS2 } from "../runtime.js";
import { withDevice } from "../device.js";
import { Schedule } from "../time.js";
import { frameId, header } from "../messages.js";

export class LaserSensor extends withDevice(Object3D) {
  private readonly schedule: Schedule;
  private readonly samples: number;
  private readonly range: number;
  private readonly minRange: number;
  private readonly angleMin: number;
  private readonly angleMax: number;
  readonly frameId: string;
  readonly topic: string;
  constructor(
    options: {
      name?: string;
      rate?: number;
      samples?: number;
      range?: number;
      /** Returns closer than this are dropped, as on a real lidar; also published as `range_min`. */
      minRange?: number;
      angleMin?: number;
      angleMax?: number;
      frameId?: string;
      ros?: ROS2;
    } = {},
  ) {
    const ros = options.ros ?? getROS2();
    const schedule = new Schedule(options.rate ?? 10);
    const samples = options.samples ?? 61;
    const range = options.range ?? 6;
    const minRange = options.minRange ?? 0;
    const frame = frameId(options.frameId ?? "laser");
    const name = options.name ?? "scan";
    const topic = ros.topic(name);
    if (!Number.isInteger(samples) || samples < 2)
      throw new Error("Laser needs at least two samples");
    if (!Number.isFinite(range) || range <= 0)
      throw new Error("Laser range must be positive");
    if (!Number.isFinite(minRange) || minRange < 0 || minRange >= range)
      throw new Error("Laser minimum range must lie in [0, range)");
    const angleMin = options.angleMin ?? -Math.PI;
    const angleMax = options.angleMax ?? Math.PI;
    if (
      !Number.isFinite(angleMin) ||
      !Number.isFinite(angleMax) ||
      angleMax <= angleMin ||
      angleMax - angleMin > 2 * Math.PI
    )
      throw new Error(
        "Laser angles must define a positive sweep of at most 2π",
      );
    super({ ros });
    this.schedule = schedule;
    this.samples = samples;
    this.range = range;
    this.minRange = minRange;
    this.angleMin = angleMin;
    this.angleMax = angleMax;
    this.frameId = frame;
    this.name = name;
    this.topic = topic;
  }
  afterStep(time: number): void {
    if (!this.schedule.due(time)) return;
    this.updateWorldMatrix(true, false);
    const origin = this.getWorldPosition(new Vector3());
    const rotation = this.getWorldQuaternion(new Quaternion());
    const increment = (this.angleMax - this.angleMin) / (this.samples - 1);
    const ranges = Array.from({ length: this.samples }, (_, i) => {
      const angle = this.angleMin + i * increment;
      const direction = new Vector3(
        -Math.sin(angle),
        0,
        -Math.cos(angle),
      ).applyQuaternion(rotation);
      const hit = this.ros.world.raycast(origin, direction, this.range);
      // No return, or one inside the dead zone, is reported beyond the maximum like a real scanner does.
      return hit && hit.distance >= this.minRange
        ? hit.distance
        : this.range + 1;
    });
    this.ros.publish(this.topic, "sensor_msgs/msg/LaserScan", {
      header: header(time, this.frameId),
      angle_min: this.angleMin,
      angle_max: this.angleMax,
      angle_increment: increment,
      time_increment: 0,
      scan_time: this.schedule.period,
      range_min: this.minRange,
      range_max: this.range,
      ranges,
      intensities: [],
    });
  }
  dispose(): void {
    super.dispose();
    this.removeFromParent();
  }
}
