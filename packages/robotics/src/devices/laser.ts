import { Object3D, Quaternion, Vector3 } from "three";
import { header, identifier, topic } from "../messages.js";
import type { Device, ROS2 } from "../runtime.js";
import { Schedule } from "./schedule.js";
import { robotOf } from "./robot.js";

export class LaserSensor extends Object3D implements Device {
  readonly isDevice = true;
  readonly frameId: string;
  private readonly schedule: Schedule;
  private readonly samples: number;
  private readonly range: number;
  private readonly minRange: number;
  private readonly angleMin: number;
  private readonly angleMax: number;
  private ros?: ROS2;
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
    } = {},
  ) {
    super();
    const samples = options.samples ?? 61;
    const range = options.range ?? 6;
    const minRange = options.minRange ?? 0;
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
    this.name = options.name ?? "scan";
    this.frameId = identifier("frame", options.frameId ?? "laser");
    this.schedule = new Schedule(options.rate ?? 10);
    this.samples = samples;
    this.range = range;
    this.minRange = minRange;
    this.angleMin = angleMin;
    this.angleMax = angleMax;
  }
  join(ros: ROS2): void {
    this.ros = ros;
  }
  leave(): void {
    this.ros = undefined;
  }
  get topic(): string {
    return topic(this.name, robotOf(this)?.namespace);
  }
  afterStep(time: number): void {
    const ros = this.ros;
    if (!ros || !this.schedule.due(time)) return;
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
      const hit = ros.world.raycast(origin, direction, this.range);
      // REP 117: +Inf for no return within range, -Inf for a return inside the dead zone.
      if (!hit) return Infinity;
      return hit.distance < this.minRange ? -Infinity : hit.distance;
    });
    ros.publish(this.topic, "sensor_msgs/msg/LaserScan", {
      header: header(time, robotOf(this)?.frame(this.frameId) ?? this.frameId),
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
}
