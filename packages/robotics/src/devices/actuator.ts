import {
  GenericJoint,
  JointDrive,
  ScalarJoint,
  type JointDriveOptions,
  type JointDriveTarget,
} from "@drawcall/physics";
import { getROS2, type Device, type ROS2 } from "../runtime.js";

/** Seconds after which a joint command expires and the actuator goes passive. */
const COMMAND_TIMEOUT = 0.5;

export type CommandInterface = "position" | "velocity" | "effort";
export interface JointActuatorOptions extends JointDriveOptions {
  /** The ros2_control command interface: a position servo, a velocity motor, or a torque input. */
  readonly interface: CommandInterface;
  /** URDF velocity limit for controllers; no physics engine enforces it. */
  readonly maxVelocity?: number;
}
interface Command {
  value: number;
  time: number;
  wall: number;
}

/**
 * A drive that ros2_control commands. Attach it with `joint.setDrive`: it reads the command
 * for its joint's name from `joint_commands`, applies it before each step, and goes passive when
 * the command expires. Its target belongs to the controller, so scene code cannot set it.
 */
export class JointActuator
  extends JointDrive<JointActuatorOptions>
  implements Device
{
  readonly ros: ROS2;
  private command?: Command;
  private readonly unsubscribe: () => void;
  private readonly unregister: () => void;
  constructor({ ros, ...options }: JointActuatorOptions & { ros?: ROS2 }) {
    super(options);
    validateGains(options);
    if (
      options.maxVelocity !== undefined &&
      (!Number.isFinite(options.maxVelocity) || options.maxVelocity < 0)
    )
      throw new Error("Actuator velocity limit must be finite and nonnegative");
    this.ros = ros ?? getROS2();
    this.unsubscribe = this.ros.subscribe(
      "joint_commands",
      "sensor_msgs/msg/JointState",
      (message) => this.receive(message),
    );
    this.unregister = this.ros.register(this);
  }
  override setTarget(): never {
    throw new Error("Actuator targets come from joint_commands");
  }
  /** A copy keeps the gains and interface; its target comes from ROS. */
  override copy(): this {
    return this;
  }
  private receive(message: unknown): void {
    const name = this.joint?.name;
    if (name === undefined) return;
    const index = list(message, "name").indexOf(name);
    if (index < 0) return;
    const value: unknown = list(message, this.options.interface)[index];
    if (typeof value !== "number" || !Number.isFinite(value))
      throw new Error(
        `Joint command for ${name} needs a finite ${this.options.interface}`,
      );
    this.command = {
      value,
      time: this.ros.world.time,
      wall: performance.now() / 1000,
    };
  }
  beforeStep(time: number): void {
    const command = this.command;
    const wall = performance.now() / 1000;
    if (
      !command ||
      time - command.time > COMMAND_TIMEOUT ||
      wall - command.wall > COMMAND_TIMEOUT
    ) {
      this.stop();
      return;
    }
    const target: JointDriveTarget = {
      position: 0,
      velocity: 0,
      effort: 0,
      [this.options.interface]: command.value,
    };
    if (!sameTarget(this.target, target)) super.setTarget(target);
  }
  /** Drops the command and exerts no force, without braking. */
  stop(): void {
    this.command = undefined;
    if (this.target) super.setTarget(undefined);
  }
  dispose(): void {
    this.stop();
    this.unsubscribe();
    this.unregister();
    const joint = this.joint;
    if (!joint || joint.disposed) return;
    if (joint instanceof ScalarJoint) joint.setDrive(undefined);
    if (joint instanceof GenericJoint)
      for (const [dof, drive] of joint.drives)
        if (drive === this) joint.setDrive(dof, undefined);
  }
}

function validateGains(options: JointActuatorOptions): void {
  const stiffness = options.stiffness ?? 0;
  const damping = options.damping ?? 0;
  switch (options.interface) {
    case "position":
      if (!stiffness) throw new Error("A position actuator needs stiffness");
      return;
    case "velocity":
      if (!damping || stiffness)
        throw new Error("A velocity actuator needs damping and no stiffness");
      return;
    case "effort":
      if (stiffness || damping)
        throw new Error("An effort actuator has no gains");
  }
}

function sameTarget(
  a: JointDriveTarget | undefined,
  b: JointDriveTarget,
): boolean {
  return (
    a !== undefined &&
    a.position === b.position &&
    a.velocity === b.velocity &&
    a.effort === b.effort
  );
}

/** A `JointState` array field; native transports deliver float arrays as typed arrays, rosbridge as plain arrays. */
function list(message: unknown, field: string): readonly unknown[] {
  if (typeof message !== "object" || message === null)
    throw new Error("Invalid joint command message");
  const value: unknown = Reflect.get(message, field);
  if (Array.isArray(value)) return value;
  if (value instanceof Float64Array) return Array.from(value);
  throw new Error(`Joint command needs a ${field} array`);
}
