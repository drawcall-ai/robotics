import {
  GenericJoint,
  JointDrive,
  ScalarJoint,
  type JointDriveOptions,
  type JointDriveTarget,
} from "@drawcall/physics";
import { getROS2, type Device, type ROS2 } from "../runtime.js";
import { runAll } from "../cleanup.js";

/** Simulation seconds after which a joint command expires and the actuator goes passive; a pause never expires one. */
const COMMAND_TIMEOUT = 0.5;

export type CommandInterface = "position" | "velocity" | "effort";
export interface JointActuatorOptions extends JointDriveOptions {
  /** The ros2_control command interface: a position servo, a velocity motor, or a torque input. */
  readonly interface: CommandInterface;
}
interface Command {
  value: number;
  time: number;
}

/**
 * A drive that ros2_control commands. Attach it with `joint.setDrive`: its robot hands it the
 * `joint_commands` for its joint's name, it applies the command before each step, and goes passive
 * when the command expires. Its target belongs to the controller, so scene code cannot set it.
 */
export class JointActuator
  extends JointDrive<JointActuatorOptions>
  implements Device
{
  readonly ros: ROS2;
  private command?: Command;
  private commanded = false;
  private resting?: number;
  private readonly unregister: () => void;
  constructor({ ros, ...options }: JointActuatorOptions & { ros?: ROS2 }) {
    super(options);
    validateGains(options);
    this.ros = ros ?? getROS2();
    this.unregister = this.ros.register(this);
  }
  override setTarget(): never {
    throw new Error("Actuator targets come from joint_commands");
  }
  /** Inherited cloning constructs the gains and interface; controller targets are never copied. */
  override copy(): this {
    return this;
  }
  /** Takes this joint's value from a `sensor_msgs/JointState` command, if it names the joint. */
  receive(message: unknown): void {
    const name = this.joint?.name;
    if (name === undefined) return;
    const index = list(message, "name").indexOf(name);
    if (index < 0) return;
    const value: unknown = list(message, this.options.interface)[index];
    if (typeof value !== "number" || !Number.isFinite(value))
      throw new Error(
        `Joint command for ${name} needs a finite ${this.options.interface}`,
      );
    this.command = { value, time: this.ros.world.time };
    this.commanded = true;
  }
  beforeStep(time: number): void {
    const command = this.command;
    if (!command || time - command.time > COMMAND_TIMEOUT) {
      if (this.commanded) this.stop();
      else this.hold(time);
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
  /**
   * Holds the pose the scene authored, as a powered servo does before its controller connects.
   * Once commanded, an expiry releases the joint instead: a controller that stops talking lets go.
   */
  private hold(time: number): void {
    if (this.options.interface === "effort") return;
    // A joint only measures itself once the backend has built it, one step in.
    if (this.resting === undefined && time > 0)
      this.resting = this.restingPosition();
    if (this.resting === undefined) return;
    const target: JointDriveTarget = {
      position: this.options.interface === "position" ? this.resting : 0,
      velocity: 0,
      effort: 0,
    };
    if (!sameTarget(this.target, target)) super.setTarget(target);
  }
  private restingPosition(): number | undefined {
    const joint = this.joint;
    // A joint measures itself from world matrices, which authoring leaves stale.
    for (const body of [joint?.options.body0, joint?.options.body1])
      body?.updateWorldMatrix(true, false);
    if (joint instanceof GenericJoint) {
      const dof = [...joint.drives].find(([, drive]) => drive === this)?.[0];
      return dof === undefined ? undefined : joint.getState(dof).position;
    }
    if (!(joint instanceof ScalarJoint)) return undefined;
    const state = joint.getState();
    return "position" in state ? state.position : undefined;
  }
  /** Drops the command and holds the current pose, as if never commanded: a paused controller no longer drives the joint. */
  release(): void {
    this.stop();
    this.commanded = false;
    // Measured again at the next step, where the joint now is.
    this.resting = undefined;
  }
  /** Drops the command and exerts no force, without braking. */
  stop(): void {
    this.command = undefined;
    if (this.target) super.setTarget(undefined);
  }
  dispose(): void {
    runAll(
      [
        () => this.stop(),
        this.unregister,
        () => {
          const joint = this.joint;
          if (!joint) return;
          if (joint instanceof ScalarJoint) joint.setDrive(undefined);
          if (joint instanceof GenericJoint)
            for (const [dof, drive] of joint.drives)
              if (drive === this) joint.setDrive(dof, undefined);
        },
      ],
      "Failed to dispose joint actuator",
    );
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
