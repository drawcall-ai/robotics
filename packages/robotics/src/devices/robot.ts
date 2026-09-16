import {
  AxisJoint,
  Joint,
  RigidBody,
  type JointMotor,
} from "@drawcall/physics";
import { Group, type Object3D } from "three";
import { runAll } from "../cleanup.js";
import { header, identifier } from "../messages.js";
import { getROS2, type ROS2 } from "../runtime.js";
import { Schedule } from "./schedule.js";

/** Seconds after which a joint command expires and its motor is disabled. */
const COMMAND_TIMEOUT = 0.5;
/** Joint state publication rate in Hz. */
const STATE_RATE = 20;

interface Command {
  value: number;
  time: number;
  wall: number;
}
/** A motor ROS commands, through the interface its stiffness implies. */
export interface Drive {
  motor: JointMotor;
  command: "position" | "velocity";
}
/** What the children describe, read at first use and frozen: children added later are ignored. */
interface Structure {
  base: RigidBody;
  joints: Map<string, AxisJoint>;
  drives: Map<string, Drive>;
  frames: Map<string, Object3D>;
}

/** Any object with a frame id is a fixed frame of the robot it sits in. */
function isFrame(object: Object3D): object is Object3D & { frameId: string } {
  return "frameId" in object && typeof object.frameId === "string";
}

/**
 * A robot is a group: the bodies, joints and sensors under it are the robot. Its URDF, joint states and
 * joint commands follow from that structure: the base is the body no joint hangs from, every named axis
 * joint is a URDF joint, a joint's motor makes it commanded, and each sensor is a frame.
 */
export class Robot extends Group {
  readonly ros: ROS2;
  private readonly schedule = new Schedule(STATE_RATE);
  private readonly commands = new Map<string, Command>();
  private readonly unsubscribe: () => void;
  private readonly unregister: () => void;
  private resolved?: Structure;
  private disposed = false;
  constructor(options: { ros?: ROS2 } = {}) {
    super();
    this.ros = options.ros ?? getROS2();
    this.unsubscribe = this.ros.subscribe(
      "joint_commands",
      "sensor_msgs/msg/JointState",
      (message) => this.receive(message),
    );
    this.unregister = this.ros.register(this);
  }
  /** The body every joint hangs from; its pose is the robot's. */
  get base(): RigidBody {
    return this.structure().base;
  }
  /** Every named axis joint under the robot, by name. */
  get joints(): ReadonlyMap<string, AxisJoint> {
    return this.structure().joints;
  }
  /** The joints ROS commands, by name. */
  get drives(): ReadonlyMap<string, Drive> {
    return this.structure().drives;
  }
  /** The fixed frames by id: every sensor under the robot. */
  get frames(): ReadonlyMap<string, Object3D> {
    return this.structure().frames;
  }

  private structure(): Structure {
    if (this.resolved) return this.resolved;
    identifier("robot", this.name);
    const bodies: RigidBody[] = [];
    const allJoints: Joint[] = [];
    const joints = new Map<string, AxisJoint>();
    const frames = new Map<string, Object3D>();
    this.traverse((object) => {
      if (object instanceof RigidBody) bodies.push(object);
      if (object instanceof Joint) allJoints.push(object);
      if (object instanceof AxisJoint) {
        identifier("joint", object.name);
        if (joints.has(object.name))
          throw new Error(`${this.name} has two joints named ${object.name}`);
        joints.set(object.name, object);
      }
      if (isFrame(object)) {
        if (frames.has(object.frameId))
          throw new Error(
            `${this.name} has two frames named ${object.frameId}`,
          );
        frames.set(object.frameId, object);
      }
    });
    const roots = bodies.filter(
      (body) => !allJoints.some((joint) => joint.options.body1 === body),
    );
    const [base] = roots;
    if (!base || roots.length !== 1)
      throw new Error(
        `${this.name} needs exactly one body that no joint hangs from, found ${roots.length}`,
      );
    const drives = new Map<string, Drive>();
    for (const [name, joint] of joints)
      if (joint.motor)
        drives.set(name, {
          motor: joint.motor,
          command: joint.motor.options.stiffness ? "position" : "velocity",
        });
    this.resolved = { base, joints, drives, frames };
    return this.resolved;
  }
  private receive(message: unknown): void {
    if (typeof message !== "object" || message === null)
      throw new Error("Invalid joint command message");
    const { drives } = this.structure();
    const names: unknown = Reflect.get(message, "name");
    if (!Array.isArray(names))
      throw new Error("Joint command needs a name array");
    const time = this.ros.world.time;
    const wall = performance.now() / 1000;
    names.forEach((name: unknown, index) => {
      if (typeof name !== "string")
        throw new Error("Joint command names must be strings");
      const drive = drives.get(name);
      if (!drive) throw new Error(`Joint command names unknown joint ${name}`);
      const values: unknown = Reflect.get(message, drive.command);
      // Native transports deliver float arrays as typed arrays, rosbridge as plain arrays.
      const value: unknown =
        Array.isArray(values) || values instanceof Float64Array
          ? values[index]
          : undefined;
      if (typeof value !== "number" || !Number.isFinite(value))
        throw new Error(
          `Joint command for ${name} needs a finite ${drive.command}`,
        );
      this.commands.set(name, { value, time, wall });
    });
  }
  beforeStep(time: number): void {
    const wall = performance.now() / 1000;
    for (const [name, { motor, command }] of this.structure().drives) {
      const latest = this.commands.get(name);
      if (
        !latest ||
        time - latest.time > COMMAND_TIMEOUT ||
        wall - latest.wall > COMMAND_TIMEOUT
      ) {
        this.commands.delete(name);
        if (motor.enabled) motor.setEnabled(false);
        continue;
      }
      const target = { position: 0, velocity: 0, [command]: latest.value };
      if (
        motor.target?.position !== target.position ||
        motor.target.velocity !== target.velocity
      )
        motor.setTarget(target);
      if (!motor.enabled) motor.setEnabled(true);
    }
  }
  afterStep(time: number): void {
    const { joints } = this.structure();
    if (!joints.size || !this.schedule.due(time)) return;
    const states = [...joints.values()].map((joint) => joint.getState());
    this.ros.publish("joint_states", "sensor_msgs/msg/JointState", {
      header: header(time, ""),
      name: [...joints.keys()],
      position: states.map((state) => state.position),
      velocity: states.map((state) => state.velocity),
      effort: [],
    });
  }
  stop(): void {
    this.commands.clear();
    for (const { motor } of this.resolved?.drives.values() ?? [])
      if (motor.enabled) motor.setEnabled(false);
  }
  dispose(): void {
    if (this.disposed) return;
    this.disposed = true;
    runAll(
      [() => this.stop(), this.unsubscribe, this.unregister],
      "Failed to dispose robot",
    );
  }
}

/** The runtime's robot named `name`, or its only robot. */
export function findRobot(ros: ROS2, name?: string): Robot {
  const robots = ros
    .find(Robot)
    .filter((robot) => name === undefined || robot.name === name);
  const [robot] = robots;
  if (!robot || robots.length !== 1)
    throw new Error(
      name === undefined
        ? `The scene has ${robots.length} robots; pick one by name`
        : `The scene has no robot named ${name}`,
    );
  return robot;
}
