import {
  AxisJoint,
  Joint,
  RigidBody,
  type JointMotor,
} from "@drawcall/physics";
import { Group } from "three";
import { runAll } from "./cleanup.js";
import { withDevice, type DeviceOptions } from "./device.js";
import { header } from "./messages.js";
import type { ROS2 } from "./runtime.js";
import { CameraSensor } from "./sensors/camera.js";
import { LaserSensor } from "./sensors/laser.js";
import { Schedule } from "./time.js";
import { exportURDF, type Frame } from "./urdf.js";

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
interface Drive {
  motor: JointMotor;
  field: "position" | "velocity";
}
/** What the children describe, read once after the scene is built. */
interface Structure {
  base: RigidBody;
  joints: Map<string, AxisJoint>;
  drives: Map<string, Drive>;
  frames: Record<string, Frame>;
}

/**
 * A robot is a group: the bodies, joints and sensors under it are the robot. Its URDF, joint states and
 * joint commands follow from that structure: the base is the body no joint hangs from, every named axis
 * joint is a URDF joint, a joint's motor makes it commanded, and each sensor is a frame.
 */
export class Robot extends withDevice(Group) {
  private readonly schedule = new Schedule(STATE_RATE);
  private readonly commands = new Map<string, Command>();
  private readonly unsubscribe: () => void;
  private structure?: Structure;
  private disposed = false;
  constructor(options: DeviceOptions = {}) {
    super(options);
    try {
      this.unsubscribe = this.ros.subscribe(
        "joint_commands",
        "sensor_msgs/msg/JointState",
        (message) => this.receive(message),
      );
    } catch (error) {
      super.dispose();
      throw error;
    }
  }
  /** The body every joint hangs from; its pose is the robot's. */
  get base(): RigidBody {
    return this.resolve().base;
  }
  /** Serializes the robot as URDF, with a ros2_control block when it has motors. */
  urdf(): string {
    const { base, joints, drives, frames } = this.resolve();
    this.updateWorldMatrix(true, true);
    return exportURDF({
      name: this.name,
      base,
      joints: Object.fromEntries(joints),
      frames,
      commands: Object.fromEntries(
        [...drives].map(([name, { field }]) => [name, field]),
      ),
      commandTopic: this.ros.topic("joint_commands"),
      stateTopic: this.ros.topic("joint_states"),
    });
  }

  private resolve(): Structure {
    if (this.structure) return this.structure;
    if (!this.name) throw new Error("A robot needs a name");
    const bodies: RigidBody[] = [];
    const children: Joint[] = [];
    const joints = new Map<string, AxisJoint>();
    const frames: Record<string, Frame> = {};
    this.traverse((object) => {
      if (object instanceof RigidBody) bodies.push(object);
      if (object instanceof Joint) children.push(object);
      if (object instanceof AxisJoint) {
        if (!object.name)
          throw new Error(`Every joint of ${this.name} needs a name`);
        if (joints.has(object.name))
          throw new Error(`${this.name} has two joints named ${object.name}`);
        joints.set(object.name, object);
      }
      if (object instanceof CameraSensor || object instanceof LaserSensor) {
        if (object.frameId in frames)
          throw new Error(
            `${this.name} has two frames named ${object.frameId}`,
          );
        frames[object.frameId] = {
          object,
          optical: object instanceof CameraSensor,
        };
      }
    });
    const roots = bodies.filter(
      (body) => !children.some((joint) => joint.options.body1 === body),
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
          field: joint.motor.options.stiffness ? "position" : "velocity",
        });
    this.structure = { base, joints, drives, frames };
    return this.structure;
  }
  private receive(message: unknown): void {
    if (typeof message !== "object" || message === null)
      throw new Error("Invalid joint command message");
    const { drives } = this.resolve();
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
      const values: unknown = Reflect.get(message, drive.field);
      // Native transports deliver float arrays as typed arrays, rosbridge as plain arrays.
      const value: unknown =
        Array.isArray(values) || values instanceof Float64Array
          ? values[index]
          : undefined;
      if (typeof value !== "number" || !Number.isFinite(value))
        throw new Error(
          `Joint command for ${name} needs a finite ${drive.field}`,
        );
      this.commands.set(name, { value, time, wall });
    });
  }
  beforeStep(time: number): void {
    const wall = performance.now() / 1000;
    for (const [name, { motor, field }] of this.resolve().drives) {
      const command = this.commands.get(name);
      if (
        !command ||
        time - command.time > COMMAND_TIMEOUT ||
        wall - command.wall > COMMAND_TIMEOUT
      ) {
        this.commands.delete(name);
        if (motor.enabled) motor.setEnabled(false);
        continue;
      }
      const target = { position: 0, velocity: 0, [field]: command.value };
      if (
        motor.target?.position !== target.position ||
        motor.target.velocity !== target.velocity
      )
        motor.setTarget(target);
      if (!motor.enabled) motor.setEnabled(true);
    }
  }
  afterStep(time: number): void {
    const { joints } = this.resolve();
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
    for (const { motor } of this.structure?.drives.values() ?? [])
      if (motor.enabled) motor.setEnabled(false);
  }
  dispose(): void {
    if (this.disposed) return;
    this.disposed = true;
    runAll(
      [() => this.stop(), this.unsubscribe, () => super.dispose()],
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
        ? `The scene has ${robots.length} robots; pick one with --robot`
        : `The scene has no robot named ${name}`,
    );
  return robot;
}
