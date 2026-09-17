import { AxisJoint, Joint, RigidBody } from "@drawcall/physics";
import { Group, type Object3D } from "three";
import { header, identifier } from "../messages.js";
import { getROS2, type ROS2 } from "../runtime.js";
import { JointActuator } from "./actuator.js";
import { Schedule } from "./schedule.js";

/** Joint state publication rate in Hz. */
const STATE_RATE = 20;

/** What the children describe, read at first use and frozen: children added later are ignored. */
interface Structure {
  base: RigidBody;
  joints: Map<string, AxisJoint>;
  frames: Map<string, Object3D>;
}

/** Any object with a frame id is a fixed frame of the robot it sits in. */
function isFrame(object: Object3D): object is Object3D & { frameId: string } {
  return "frameId" in object && typeof object.frameId === "string";
}

/**
 * A robot is a group: the bodies, joints and sensors under it are the robot. Its URDF and joint
 * states follow from that structure: the base is the body no joint hangs from, every named axis
 * joint is a URDF joint, a joint driven by a `JointActuator` is commanded by that actuator, and
 * each sensor is a frame.
 */
export class Robot extends Group {
  readonly ros: ROS2;
  private readonly schedule = new Schedule(STATE_RATE);
  private readonly unregister: () => void;
  private resolved?: Structure;
  private disposed = false;
  constructor(options: { ros?: ROS2 } = {}) {
    super();
    this.ros = options.ros ?? getROS2();
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
  /** The actuators ROS commands, by joint name. */
  get actuators(): ReadonlyMap<string, JointActuator> {
    const actuators = new Map<string, JointActuator>();
    for (const [name, joint] of this.joints)
      if (joint.drive instanceof JointActuator)
        actuators.set(name, joint.drive);
    return actuators;
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
    this.resolved = { base, joints, frames };
    return this.resolved;
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
  dispose(): void {
    if (this.disposed) return;
    this.disposed = true;
    this.unregister();
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
