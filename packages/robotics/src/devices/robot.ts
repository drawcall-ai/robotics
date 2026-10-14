import { AxisJoint, FixedJoint, Joint, RigidBody } from "@drawcall/physics";
import { Group, type Object3D } from "three";
import {
  header,
  identifier,
  namespace as checked,
  NAMESPACE,
  topic,
} from "../messages.js";
import { buildLinkTree, type LinkTree } from "../urdf/links.js";
import type { Device, ROS2 } from "../runtime.js";
import { JointActuator } from "./actuator.js";
import { Model } from "./model.js";
import { Schedule } from "./schedule.js";
import { runAll } from "../cleanup.js";

/** What the children describe, read at first use and frozen: children added later are ignored. */
interface Structure {
  base: RigidBody;
  bodies: RigidBody[];
  joints: Map<string, AxisJoint>;
  frames: Map<string, Object3D>;
  links: Map<string, AxisJoint | FixedJoint>;
}

/** Any object with a frame id is a fixed frame of the robot it sits in. */
function isFrame(object: Object3D): object is Object3D & { frameId: string } {
  return "frameId" in object && typeof object.frameId === "string";
}

export interface RobotOptions {
  /**
   * The robot's own namespace, for several robots in one scene. Its topics go under it and its TF
   * frames carry it as a prefix, as `robot_state_publisher`'s `frame_prefix` does. Without one, the
   * robot's topics go under `/robot` and its frames keep plain names.
   */
  namespace?: string;
  /** Joint state and TF publication rate in Hz. */
  rate?: number;
  /**
   * Publishes the robot's description, joint states and TF privately: recorded, never seen by the
   * robot's software, for a scene whose robot reports only what its real hardware does. A private
   * robot takes no joint commands either; devices of the scene drive its joints.
   */
  private?: boolean;
}

/**
 * A robot is a group: the bodies, joints and sensors under it are the robot. Its URDF and joint
 * states follow from that structure: the base is the body no joint hangs from, every named axis
 * joint is a URDF joint, a joint driven by a `JointActuator` is commanded by that actuator, and
 * each sensor is a frame. It drives ROS while a runtime's scene holds it.
 */
export class Robot extends Group implements Device {
  readonly isDevice = true;
  readonly namespace: string;
  /** Prepended to every TF frame the robot publishes: empty, or its namespace and a slash. */
  readonly framePrefix: string;
  /** Whether the robot publishes privately. */
  readonly private: boolean;
  private readonly schedule: Schedule;
  private ros?: ROS2;
  private unsubscribe?: () => void;
  private resolved?: Structure;
  private linkTree?: LinkTree;
  private model?: Model;
  constructor({
    namespace,
    rate = 100,
    private: hidden = false,
  }: RobotOptions = {}) {
    super();
    this.private = hidden;
    this.namespace = checked(namespace ?? NAMESPACE);
    this.framePrefix =
      namespace && namespace !== "/" ? `${namespace.slice(1)}/` : "";
    this.schedule = new Schedule(rate);
  }
  join(ros: ROS2): void {
    if (ros.find(Robot).some((robot) => robot.namespace === this.namespace))
      throw new Error(
        `Two robots share the namespace ${this.namespace}; give each its own`,
      );
    // One command topic per robot, as the topic-based ros2_control hardware publishes it.
    this.unsubscribe = this.private
      ? undefined
      : ros.subscribe(
          this.topic("joint_commands"),
          "sensor_msgs/msg/JointState",
          (message) => {
            for (const actuator of this.actuators.values())
              actuator.receive(message, ros.world.time);
          },
        );
    this.ros = ros;
  }
  /**
   * Stops listening and releases its actuators, as if never commanded, so it holds its pose again
   * when a runtime's scene takes it back.
   */
  leave(): void {
    const unsubscribe = this.unsubscribe;
    this.ros = this.unsubscribe = this.model = undefined;
    runAll(
      [
        () => unsubscribe?.(),
        () => {
          for (const actuator of this.actuators.values()) actuator.release();
        },
      ],
      "Failed to release robot",
    );
  }
  /** The absolute topic for `name` in the robot's namespace. */
  topic(name: string): string {
    return topic(name, this.namespace);
  }
  /** The TF frame the robot publishes for `id`. */
  frame(id: string): string {
    return this.framePrefix + id;
  }
  /** The body every joint hangs from; its pose is the robot's. */
  get base(): RigidBody {
    return this.structure().base;
  }
  /** Every named axis joint under the robot, by name. */
  get joints(): ReadonlyMap<string, AxisJoint> {
    return this.structure().joints;
  }
  get descriptionTopic(): string {
    return this.topic("robot_description");
  }
  /** The kinematic tree in link frames, measured at first use and frozen like the structure. */
  get tree(): LinkTree {
    if (this.linkTree) return this.linkTree;
    const { base, bodies, links, frames } = this.structure();
    this.updateWorldMatrix(true, true);
    this.linkTree = buildLinkTree(this.name, base, bodies, links, frames);
    return this.linkTree;
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
    const links = new Map<string, AxisJoint | FixedJoint>();
    const frames = new Map<string, Object3D>();
    this.traverse((object) => {
      if (object instanceof RigidBody) bodies.push(object);
      if (object instanceof Joint) allJoints.push(object);
      if (object instanceof AxisJoint || object instanceof FixedJoint) {
        identifier("joint", object.name);
        if (links.has(object.name))
          throw new Error(`${this.name} has two joints named ${object.name}`);
        links.set(object.name, object);
        if (object instanceof AxisJoint) joints.set(object.name, object);
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
    this.resolved = { base, bodies, joints, frames, links };
    return this.resolved;
  }
  /** Called after authoring, before the first physics step; also usable while paused at startup. */
  beforeStep(time: number): void {
    if (!this.model) {
      if (this.private && this.actuators.size)
        throw new Error(
          `${this.name} is private and takes no joint commands; drive its joints from the scene instead of with JointActuators`,
        );
      this.model = new Model(this, this.joined(), time);
    }
    for (const actuator of this.actuators.values()) actuator.beforeStep(time);
  }
  /** Lets every actuator go, as when the runtime faults. */
  stop(): void {
    for (const actuator of this.actuators.values()) actuator.stop();
  }
  afterStep(time: number): void {
    const { joints } = this.structure();
    if (!this.schedule.due(time)) return;
    this.model?.publish(time);
    if (!joints.size) return;
    const states = [...joints.values()].map((joint) => joint.getState());
    this.joined().publish(
      this.topic("joint_states"),
      "sensor_msgs/msg/JointState",
      {
        header: header(time, ""),
        name: [...joints.keys()],
        position: states.map((state) => state.position),
        velocity: states.map((state) => state.velocity),
        effort: [],
      },
      { private: this.private },
    );
  }
  private joined(): ROS2 {
    if (!this.ros)
      throw new Error(`Robot ${this.name} is not under a ROS runtime's scene`);
    return this.ros;
  }
}

/** The robot `object` is mounted on, if any. */
export function robotOf(object: Object3D): Robot | undefined {
  for (let parent = object.parent; parent; parent = parent.parent)
    if (parent instanceof Robot) return parent;
  return undefined;
}

/** The robot named `name` under `root`, or its only robot. */
export function findRobot(root: Object3D, name?: string): Robot {
  const robots: Robot[] = [];
  root.traverse((object) => {
    if (object instanceof Robot && (name === undefined || object.name === name))
      robots.push(object);
  });
  const [robot] = robots;
  if (!robot || robots.length !== 1)
    throw new Error(
      name === undefined
        ? `The scene has ${robots.length} robots; pick one by name`
        : `The scene has no robot named ${name}`,
    );
  return robot;
}
