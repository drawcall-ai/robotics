import { registry, type PhysicsWorld } from "@drawcall/physics";
import { runAll } from "./cleanup.js";
import { SEGMENT, stamp } from "./messages.js";
import type { Camera, Object3D, RenderTarget, Texture } from "three";

export interface PublishOptions {
  retain?: boolean;
  /**
   * Written to the episode's recording only, never sent to other ROS nodes: simulator truth that
   * the robot's software must not see. Without a recording attached, it goes nowhere.
   */
  private?: boolean;
}

/** Where private messages go: the episode's recording. */
export interface Recording {
  /** Records `message` on `topic` at simulated `time`, in seconds. */
  write(topic: string, type: string, message: unknown, time: number): void;
}

export interface Connection {
  readonly connected: boolean;
  readonly error?: Error;
  publish(
    name: string,
    type: string,
    message: unknown,
    options?: PublishOptions,
  ): void;
  subscribe(
    name: string,
    type: string,
    receive: (message: unknown) => void,
  ): () => void;
  /** Whether anyone receives what is published on a topic; a connection that cannot tell says yes. */
  subscribed(name: string, type: string): boolean;
  /** Logs to /rosout, for a connection that can; the runtime writes to stderr otherwise. */
  log?(message: string): void;
  dispose(): void;
}

export interface Device {
  beforeStep?(time: number, delta: number): void;
  afterStep?(time: number, delta: number): void;
  evaluate?(): void;
  /** True while the device still works on a past step, such as a camera reading back its frame. */
  readonly busy?: boolean;
  stop?(): void;
  dispose(): void | Promise<void>;
}

/** The renderer operations sensors use. Three.js `WebGPURenderer` satisfies it in browsers and in Node. */
export interface CaptureRenderer {
  render(scene: Object3D, camera: Camera): unknown;
  getRenderTarget(): RenderTarget<Texture> | null;
  setRenderTarget(target: RenderTarget<Texture> | null): void;
  readRenderTargetPixelsAsync(
    target: RenderTarget,
    x: number,
    y: number,
    width: number,
    height: number,
  ): Promise<ArrayBufferView>;
}
export interface ROS2Options {
  /** The simulation to attach; defaults to the registry’s built world, if any. */
  world?: PhysicsWorld;
  /** Needed by cameras; a runtime that only builds a scene, as for URDF export, has none. */
  renderer?: CaptureRenderer;
  namespace?: string;
}

/** An absolute topic: name segments joined by `/`. */
const TOPIC = new RegExp(`^(?:/${SEGMENT})+$`);
let current: ROS2 | undefined;

/** The runtime devices register with. Creating one makes it the default that `getROS2()` returns to a scene. */
export class ROS2 {
  private simulation?: PhysicsWorld;
  private recording?: Recording;
  readonly renderer?: CaptureRenderer;
  readonly namespace: string;
  private readonly devices = new Set<Device>();
  private readonly hooks: (() => void)[] = [];
  private fault?: Error;
  private disposed = false;
  private disposal?: Promise<unknown[]>;
  constructor(
    readonly connection: Connection,
    options: ROS2Options = {},
  ) {
    if (!connection.connected)
      throw new Error("Connect ROS before creating a runtime");
    if (current)
      throw new Error(
        "Dispose the current ROS runtime before creating another",
      );
    const namespace = options.namespace ?? "/robot";
    if (namespace !== "/" && !TOPIC.test(namespace))
      throw new Error(`Invalid ROS namespace: ${namespace}`);
    this.renderer = options.renderer;
    this.namespace = namespace;
    const world = options.world ?? registry.world;
    if (world) this.attachWorld(world);
    current = this;
  }
  /** Attaches simulation callbacks after the initial scene has been authored and built. */
  attachWorld(world: PhysicsWorld): void {
    if (this.disposed) throw new Error("ROS runtime is disposed");
    if (this.simulation) throw new Error("ROS runtime already has a world");
    this.simulation = world;
    this.hooks.push(
      world.onBeforeStep((dt) => this.update("beforeStep", dt)),
      world.onAfterStep((dt) => this.update("afterStep", dt)),
    );
  }
  /** Receives the messages published privately from now on. */
  attachRecording(recording: Recording): void {
    if (this.recording) throw new Error("ROS runtime already has a recording");
    this.recording = recording;
  }
  get world(): PhysicsWorld {
    if (!this.simulation)
      throw new Error("Build and attach a physics world before simulation");
    return this.simulation;
  }
  get connected(): boolean {
    return !this.disposed && !this.fault && this.connection.connected;
  }
  /** True while any device still works on a past step; a runner that waits for it keeps sensor output deterministic. */
  get busy(): boolean {
    return [...this.devices].some((device) => device.busy);
  }
  get error(): Error | undefined {
    return this.fault ?? this.connection.error;
  }
  /** The absolute topic for `name`: under `namespace`, by default the runtime's, unless already absolute. */
  topic(name: string, namespace = this.namespace): string {
    if (namespace !== "/" && !TOPIC.test(namespace))
      throw new Error(`Invalid ROS namespace: ${namespace}`);
    const topic = name.startsWith("/")
      ? name
      : `${namespace === "/" ? "" : namespace}/${name}`;
    if (!TOPIC.test(topic)) throw new Error(`Invalid ROS topic: ${topic}`);
    return topic;
  }
  /** The registered devices of a class, in registration order. */
  find<T>(type: abstract new (...args: never[]) => T): T[] {
    return [...this.devices].filter(
      (device): device is T & Device => device instanceof type,
    );
  }
  register(device: Device): () => void {
    if (!this.connected) throw new Error("ROS runtime is disconnected");
    if (this.devices.has(device))
      throw new Error("Device is already registered");
    this.devices.add(device);
    return () => this.devices.delete(device);
  }
  publish(
    name: string,
    type: string,
    message: unknown,
    options?: PublishOptions,
  ): void {
    // Private messages bypass the connection, so a final diagnostic is recorded after a fault.
    if (options?.private) {
      if (this.disposed) throw new Error("ROS runtime is disposed");
      const time = this.simulation?.time ?? 0;
      this.recording?.write(this.topic(name), type, message, time);
      return;
    }
    if (!this.connected) throw new Error("ROS runtime is disconnected");
    this.connection.publish(this.topic(name), type, message, options);
  }
  /** Whether anyone receives a topic, so devices can skip work nobody reads, as image_transport does. */
  subscribed(name: string, type: string): boolean {
    return this.connection.subscribed(this.topic(name), type);
  }
  /** The returned release is a no-op once the runtime is disposed: disposing the connection ends every subscription. */
  subscribe(
    name: string,
    type: string,
    receive: (message: unknown) => void,
  ): () => void {
    if (!this.connected) throw new Error("ROS runtime is disconnected");
    const unsubscribe = this.connection.subscribe(
      this.topic(name),
      type,
      (message) => {
        try {
          receive(message);
        } catch (error) {
          this.fail(error);
        }
      },
    );
    return () => {
      if (!this.disposed) unsubscribe();
    };
  }
  /** Logs `message` to /rosout, or to stderr where the connection cannot. */
  log(message: string): void {
    if (this.connection.log) this.connection.log(message);
    else console.error(message);
  }
  /** Records the first fault and stops every actuator; the runtime stays disconnected until disposed. */
  fail(error: unknown): void {
    if (this.fault) return;
    const cause = error instanceof Error ? error : new Error(String(error));
    this.fault = cause;
    try {
      runAll(
        [...this.devices].map((device) => () => device.stop?.()),
        "Failed to stop ROS devices",
      );
    } catch (failure) {
      this.fault = new AggregateError([cause, failure], cause.message, {
        cause,
      });
    }
  }
  private update(phase: "beforeStep" | "afterStep", dt: number): void {
    if (this.fault) return;
    if (!this.connection.connected) {
      this.fail(this.connection.error ?? new Error("ROS connection lost"));
      return;
    }
    try {
      if (phase === "afterStep")
        this.publish("/clock", "rosgraph_msgs/msg/Clock", {
          clock: stamp(this.world.time),
        });
      for (const device of this.devices) device[phase]?.(this.world.time, dt);
      if (phase === "afterStep")
        for (const device of this.devices) device.evaluate?.();
    } catch (error) {
      this.fail(error);
      throw this.fault;
    }
  }
  /** Disconnects synchronously and reports synchronous release failures. Use disposeAsync to also join device work. */
  dispose(): void {
    const failure = this.release();
    if (failure) throw failure;
  }
  /** Disconnects immediately, then joins every device. Repeated calls report all original release failures. */
  async disposeAsync(): Promise<void> {
    this.release();
    const errors = await this.disposal;
    if (errors?.length)
      throw new AggregateError(errors, "Failed to dispose ROS runtime");
  }
  private release(): AggregateError | undefined {
    if (this.disposed) return;
    this.disposed = true;
    if (current === this) current = undefined;
    const releases: (() => void | Promise<void>)[] = [
      ...[...this.devices].map((device) => () => device.dispose()),
      ...this.hooks,
      () => this.connection.dispose(),
    ];
    this.devices.clear();
    const errors: unknown[] = [];
    const pending: Promise<void>[] = [];
    for (const release of releases) {
      try {
        const completion = release();
        if (completion) pending.push(completion);
      } catch (error) {
        errors.push(error);
      }
    }
    // Observe every rejection now, including when the caller only disconnects synchronously.
    this.disposal = Promise.allSettled(pending).then((results) => [
      ...errors,
      ...results
        .filter((result) => result.status === "rejected")
        .map((result) => result.reason),
    ]);
    if (errors.length)
      return new AggregateError(errors, "Failed to dispose ROS runtime");
  }
}

export function getROS2(): ROS2 {
  if (!current)
    throw new Error("Create a ROS runtime before creating ROS devices");
  return current;
}
