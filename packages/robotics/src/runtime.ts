import { getDefaultWorld, type PhysicsWorld } from "@drawcall/physics";
import { runAll } from "./cleanup.js";
import { SEGMENT, stamp } from "./messages.js";
import type { Camera, Object3D, RenderTarget, Texture } from "three";

export interface Connection {
  readonly connected: boolean;
  readonly error?: Error;
  publish(name: string, type: string, message: unknown): void;
  subscribe(
    name: string,
    type: string,
    receive: (message: unknown) => void,
  ): () => void;
  dispose(): void;
}

export interface Device {
  beforeStep?(time: number, delta: number): void;
  afterStep?(time: number, delta: number): void;
  stop?(): void;
  dispose(): void;
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
  /** The world the scene builds in; the physics default world unless given. */
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
  readonly world: PhysicsWorld;
  readonly renderer?: CaptureRenderer;
  readonly namespace: string;
  private readonly devices = new Set<Device>();
  private readonly hooks: (() => void)[];
  private fault?: Error;
  private disposed = false;
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
    this.world = options.world ?? getDefaultWorld();
    this.renderer = options.renderer;
    this.namespace = namespace;
    this.hooks = [
      this.world.onBeforeStep((dt) => this.update("beforeStep", dt)),
      this.world.onAfterStep((dt) => this.update("afterStep", dt)),
    ];
    current = this;
  }
  get connected(): boolean {
    return !this.disposed && !this.fault && this.connection.connected;
  }
  get error(): Error | undefined {
    return this.fault ?? this.connection.error;
  }
  /** The absolute topic for `name`: under the namespace unless already absolute. */
  topic(name: string): string {
    const topic = name.startsWith("/")
      ? name
      : `${this.namespace === "/" ? "" : this.namespace}/${name}`;
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
  publish(name: string, type: string, message: unknown): void {
    if (!this.connected) throw new Error("ROS runtime is disconnected");
    this.connection.publish(this.topic(name), type, message);
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
    } catch (error) {
      this.fail(error);
      throw this.fault;
    }
  }
  dispose(): void {
    if (this.disposed) return;
    this.disposed = true;
    if (current === this) current = undefined;
    const releases = [
      ...[...this.devices].map((device) => () => device.dispose()),
      ...this.hooks,
      () => this.connection.dispose(),
    ];
    this.devices.clear();
    runAll(releases, "Failed to dispose ROS runtime");
  }
}

export function getROS2(): ROS2 {
  if (!current)
    throw new Error("Create a ROS runtime before creating ROS devices");
  return current;
}
