import { disposeAll } from "./cleanup.js";
import type { DeviceLifecycle } from "./device.js";
import type { PhysicsWorld } from "@drawcall/physics";
import type { Scene, WebGLRenderer } from "three";
import { RosbridgeConnection, type Connection } from "./connection.js";
import { stamp } from "./time.js";

export interface ROS2Options {
  world: PhysicsWorld;
  scene: Scene;
  renderer: WebGLRenderer;
  namespace?: string;
}
let current: ROS2 | undefined;
let connecting = false;

export class ROS2 {
  readonly world: PhysicsWorld;
  readonly scene: Scene;
  readonly renderer: WebGLRenderer;
  readonly namespace: string;
  private readonly devices = new Set<DeviceLifecycle>();
  private readonly subscriptions = new Set<() => void>();
  private readonly hooks: (() => void)[];
  private fault?: Error;
  private disposed = false;
  constructor(
    readonly connection: Connection,
    options: ROS2Options,
  ) {
    if (!connection.connected)
      throw new Error("Connect ROS before creating a runtime");
    this.world = options.world;
    this.scene = options.scene;
    this.renderer = options.renderer;
    this.namespace = options.namespace ?? "/robot";
    this.topic("validate");
    this.hooks = [
      this.world.onBeforeStep((dt) => this.update("beforeStep", dt)),
      this.world.onAfterStep((dt) => this.update("afterStep", dt)),
    ];
  }
  get connected(): boolean {
    return !this.disposed && !this.error && this.connection.connected;
  }
  get error(): Error | undefined {
    return this.fault ?? this.connection.error;
  }
  get dropped(): number {
    return this.connection.dropped;
  }
  topic(name: string): string {
    const topic = name.startsWith("/")
      ? name
      : `${this.namespace === "/" ? "" : this.namespace}/${name}`;
    if (!/^\/(?:[A-Za-z_][A-Za-z0-9_]*\/)*[A-Za-z_][A-Za-z0-9_]*$/.test(topic))
      throw new Error(`Invalid ROS topic: ${topic}`);
    return topic;
  }
  register(device: DeviceLifecycle): () => void {
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
    const release = () => {
      if (this.subscriptions.delete(release)) unsubscribe();
    };
    this.subscriptions.add(release);
    return release;
  }
  fail(error: unknown): void {
    const cause = error instanceof Error ? error : new Error(String(error));
    this.fault = cause;
    const errors = this.stop();
    this.fault = errors.length
      ? new AggregateError([cause, ...errors], cause.message, { cause })
      : cause;
  }
  private stop(): unknown[] {
    const errors: unknown[] = [];
    for (const device of this.devices) {
      try {
        device.stop?.();
      } catch (error) {
        errors.push(error);
      }
    }
    return errors;
  }
  private update(phase: "beforeStep" | "afterStep", dt: number): void {
    if (!this.connected) {
      const errors = this.stop();
      if (errors.length) {
        this.fault = new AggregateError(errors, "Failed to stop ROS devices");
        throw this.fault;
      }
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
      ...this.subscriptions,
      ...this.hooks,
      () => this.connection.dispose(),
    ];
    this.devices.clear();
    disposeAll(releases, "Failed to dispose ROS runtime");
  }
}

export async function setupROS2(
  url: string,
  options: ROS2Options & { timeout?: number },
): Promise<ROS2> {
  if (current || connecting)
    throw new Error(
      "Dispose the current ROS runtime before setting up another",
    );
  const timeout = options.timeout ?? 10000;
  if (!Number.isFinite(timeout) || timeout <= 0)
    throw new Error("Connection timeout must be positive");
  const connection = new RosbridgeConnection();
  connecting = true;
  try {
    await connection.connect(url, timeout);
    current = new ROS2(connection, options);
    return current;
  } catch (error) {
    connection.dispose();
    throw error;
  } finally {
    connecting = false;
  }
}
export function getROS2(): ROS2 {
  if (!current?.connected)
    throw new Error("Call and await setupROS2() before creating ROS devices");
  return current;
}
