import { afterEach, vi } from "vitest";
import { Scene, type RenderTarget, type Texture } from "three";
import { RevoluteJoint, RigidBody } from "@drawcall/physics";
import { buildWorld } from "@drawcall/physics-mujoco";
import type { Connection } from "../src/runtime.js";
import { JointActuator } from "../src/devices/actuator.js";
import { Robot } from "../src/devices/robot.js";
import type { CaptureRenderer } from "../src/runtime.js";
import { ROS2 } from "../src/runtime.js";

/** Releases run in reverse after each test of the importing file. */
const cleanup: (() => void)[] = [];
afterEach(() => {
  for (const release of cleanup.splice(0).reverse()) release();
  vi.restoreAllMocks();
});
export function defer(release: () => void): void {
  cleanup.push(release);
}

/** Records publications and hands received messages to subscribers. */
export class ConnectionStub implements Connection {
  connected = true;
  disposed = false;
  readonly messages: { name: string; type: string; message: unknown }[] = [];
  readonly receivers = new Map<string, Set<(message: unknown) => void>>();
  /** Whether the stub reports subscribers on every topic. */
  listening = true;
  subscribed(): boolean {
    return this.listening;
  }
  publish(name: string, type: string, message: unknown): void {
    this.messages.push({ name, type, message });
  }
  subscribe(
    name: string,
    _type: string,
    receive: (message: unknown) => void,
  ): () => void {
    let receivers = this.receivers.get(name);
    if (!receivers) this.receivers.set(name, (receivers = new Set()));
    receivers.add(receive);
    return () => {
      if (this.disposed) throw new Error("Released after close");
      receivers.delete(receive);
      if (!receivers.size) this.receivers.delete(name);
    };
  }
  receive(name: string, message: unknown): void {
    const receivers = this.receivers.get(name);
    if (!receivers?.size) throw new Error(`No subscription for ${name}`);
    for (const receive of receivers) receive(message);
  }
  dispose(): void {
    this.connected = false;
    this.disposed = true;
    this.receivers.clear();
  }
}

/** Returns a constant pixel value on readback, once `flush()` completes the readbacks. */
export class RendererStub implements CaptureRenderer {
  target: RenderTarget<Texture> | null = null;
  pixel = 7;
  failRender = false;
  readbacks: (() => void)[] = [];
  render(): void {
    if (this.failRender) throw new Error("GPU render failed");
  }
  getRenderTarget() {
    return this.target;
  }
  setRenderTarget(target: RenderTarget<Texture> | null): void {
    this.target = target;
  }
  readRenderTargetPixelsAsync(
    _target: RenderTarget,
    _x: number,
    _y: number,
    width: number,
    height: number,
  ): Promise<Uint8Array> {
    return new Promise((resolve) => {
      this.readbacks.push(() =>
        resolve(new Uint8Array(width * height * 4).fill(this.pixel)),
      );
    });
  }
  async flush(): Promise<void> {
    for (const resolve of this.readbacks.splice(0)) resolve();
    await new Promise((resolve) => setTimeout(resolve, 0));
  }
}

/** A runtime over a stepping MuJoCo world without gravity; tests add what it simulates to `world.scene`. */
export async function simulation({
  gravity = [0, 0, 0],
}: { gravity?: [number, number, number] } = {}) {
  const world = await buildWorld({
    scene: new Scene(),
    gravity,
    fixedDelta: 0.01,
  });
  defer(() => world.dispose());
  const connection = new ConnectionStub();
  const renderer = new RendererStub();
  const ros = new ROS2(connection, { scene: world.scene, world, renderer });
  defer(() => ros.dispose());
  return { world, ros, connection, renderer };
}

/** A robot with one actuated wheel on a static base; stiffness makes the actuator position-commanded. */
export function cart({
  stiffness,
  interface: command = stiffness ? "position" : "velocity",
  namespace,
}: {
  stiffness?: number;
  interface?: "position" | "velocity" | "effort";
  namespace?: string;
} = {}) {
  const base = new RigidBody({ bodyType: "static" });
  base.name = "base_link";
  const wheel = new RigidBody({
    mass: 1,
    centerOfMass: [0, 0, 0],
    diagonalInertia: [1, 1, 1],
  });
  wheel.name = "wheel_link";
  const joint = new RevoluteJoint({ body0: base, body1: wheel, axis: "Z" });
  joint.name = "wheel";
  const actuator = new JointActuator({
    interface: command,
    stiffness,
    damping: command === "effort" ? undefined : 10,
    maxForce: 20,
  });
  joint.setDrive(actuator);
  const robot = new Robot({ namespace });
  robot.name = "cart";
  robot.add(base, wheel, joint);
  return { robot, base, wheel, joint, actuator };
}

export function jointCommand(
  field: "position" | "velocity" | "effort",
  value: number,
  name = "wheel",
) {
  return {
    name: [name],
    position: [],
    velocity: [],
    effort: [],
    [field]: [value],
  };
}
