import { afterEach, vi } from "vitest";
import type { RenderTarget, Texture } from "three";
import {
  AuthoringWorld,
  RevoluteJoint,
  RigidBody,
  setDefaultWorld,
} from "@drawcall/physics";
import { setupWorld } from "@drawcall/physics-rapier";
import type { Connection } from "../src/runtime.js";
import { OfflineConnection } from "../src/connections/offline.js";
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

/** A runtime over a stepping Rapier world without gravity. */
export async function simulation({ namespace }: { namespace?: string } = {}) {
  const world = await setupWorld({ gravity: [0, 0, 0], fixedDelta: 0.01 });
  defer(() => world.dispose());
  const connection = new ConnectionStub();
  const renderer = new RendererStub();
  const ros = new ROS2(connection, { world, renderer, namespace });
  defer(() => ros.dispose());
  return { world, ros, connection, renderer };
}

/** A runtime that only builds a scene, as URDF export does. */
export function offline(namespace?: string): ROS2 {
  setDefaultWorld(new AuthoringWorld());
  const ros = new ROS2(new OfflineConnection(), { namespace });
  defer(() => ros.dispose());
  return ros;
}

/** A robot with one actuated wheel on a static base; stiffness makes the actuator position-commanded. */
export function cart(
  ros: ROS2,
  {
    stiffness,
    interface: command = stiffness ? "position" : "velocity",
  }: {
    stiffness?: number;
    interface?: "position" | "velocity" | "effort";
  } = {},
) {
  const { world } = ros;
  const base = new RigidBody({ world, type: "static" });
  base.name = "base_link";
  const wheel = new RigidBody({
    world,
    mass: 1,
    centerOfMass: [0, 0, 0],
    diagonalInertia: [1, 1, 1],
  });
  wheel.name = "wheel_link";
  const joint = new RevoluteJoint({ body0: base, body1: wheel, axis: "Z" });
  joint.name = "wheel";
  const actuator = new JointActuator({
    ros,
    interface: command,
    stiffness,
    damping: command === "effort" ? undefined : 10,
    maxForce: 20,
  });
  joint.setDrive(actuator);
  const robot = new Robot({ ros });
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
