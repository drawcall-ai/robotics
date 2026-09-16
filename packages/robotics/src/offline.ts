import type { Connection } from "./connection.js";

/** A connection to nothing, for runtimes that only build a scene: URDF export and tests. Publishing on it is a fault. */
export class OfflineConnection implements Connection {
  readonly connected = true;
  publish(name: string): void {
    throw new Error(`Cannot publish ${name}: this runtime is offline`);
  }
  subscribe(): () => void {
    return () => undefined;
  }
  dispose(): void {}
}
