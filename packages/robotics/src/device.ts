import { getROS2, type ROS2 } from "./runtime.js";

export interface DeviceLifecycle {
  beforeStep?(time: number, delta: number): void;
  afterStep?(time: number, delta: number): void;
  stop?(): void;
  dispose(): void;
}

export interface DeviceOptions {
  ros?: ROS2;
}

// The overload preserves the base constructor and instance types. The implementation
// only forwards base arguments, so it need not inspect or widen them to `any`.
export function withDevice<Args extends unknown[], Instance extends object>(
  Base: new (...args: Args) => Instance,
): new (
  ...options: [] extends Args
    ? [options?: DeviceOptions & { args?: Args }]
    : [options: DeviceOptions & { args: Args }]
) => Instance & DeviceLifecycle & { readonly ros: ROS2 };
export function withDevice(Base: new (...args: never[]) => object) {
  return class extends Base {
    readonly ros: ROS2;
    #unregister: () => void;

    constructor(options: DeviceOptions & { args?: never[] } = {}) {
      super(...(options.args ?? []));
      this.ros = options.ros ?? getROS2();
      this.#unregister = this.ros.register(this);
    }

    dispose(): void {
      this.#unregister();
    }
  };
}

export class Device extends withDevice(class {}) {}
