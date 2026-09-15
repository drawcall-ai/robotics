import { disposeAll } from "./cleanup.js";
import type { JointMotor } from "@drawcall/physics";
import { getROS2, type ROS2 } from "./runtime.js";
import { Device } from "./device.js";

const bindings = new WeakSet<JointMotor>();
export interface MotorBindingOptions {
  motor: JointMotor;
  name: string;
  mode: "position" | "velocity";
  timeout?: number;
  ros?: ROS2;
}
export class MotorBinding extends Device {
  readonly motor: JointMotor;
  private readonly mode: "position" | "velocity";
  private readonly timeout: number;
  private command?: { value: number; time: number; wall: number };
  private readonly unsubscribe: () => void;
  private disposed = false;
  constructor(options: MotorBindingOptions) {
    const ros = options.ros ?? getROS2();
    const motor = options.motor;
    const mode = options.mode;
    const timeout = options.timeout ?? 0.5;
    if (motor.options.joint.world !== ros.world)
      throw new Error("Motor and ROS must use the same physics world");
    if (motor.disposed || bindings.has(motor))
      throw new Error("Motor is disposed or already bound");
    if (!Number.isFinite(timeout) || timeout <= 0)
      throw new Error("Command timeout must be positive");
    if (mode === "velocity" && (motor.options.stiffness ?? 0) !== 0)
      throw new Error("Velocity bindings require a motor with zero stiffness");
    super({ ros });
    this.motor = motor;
    this.mode = mode;
    this.timeout = timeout;
    try {
      this.unsubscribe = this.ros.subscribe(
        `${options.name}/command`,
        "std_msgs/msg/Float64",
        (message) => {
          if (
            typeof message !== "object" ||
            message === null ||
            !("data" in message) ||
            typeof message.data !== "number" ||
            !Number.isFinite(message.data)
          )
            throw new Error(`Invalid ${options.name} motor command`);
          this.command = {
            value: message.data,
            time: this.ros.world.time,
            wall: performance.now() / 1000,
          };
        },
      );
    } catch (error) {
      super.dispose();
      throw error;
    }
    bindings.add(this.motor);
    this.motor.setEnabled(false);
  }
  beforeStep(time: number): void {
    const command = this.command;
    if (
      !command ||
      time - command.time > this.timeout ||
      performance.now() / 1000 - command.wall > this.timeout
    ) {
      this.stop();
      return;
    }
    const position = this.mode === "position" ? command.value : 0;
    const velocity = this.mode === "velocity" ? command.value : 0;
    if (
      this.motor.target?.position !== position ||
      this.motor.target.velocity !== velocity
    )
      this.motor.setTarget({ position, velocity });
    if (!this.motor.enabled) this.motor.setEnabled(true);
  }
  stop(): void {
    this.command = undefined;
    if (!this.motor.disposed && this.motor.enabled)
      this.motor.setEnabled(false);
  }
  dispose(): void {
    if (this.disposed) return;
    this.disposed = true;
    disposeAll(
      [
        () => this.stop(),
        this.unsubscribe,
        () => super.dispose(),
        () => {
          bindings.delete(this.motor);
        },
      ],
      "Failed to dispose motor binding",
    );
  }
}
