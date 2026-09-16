import type { RigidBody } from "@drawcall/physics";
import { Quaternion, Vector3 } from "three";
import type { RclNode } from "rclnodejs";
import { rollPitchYaw, rosPosition, rosRotation } from "../coordinates.js";
import type { ROS2 } from "../runtime.js";

/** Pause, resume, step and reset services plus a ground-truth state topic, the way other simulators expose them. */
export class SimulationControl {
  paused = false;
  private pendingSteps = 0;
  constructor(
    private readonly ros: ROS2,
    node: RclNode,
    private readonly base: RigidBody,
  ) {
    const trigger = (name: string, run: () => string) =>
      node.createService(
        "std_srvs/srv/Trigger",
        `/sim/${name}`,
        (_request, response) => {
          response.send({ success: true, message: run() });
        },
      );
    trigger("pause", () => {
      this.paused = true;
      return "paused";
    });
    trigger("resume", () => {
      this.paused = false;
      return "running";
    });
    trigger("step", () => {
      this.pendingSteps++;
      return `stepping ${this.ros.world.fixedDelta} s`;
    });
    trigger("reset", () => {
      this.ros.world.reset();
      return "world reset; the clock restarts at zero";
    });
  }
  /** Advances the world by `delta` seconds of wall time unless paused; single steps run while paused. */
  advance(delta: number): void {
    if (!this.paused) {
      this.ros.world.update(delta);
      return;
    }
    while (this.pendingSteps > 0) {
      this.pendingSteps--;
      this.ros.world.update(this.ros.world.fixedDelta);
    }
  }
  /** Publishes ground truth for episode logic; navigation must never subscribe to it. Runs on wall time so a paused world still reports. */
  publishState(): void {
    this.base.updateWorldMatrix(true, false);
    const position = rosPosition(this.base.getWorldPosition(new Vector3()));
    const [, , yaw] = rollPitchYaw(
      rosRotation(this.base.getWorldQuaternion(new Quaternion())),
    );
    this.ros.publish("/sim/state", "std_msgs/msg/String", {
      data: JSON.stringify({
        time: this.ros.world.time,
        paused: this.paused,
        base: { x: position.x, y: position.y, yaw },
      }),
    });
  }
}
