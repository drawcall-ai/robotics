import { spawn, type ChildProcess } from "node:child_process";
import { setTimeout } from "node:timers/promises";
import { resolve } from "node:path";
import {
  EVALUATION_TOPIC,
  EVALUATION_TYPE,
  SCORE_TOPIC,
  SCORE_TYPE,
  type Evaluator,
} from "../evaluation.js";
import type { RclConnection } from "../connections/rcl.js";
import { stamp } from "../messages.js";
import { readResult } from "./recording.js";

/** Owns rosbag through discovery, final delivery, graceful close and on-disk verification. */
export class Recorder {
  private stopping = false;
  private ended = false;
  private failure?: Error;
  private readonly child: ChildProcess;
  private readonly closed: Promise<void>;
  readonly path: string;
  constructor(path: string) {
    this.path = resolve(path);
    this.child = spawn(
      "ros2",
      [
        "bag",
        "record",
        "--storage",
        "mcap",
        "--output",
        this.path,
        "--all",
        "--use-sim-time",
        "--exclude-regex",
        "^/(clock|bond|evaluation|marker)$|_costmap/.+_(layer|raw|updates)",
      ],
      { stdio: ["ignore", "inherit", "inherit"], detached: true },
    );
    this.closed = new Promise((resolve) => {
      this.child.once("error", (error) => {
        this.failure = error;
        this.ended = true;
        resolve();
      });
      this.child.once("close", (code, signal) => {
        this.ended = true;
        if (!this.stopping || (code !== 0 && signal !== "SIGINT"))
          this.failure ??= new Error(
            `Recorder exited unexpectedly: ${code ?? signal}`,
          );
        resolve();
      });
    });
  }
  check(): void {
    if (this.failure) throw this.failure;
  }
  async ready(
    connection: RclConnection,
    time: number,
    signal: AbortSignal,
    retainedTopics: string[],
  ): Promise<void> {
    connection.advertise(EVALUATION_TOPIC, EVALUATION_TYPE, { retain: true });
    connection.advertise(SCORE_TOPIC, SCORE_TYPE, {
      retain: true,
    });
    await this.wait(
      () => {
        connection.publish("/clock", "rosgraph_msgs/msg/Clock", {
          clock: stamp(time),
        });
        return (
          connection.hasSubscriber(EVALUATION_TOPIC, "rosbag2_recorder") &&
          connection.hasSubscriber(SCORE_TOPIC, "rosbag2_recorder") &&
          retainedTopics.every(
            (topic) =>
              connection.hasSubscriber(topic, "rosbag2_recorder") &&
              connection.acknowledged(topic),
          )
        );
      },
      "Recorder did not acknowledge scene metadata and evaluation",
      signal,
    );
  }
  async finish(
    connection: RclConnection,
    evaluation: Evaluator,
  ): Promise<void> {
    await this.wait(
      () =>
        connection.acknowledged(EVALUATION_TOPIC) &&
        connection.acknowledged(SCORE_TOPIC),
      "Evaluation was not acknowledged",
    );
    await this.stop();
    const result = await readResult(this.path);
    if (JSON.stringify(result) !== JSON.stringify(evaluation.result))
      throw new Error(
        "Recorded evaluation differs from the simulator's final result",
      );
    console.error(`Recorded ${result.status}: ${this.path}`);
  }
  private async wait(
    ready: () => boolean,
    message: string,
    signal?: AbortSignal,
  ): Promise<void> {
    const deadline = performance.now() + 15000;
    while (!ready()) {
      if (signal?.aborted) return;
      this.check();
      if (performance.now() >= deadline) throw new Error(message);
      await setTimeout(25);
    }
    this.check();
  }
  async stop(): Promise<void> {
    if (!this.stopping && !this.ended) {
      this.stopping = true;
      this.child.kill("SIGINT");
    }
    const timeout = globalThis.setTimeout(() => {
      this.failure = new Error("Recorder did not close within 15 seconds");
      this.child.kill("SIGKILL");
    }, 15000);
    try {
      await this.closed;
      this.check();
    } finally {
      clearTimeout(timeout);
    }
  }
}
