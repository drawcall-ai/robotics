import { getROS2, type ROS2 } from "./runtime.js";
import { identifier, stamp } from "./messages.js";

export const EVALUATION_TOPIC = "/sim/evaluation";
export const EVALUATION_TYPE = "std_msgs/msg/String";
export const SCORE_TOPIC = "/sim/score";
export const SCORE_TYPE = "std_msgs/msg/Float64";
export const EVALUATION_STATUSES = [
  "running",
  "success",
  "failure",
  "timeout",
  "aborted",
  "error",
] as const;
export type EvaluationStatus = (typeof EVALUATION_STATUSES)[number];
export interface EvaluationState {
  readonly stamp: { readonly sec: number; readonly nanosec: number };
  readonly elapsed: number;
  readonly status: EvaluationStatus;
  readonly reason: string;
  readonly metrics: Readonly<Record<string, number>>;
}
export interface EvaluationContext {
  readonly elapsed: number;
  readonly status: EvaluationStatus;
  readonly metrics: Readonly<Record<string, number>>;
}
export interface EvaluatorOptions {
  timeout: number;
  metrics?: Record<string, () => number>;
  success: Record<string, (context: EvaluationContext) => boolean>;
  failure?: Record<string, (context: EvaluationContext) => boolean>;
  score: (context: EvaluationContext) => number;
  ros?: ROS2;
  /**
   * Publishes /sim/evaluation and /sim/score privately: recorded, never seen by the robot's
   * software. The run's result.json gets the final result either way.
   */
  private?: boolean;
}

/** One episode. The runner starts it after ROS and the recorder are ready. */
export class Evaluator {
  readonly ros: ROS2;
  /** Whether the evaluation is published privately. */
  readonly private: boolean;
  private readonly unregister: () => void;
  private started?: number;
  private state?: EvaluationState;
  private published?: EvaluationState;
  private disposed = false;
  constructor(private readonly options: EvaluatorOptions) {
    if (!Number.isFinite(options.timeout) || options.timeout <= 0)
      throw new Error("Evaluation timeout must be positive and finite");
    for (const name of Object.keys(options.metrics ?? {})) {
      identifier("metric", name);
      if (name === "score")
        throw new Error("score is reserved for the score callback");
    }
    for (const name of [
      ...Object.keys(options.success),
      ...Object.keys(options.failure ?? {}),
    ])
      identifier("condition", name);
    this.ros = options.ros ?? getROS2();
    this.private = options.private ?? false;
    if (this.ros.find(Evaluator).length)
      throw new Error("Only one evaluator is allowed per runtime");
    this.unregister = this.ros.register(this);
  }
  get active(): boolean {
    return !this.disposed && this.state !== undefined && !this.finished;
  }
  get finished(): boolean {
    return this.state !== undefined && this.state.status !== "running";
  }
  get result(): EvaluationState | undefined {
    return this.state;
  }
  start(): void {
    if (this.disposed || this.started !== undefined)
      throw new Error(
        "An evaluator can only start once; create a new episode to restart",
      );
    this.started = this.ros.world.time;
    this.sample(false);
  }
  /** Called after all synchronous devices have sampled the completed physics step. */
  evaluate(): void {
    if (this.active) this.sample(true);
  }
  end(status: "aborted" | "error", reason: string): void {
    const state = this.state;
    if (this.disposed || state?.status !== "running") return;
    // Preserve the last valid measurements, even if a metric callback caused the error.
    this.state = Object.freeze({
      ...state,
      stamp: Object.freeze(stamp(this.ros.world.time)),
      elapsed: this.elapsed,
      status,
      reason,
    });
    this.publish(this.state);
  }
  private get elapsed(): number {
    if (this.started === undefined)
      throw new Error("Evaluation has not started");
    return this.ros.world.time - this.started;
  }
  private sample(check: boolean): void {
    let status: EvaluationStatus = "running";
    let reason = "";
    const elapsed = this.elapsed;
    const metrics = Object.fromEntries(
      Object.entries(this.options.metrics ?? {}).map(([name, read]) => [
        name,
        finite(name, read()),
      ]),
    );
    const context = {
      elapsed,
      status,
      metrics: Object.freeze(metrics),
    };
    if (check) {
      const failure = Object.entries(this.options.failure ?? {}).find(
        ([, test]) => test(context),
      );
      const success = failure
        ? undefined
        : Object.entries(this.options.success).find(([, test]) =>
            test(context),
          );
      if (failure) {
        status = "failure";
        reason = failure[0];
      } else if (success) {
        status = "success";
        reason = success[0];
      } else if (elapsed >= this.options.timeout) {
        status = "timeout";
        reason = "timeLimit";
      }
    }
    const score = finite("score", this.options.score({ ...context, status }));
    const state = Object.freeze({
      stamp: Object.freeze(stamp(this.ros.world.time)),
      elapsed,
      status,
      reason,
      metrics: Object.freeze({ ...metrics, score }),
    });
    this.state = state;
    const previous = this.published;
    const changed =
      !previous ||
      Object.entries(state.metrics).some(
        ([name, value]) => value !== previous.metrics[name],
      );
    if (
      !previous ||
      status !== "running" ||
      (changed && elapsed - previous.elapsed >= 0.1 - 1e-9)
    )
      this.publish(state);
  }
  private publish(state: EvaluationState): void {
    // Runtime faults stop devices, but the final diagnostic must still reach ROS. Private
    // messages go to the recording, which a fault does not disconnect.
    const publisher =
      state.status === "error" && this.ros.error && !this.private
        ? this.ros.connection
        : this.ros;
    const options = { retain: true, private: this.private };
    publisher.publish(
      SCORE_TOPIC,
      SCORE_TYPE,
      { data: state.metrics.score },
      options,
    );
    publisher.publish(
      EVALUATION_TOPIC,
      EVALUATION_TYPE,
      { data: JSON.stringify(state) },
      options,
    );
    this.published = state;
  }

  dispose(): void {
    this.disposed = true;
    this.unregister();
  }
}
function finite(name: string, value: number): number {
  if (!Number.isFinite(value)) throw new Error(`Metric ${name} must be finite`);
  return value;
}
