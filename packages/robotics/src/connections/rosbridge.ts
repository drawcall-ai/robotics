import { Ros, Topic, WebSocketTransportFactory } from "roslib";
import { runAll } from "../cleanup.js";
import { base64 } from "../messages.js";
import type { Connection, PublishOptions } from "../runtime.js";

export class RosbridgeConnection implements Connection {
  private readonly ros: Ros;
  private readonly topics = new Map<string, Topic<unknown>>();
  error?: Error;
  constructor() {
    this.ros = new Ros({
      transportFactory: async (url) => {
        const transport = await WebSocketTransportFactory(url);
        transport.on("message", (message) => {
          if (
            message.op === "status" &&
            "level" in message &&
            message.level === "error"
          )
            this.error = new Error(
              `ROS bridge: ${"msg" in message ? String(message.msg) : "protocol error"}`,
            );
        });
        return transport;
      },
    });
    this.ros.on("error", () => {
      this.error = new Error("ROS bridge connection failed");
    });
  }
  get connected(): boolean {
    return this.ros.isConnected && !this.error;
  }
  async connect(url: string, timeout: number): Promise<void> {
    const setup = this.ros.connect(url);
    await new Promise<void>((resolve, reject) => {
      const finish = (error?: Error) => {
        clearTimeout(timer);
        this.ros.off("connection", opened);
        this.ros.off("error", failed);
        this.ros.off("close", failed);
        if (error) reject(error);
        else resolve();
      };
      const opened = () => finish();
      const failed = () =>
        finish(new Error(`Unable to connect to ROS bridge at ${url}`));
      const timer = setTimeout(
        () => finish(new Error(`ROS connection timed out after ${timeout} ms`)),
        timeout,
      );
      this.ros.once("connection", opened);
      this.ros.once("error", failed);
      this.ros.once("close", failed);
      void setup.catch((error: unknown) =>
        finish(error instanceof Error ? error : new Error(String(error))),
      );
    }).catch(async (cause: unknown) => {
      // A timed-out transport factory can still acquire a socket. Join it before closing.
      const [result] = await Promise.allSettled([setup]);
      if (result.status === "rejected" && result.reason !== cause)
        cause = new AggregateError(
          [cause, result.reason],
          "ROS bridge setup failed",
          { cause },
        );
      try {
        this.dispose();
      } catch (error) {
        throw new AggregateError(
          [cause, error],
          "ROS bridge connection and cleanup failed",
          { cause },
        );
      }
      throw cause;
    });
  }
  private topic(
    name: string,
    type: string,
    options?: PublishOptions,
  ): Topic<unknown> {
    const current = this.topics.get(name);
    if (current) return current;
    const topic = new Topic({
      ros: this.ros,
      name,
      messageType: type,
      latch: options?.retain ?? false,
      queue_size: 1,
      queue_length: 1,
      reconnect_on_close: false,
    });
    this.topics.set(name, topic);
    return topic;
  }
  publish(
    name: string,
    type: string,
    message: unknown,
    options?: PublishOptions,
  ): void {
    if (!this.connected) throw new Error("ROS bridge is disconnected");
    this.topic(name, type, options).publish(base64Bytes(message));
  }
  /** rosbridge does not report subscribers, so everything is published. */
  subscribed(): boolean {
    return true;
  }
  subscribe(
    name: string,
    type: string,
    receive: (message: unknown) => void,
  ): () => void {
    if (!this.connected) throw new Error("ROS bridge is disconnected");
    const topic = this.topic(name, type);
    topic.subscribe(receive);
    return () => topic.unsubscribe(receive);
  }
  dispose(): void {
    const releases = this.ros.isConnected
      ? [...this.topics.values()].flatMap((topic) => [
          () => topic.unsubscribe(),
          () => topic.unadvertise(),
        ])
      : [];
    releases.push(() => this.ros.close());
    this.topics.clear();
    runAll(releases, "Failed to dispose ROS connection");
  }
}

/** rosbridge's JSON protocol carries `uint8[]` fields as base64 text. */
function base64Bytes(value: unknown): unknown {
  if (value instanceof Uint8Array) return base64(value);
  if (Array.isArray(value)) return value.map(base64Bytes);
  if (typeof value === "object" && value !== null)
    return Object.fromEntries(
      Object.entries(value).map(([key, field]) => [key, base64Bytes(field)]),
    );
  return value;
}
