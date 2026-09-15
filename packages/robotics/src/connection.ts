import { disposeAll } from "./cleanup.js";
import { AbstractTransport, Ros, Topic, type RosbridgeMessage } from "roslib";

export interface Connection {
  readonly connected: boolean;
  readonly error?: Error;
  readonly dropped: number;
  publish(name: string, type: string, message: unknown): void;
  subscribe(
    name: string,
    type: string,
    receive: (message: unknown) => void,
  ): () => void;
  dispose(): void;
}

class Socket extends AbstractTransport {
  readonly socket: WebSocket;
  dropped = 0;
  constructor(url: string) {
    super();
    this.socket = new WebSocket(url);
    this.socket.onopen = (event) => this.emit("open", event);
    this.socket.onclose = (event) => this.emit("close", event);
    this.socket.onerror = (event) => this.emit("error", event);
    this.socket.onmessage = (event) => this.handleRawMessage(event.data);
  }
  send(message: RosbridgeMessage): void {
    if (
      message.op === "publish" &&
      this.socket.bufferedAmount > 4 * 1024 * 1024
    ) {
      this.dropped++;
      return;
    }
    this.socket.send(JSON.stringify(message));
  }
  close(): void {
    this.socket.close();
  }
  isConnecting(): boolean {
    return this.socket.readyState === WebSocket.CONNECTING;
  }
  isOpen(): boolean {
    return this.socket.readyState === WebSocket.OPEN;
  }
  isClosing(): boolean {
    return this.socket.readyState === WebSocket.CLOSING;
  }
  isClosed(): boolean {
    return this.socket.readyState === WebSocket.CLOSED;
  }
}

export class RosbridgeConnection implements Connection {
  private readonly ros: Ros;
  private socket?: Socket;
  private readonly topics = new Map<string, Topic<unknown>>();
  error?: Error;
  constructor() {
    this.ros = new Ros({
      transportFactory: async (url) => {
        this.socket = new Socket(url);
        this.socket.on("message", (message) => {
          if (
            message.op === "status" &&
            "level" in message &&
            message.level === "error"
          )
            this.error = new Error(
              `ROS bridge: ${"msg" in message ? String(message.msg) : "protocol error"}`,
            );
        });
        return this.socket;
      },
    });
    this.ros.on("error", () => {
      this.error = new Error("ROS bridge connection failed");
    });
  }
  get connected(): boolean {
    return this.ros.isConnected && !this.error;
  }
  get dropped(): number {
    return this.socket?.dropped ?? 0;
  }
  async connect(url: string, timeout: number): Promise<void> {
    await new Promise<void>((resolve, reject) => {
      const finish = (error?: Error) => {
        clearTimeout(timer);
        this.ros.off("connection", opened);
        this.ros.off("error", failed);
        this.ros.off("close", failed);
        if (error) {
          this.dispose();
          reject(error);
        } else resolve();
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
      void this.ros
        .connect(url)
        .catch((error: unknown) =>
          finish(error instanceof Error ? error : new Error(String(error))),
        );
    });
  }
  private topic(name: string, type: string): Topic<unknown> {
    const current = this.topics.get(name);
    if (current) {
      if (current.messageType !== type)
        throw new Error(`Topic ${name} already uses ${current.messageType}`);
      return current;
    }
    const topic = new Topic({
      ros: this.ros,
      name,
      messageType: type,
      queue_size: 1,
      queue_length: 1,
      reconnect_on_close: false,
    });
    this.topics.set(name, topic);
    return topic;
  }
  publish(name: string, type: string, message: unknown): void {
    if (!this.connected) throw new Error("ROS bridge is disconnected");
    this.topic(name, type).publish(message);
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
    disposeAll(releases, "Failed to dispose ROS connection");
  }
}
