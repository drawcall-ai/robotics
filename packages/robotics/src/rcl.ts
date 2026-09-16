import type { RclNode, RclPublisher } from "rclnodejs";
import type { Connection } from "./connection.js";

/** A native ROS 2 node through rclnodejs: CDR over DDS, no bridge process. Node.js only. */
export class RclConnection implements Connection {
  private readonly publishers = new Map<string, RclPublisher>();
  private closed = false;
  private constructor(
    private readonly shutdown: () => void,
    readonly node: RclNode,
  ) {}
  static async create(name: string): Promise<RclConnection> {
    const { default: rcl } = await import("rclnodejs");
    await rcl.init();
    const node = new rcl.Node(name);
    node.spin();
    return new RclConnection(() => rcl.shutdown(), node);
  }
  get connected(): boolean {
    return !this.closed;
  }
  publish(name: string, type: string, message: unknown): void {
    let publisher = this.publishers.get(name);
    if (!publisher) {
      publisher = this.node.createPublisher(type, name);
      this.publishers.set(name, publisher);
    }
    publisher.publish(message);
  }
  subscribe(
    name: string,
    type: string,
    receive: (message: unknown) => void,
  ): () => void {
    const subscription = this.node.createSubscription(type, name, receive);
    return () => this.node.destroySubscription(subscription);
  }
  dispose(): void {
    if (this.closed) return;
    this.closed = true;
    this.node.destroy();
    this.shutdown();
  }
}
