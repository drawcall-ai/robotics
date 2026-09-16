import type { RclNode, RclPublisher } from "rclnodejs";
import type { Connection } from "../runtime.js";

/** A native ROS 2 node through rclnodejs: CDR over DDS, no bridge process. Node.js only. */
export class RclConnection implements Connection {
  private readonly publishers = new Map<string, RclPublisher>();
  private constructor(
    private readonly resources: DisposableStack,
    private readonly node: RclNode,
  ) {}
  static async create(name: string): Promise<RclConnection> {
    const { default: rcl } = await import("rclnodejs");
    await rcl.init();
    using resources = new DisposableStack();
    resources.defer(() => rcl.shutdown());
    const node = new rcl.Node(name);
    resources.defer(() => node.destroy());
    node.spin();
    return new RclConnection(resources.move(), node);
  }

  get connected(): boolean {
    return !this.resources.disposed;
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
  /** Answers `name` with the value `handler` returns for each request. */
  service(
    type: string,
    name: string,
    handler: (request: unknown) => unknown,
  ): void {
    this.node.createService(type, name, (request, response) =>
      response.send(handler(request)),
    );
  }
  dispose(): void {
    this.resources.dispose();
  }
}
