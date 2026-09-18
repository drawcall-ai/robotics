import type { RclNode, RclPublisher, RclQoS } from "rclnodejs";
import type { Connection, PublishOptions } from "../runtime.js";

/** A native ROS 2 node through rclnodejs: CDR over DDS, no bridge process. Node.js only. */
export class RclConnection implements Connection {
  private readonly publishers = new Map<string, RclPublisher>();
  private constructor(
    private readonly resources: DisposableStack,
    private readonly node: RclNode,
    private readonly retained: RclQoS,
  ) {}
  static async create(name: string): Promise<RclConnection> {
    const { default: rcl } = await import("rclnodejs");
    // The runner publishes and records its terminal result before disposing ROS.
    rcl.removeSignalHandlers();
    await rcl.init();
    using resources = new DisposableStack();
    resources.defer(() => rcl.shutdown());
    const node = new rcl.Node(name);
    resources.defer(() => node.destroy());
    node.spin();
    const retained = new rcl.QoS(
      rcl.QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
      1,
      rcl.QoS.ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      rcl.QoS.DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    );
    return new RclConnection(resources.move(), node, retained);
  }

  get connected(): boolean {
    return !this.resources.disposed;
  }
  advertise(
    name: string,
    type: string,
    options?: PublishOptions,
  ): RclPublisher {
    let publisher = this.publishers.get(name);
    if (!publisher) {
      publisher = this.node.createPublisher(
        type,
        name,
        options?.retain ? { qos: this.retained } : undefined,
      );
      this.publishers.set(name, publisher);
    }
    return publisher;
  }
  publish(
    name: string,
    type: string,
    message: unknown,
    options?: PublishOptions,
  ): void {
    this.advertise(name, type, options).publish(message);
  }
  hasSubscriber(topic: string, node: string): boolean {
    return this.node
      .getSubscriptionsInfoByTopic(topic, false)
      .some((info) => "node_name" in info && info.node_name === node);
  }
  acknowledged(topic: string): boolean {
    const publisher = this.publishers.get(topic);
    if (!publisher) throw new Error(`No publisher for ${topic}`);
    return publisher.waitForAllAcked(0n);
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
