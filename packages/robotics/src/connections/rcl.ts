import type { RclClient, RclNode, RclPublisher, RclQoS } from "rclnodejs";
import type { Connection, PublishOptions } from "../runtime.js";

/** A native ROS 2 node through rclnodejs: CDR over DDS, no bridge process. Node.js only. */
export class RclConnection implements Connection {
  private readonly publishers = new Map<string, RclPublisher>();
  /** Kept for the connection's lifetime: a new client must be discovered by the server again. */
  private readonly clients = new Map<string, RclClient>();
  private constructor(
    private readonly resources: DisposableStack,
    private readonly node: RclNode,
    private readonly retained: RclQoS,
  ) {}
  /** A node in its own context, in `domain` if given, else in ROS_DOMAIN_ID's. */
  static async create(name: string, domain?: number): Promise<RclConnection> {
    const { default: rcl } = await import("rclnodejs");
    // The runner publishes and records its terminal result before disposing ROS.
    rcl.removeSignalHandlers();
    const context = new rcl.Context(
      domain === undefined ? undefined : BigInt(domain),
    );
    // Not process.argv: a policy command after `--` may carry its own --ros-args.
    await rcl.init(context, []);
    using resources = new DisposableStack();
    resources.defer(() => rcl.shutdown(context));
    const node = new rcl.Node(name, "", context);
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
  subscribed(name: string, type: string): boolean {
    return this.advertise(name, type).subscriptionCount > 0;
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
  /** Answers `name` with the response `handler` returns, or resolves, for each request. */
  service(
    type: string,
    name: string,
    handler: (request: unknown) => unknown,
  ): void {
    this.node.createService(type, name, handler);
  }
  /** Whether a server of service `name` appears within `timeout` milliseconds. */
  async available(
    type: string,
    name: string,
    timeout: number,
  ): Promise<boolean> {
    return await this.client(type, name).waitForService(timeout);
  }
  /** Calls a service, waiting up to `timeout` milliseconds for its server and again for its response. */
  async call(
    type: string,
    name: string,
    request: unknown,
    timeout = 10_000,
  ): Promise<unknown> {
    const client = this.client(type, name);
    if (!(await client.waitForService(timeout)))
      throw new Error(`Service ${name} is not available`);
    return await client.sendRequestAsync(request, { timeout });
  }
  private client(type: string, name: string): RclClient {
    let client = this.clients.get(name);
    if (!client) {
      client = this.node.createClient(type, name);
      this.clients.set(name, client);
    }
    return client;
  }
  /** Logs through the node, and so to /rosout. */
  log(message: string): void {
    this.node.getLogger().info(message);
  }
  dispose(): void {
    this.resources.dispose();
  }
}
