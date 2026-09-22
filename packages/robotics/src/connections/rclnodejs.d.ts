/** rclnodejs is compiled per ROS installation and installed optionally; this declares the slice this package uses. */
declare module "rclnodejs" {
  export interface RclQoS {}
  export interface RclPublisher {
    waitForAllAcked(timeout: bigint): boolean;
    publish(message: unknown): void;
  }
  export interface RclNode {
    createPublisher(
      type: string,
      topic: string,
      options?: { qos?: RclQoS },
    ): RclPublisher;
    getSubscriptionsInfoByTopic(topic: string, noDemangle: boolean): object[];
    createSubscription(
      type: string,
      topic: string,
      callback: (message: unknown) => void,
    ): unknown;
    destroySubscription(subscription: unknown): void;
    createService(
      type: string,
      name: string,
      /** The returned response, or a promise of it, is sent back. */
      handler: (request: unknown) => unknown,
    ): unknown;
    spin(): void;
    destroy(): void;
  }
  const rclnodejs: {
    QoS: {
      new (
        history: number,
        depth: number,
        reliability: number,
        durability: number,
      ): RclQoS;
      HistoryPolicy: { RMW_QOS_POLICY_HISTORY_KEEP_LAST: number };
      ReliabilityPolicy: { RMW_QOS_POLICY_RELIABILITY_RELIABLE: number };
      DurabilityPolicy: { RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL: number };
    };
    init(): Promise<void>;
    shutdown(): void;
    removeSignalHandlers(): void;
    Node: new (name: string) => RclNode;
  };
  export default rclnodejs;
}
