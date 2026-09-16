/** rclnodejs is compiled per ROS installation and installed optionally; this declares the slice this package uses. */
declare module "rclnodejs" {
  export interface RclPublisher {
    publish(message: unknown): void;
  }
  export interface RclNode {
    createPublisher(type: string, topic: string): RclPublisher;
    createSubscription(
      type: string,
      topic: string,
      callback: (message: unknown) => void,
    ): unknown;
    destroySubscription(subscription: unknown): void;
    createService(
      type: string,
      name: string,
      handler: (
        request: unknown,
        response: { send(value: unknown): void },
      ) => void,
    ): unknown;
    spin(): void;
    destroy(): void;
  }
  const rclnodejs: {
    init(): Promise<void>;
    shutdown(): void;
    Node: new (name: string) => RclNode;
  };
  export default rclnodejs;
}
