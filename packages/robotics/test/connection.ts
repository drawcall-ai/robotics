import type { Connection } from "../src/connection.js";

/** Records publications and hands received messages to subscribers. */
export class ConnectionStub implements Connection {
  connected = true;
  readonly messages: { name: string; type: string; message: unknown }[] = [];
  readonly receivers = new Map<string, (message: unknown) => void>();
  publish(name: string, type: string, message: unknown): void {
    this.messages.push({ name, type, message });
  }
  subscribe(
    name: string,
    _type: string,
    receive: (message: unknown) => void,
  ): () => void {
    this.receivers.set(name, receive);
    return () => {
      this.receivers.delete(name);
    };
  }
  receive(name: string, message: unknown): void {
    const receive = this.receivers.get(name);
    if (!receive) throw new Error(`No subscription for ${name}`);
    receive(message);
  }
  dispose(): void {
    this.connected = false;
    this.receivers.clear();
  }
}
