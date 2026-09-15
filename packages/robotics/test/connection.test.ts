import { afterEach, beforeEach, expect, it, vi } from "vitest";
import { RosbridgeConnection } from "../src/connection.js";

class WebSocketStub {
  static readonly CONNECTING = 0;
  static readonly OPEN = 1;
  static readonly CLOSING = 2;
  static readonly CLOSED = 3;
  static instances: WebSocketStub[] = [];
  readyState = WebSocketStub.CONNECTING;
  bufferedAmount = 0;
  readonly sent: unknown[] = [];
  onopen?: (event: Event) => void;
  onclose?: (event: Event) => void;
  onerror?: (event: Event) => void;
  onmessage?: (event: MessageEvent<string>) => void;
  constructor(readonly url: string) {
    WebSocketStub.instances.push(this);
  }
  open(): void {
    this.readyState = WebSocketStub.OPEN;
    this.onopen?.(new Event("open"));
  }
  receive(message: unknown): void {
    this.onmessage?.(
      new MessageEvent("message", { data: JSON.stringify(message) }),
    );
  }
  send(message: string): void {
    const parsed: unknown = JSON.parse(message);
    this.sent.push(parsed);
  }
  close(): void {
    this.readyState = WebSocketStub.CLOSED;
    this.onclose?.(new Event("close"));
  }
}
const connections: RosbridgeConnection[] = [];
beforeEach(() => {
  WebSocketStub.instances = [];
  vi.stubGlobal("WebSocket", WebSocketStub);
});
afterEach(() => {
  for (const connection of connections.splice(0)) connection.dispose();
  vi.unstubAllGlobals();
});
async function connect() {
  const connection = new RosbridgeConnection();
  connections.push(connection);
  const opening = connection.connect("ws://localhost:9090", 1000);
  await Promise.resolve();
  const socket = WebSocketStub.instances[0];
  if (!socket) throw new Error("WebSocket was not created");
  socket.open();
  await opening;
  return { connection, socket };
}

it("advertises once, decodes incoming messages, and unsubscribes cleanly", async () => {
  const { connection, socket } = await connect();
  connection.publish("/sensor", "std_msgs/msg/Float64", { data: 1 });
  connection.publish("/sensor", "std_msgs/msg/Float64", { data: 2 });
  expect(
    socket.sent.filter(
      (message) =>
        typeof message === "object" &&
        message !== null &&
        "op" in message &&
        message.op === "advertise",
    ),
  ).toHaveLength(1);
  const received = vi.fn();
  const release = connection.subscribe(
    "/command",
    "std_msgs/msg/Float64",
    received,
  );
  socket.receive({ op: "publish", topic: "/command", msg: { data: 3 } });
  expect(received).toHaveBeenCalledWith({ data: 3 });
  release();
  socket.receive({ op: "publish", topic: "/command", msg: { data: 4 } });
  expect(received).toHaveBeenCalledTimes(1);
});

it("bounds outgoing publication when the socket is congested without dropping subscription control", async () => {
  const { connection, socket } = await connect();
  socket.bufferedAmount = 5 * 1024 * 1024;
  connection.publish("/camera", "sensor_msgs/msg/CompressedImage", {
    data: "AA==",
  });
  expect(connection.dropped).toBe(1);
  connection.subscribe("/command", "std_msgs/msg/Float64", () => {});
  expect(socket.sent).toContainEqual(
    expect.objectContaining({ op: "subscribe", topic: "/command" }),
  );
  expect(socket.sent).not.toContainEqual(
    expect.objectContaining({ op: "publish" }),
  );
});

it("surfaces operation-specific rosbridge errors instead of remaining connected", async () => {
  const { connection, socket } = await connect();
  connection.publish("/sensor", "missing_msgs/msg/Missing", {});
  const advertise = socket.sent.find(
    (message) =>
      typeof message === "object" &&
      message !== null &&
      "op" in message &&
      message.op === "advertise",
  );
  if (
    typeof advertise !== "object" ||
    advertise === null ||
    !("id" in advertise)
  )
    throw new Error("Missing advertise ID");
  socket.receive({
    op: "status",
    id: advertise.id,
    level: "error",
    msg: "Unknown message type",
  });
  expect(connection.error?.message).toContain("Unknown message type");
  expect(connection.connected).toBe(false);
});

it("rejects publishing after closure and rejects conflicting topic types", async () => {
  const { connection, socket } = await connect();
  connection.publish("/sensor", "std_msgs/msg/Float64", { data: 1 });
  expect(() =>
    connection.publish("/sensor", "std_msgs/msg/String", { data: "1" }),
  ).toThrow("already uses");
  socket.close();
  expect(connection.connected).toBe(false);
  expect(() =>
    connection.publish("/sensor", "std_msgs/msg/Float64", { data: 2 }),
  ).toThrow("disconnected");
});
