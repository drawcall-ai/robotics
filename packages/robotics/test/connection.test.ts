import { afterEach, beforeEach, expect, it, vi } from "vitest";
import { RosbridgeConnection } from "../src/connection.js";

class WebSocketStub {
  static readonly CONNECTING = 0;
  static readonly OPEN = 1;
  static readonly CLOSING = 2;
  static readonly CLOSED = 3;
  static instances: WebSocketStub[] = [];
  readyState = WebSocketStub.CONNECTING;
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

it("rejects publishing after closure", async () => {
  const { connection, socket } = await connect();
  connection.publish("/sensor", "std_msgs/msg/Float64", { data: 1 });
  socket.close();
  expect(connection.connected).toBe(false);
  expect(() =>
    connection.publish("/sensor", "std_msgs/msg/Float64", { data: 2 }),
  ).toThrow("disconnected");
});

it("closes the socket and releases other topics when a control message fails", async () => {
  const { connection, socket } = await connect();
  connection.subscribe("/command", "std_msgs/msg/Float64", () => {});
  connection.publish("/sensor", "std_msgs/msg/Float64", { data: 1 });
  const send = vi.spyOn(socket, "send");
  send.mockImplementationOnce(() => {
    throw new Error("Socket send failed");
  });
  expect(() => connection.dispose()).toThrow(
    "Failed to dispose ROS connection",
  );
  expect(socket.readyState).toBe(WebSocketStub.CLOSED);
  expect(socket.sent).toContainEqual(
    expect.objectContaining({ op: "unadvertise", topic: "/sensor" }),
  );
  expect(connection.connected).toBe(false);
  expect(() => connection.dispose()).not.toThrow();
});

it("encodes byte arrays as base64 for the rosbridge wire format", async () => {
  const { connection, socket } = await connect();
  connection.publish("/camera", "sensor_msgs/msg/CompressedImage", {
    format: "jpeg",
    data: new Uint8Array([255, 216, 255, 217]),
  });
  expect(socket.sent).toContainEqual(
    expect.objectContaining({
      op: "publish",
      msg: { format: "jpeg", data: "/9j/2Q==" },
    }),
  );
});
