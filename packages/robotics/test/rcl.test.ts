import { beforeEach, expect, it, vi } from "vitest";
const native = vi.hoisted(() => ({
  init: vi.fn(),
  removeSignalHandlers: vi.fn(),
  shutdown: vi.fn(),
  spin: vi.fn(),
  destroy: vi.fn(),
}));
vi.mock("rclnodejs", () => ({
  default: {
    QoS: class {
      static HistoryPolicy = { RMW_QOS_POLICY_HISTORY_KEEP_LAST: 1 };
      static ReliabilityPolicy = { RMW_QOS_POLICY_RELIABILITY_RELIABLE: 1 };
      static DurabilityPolicy = {
        RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL: 1,
      };
    },
    init: native.init,
    removeSignalHandlers: native.removeSignalHandlers,
    shutdown: native.shutdown,
    Node: class {
      spin = native.spin;
      destroy = native.destroy;
    },
  },
}));
import { RclConnection } from "../src/connections/rcl.js";
beforeEach(() => vi.clearAllMocks());
it("destroys the node and shuts ROS down if spinning fails", async () => {
  native.spin.mockImplementationOnce(() => {
    throw new Error("spin failed");
  });
  await expect(RclConnection.create("test")).rejects.toThrow("spin failed");
  expect(native.destroy).toHaveBeenCalledOnce();
  expect(native.shutdown).toHaveBeenCalledOnce();
});
it("shuts ROS down even when node destruction fails", async () => {
  const connection = await RclConnection.create("test");
  native.destroy.mockImplementationOnce(() => {
    throw new Error("destroy failed");
  });
  expect(() => connection.dispose()).toThrow();
  expect(native.shutdown).toHaveBeenCalledOnce();
  expect(connection.connected).toBe(false);
  expect(() => connection.dispose()).not.toThrow();
});

it("preserves spinning and destruction failures and still shuts ROS down", async () => {
  const spin = new Error("spin failed");
  const destroy = new Error("destroy failed");
  native.spin.mockImplementationOnce(() => {
    throw spin;
  });
  native.destroy.mockImplementationOnce(() => {
    throw destroy;
  });
  await expect(RclConnection.create("test")).rejects.toMatchObject({
    error: destroy,
    suppressed: spin,
  });
  expect(native.shutdown).toHaveBeenCalledOnce();
});

it("lets the runner finalize recordings before native shutdown", async () => {
  const connection = await RclConnection.create("test");
  expect(native.removeSignalHandlers).toHaveBeenCalledOnce();
  expect(native.shutdown).not.toHaveBeenCalled();
  connection.dispose();
  expect(native.shutdown).toHaveBeenCalledOnce();
});
