import { afterEach, beforeEach, expect, it, vi } from "vitest";

const mocks = vi.hoisted(() => ({
  connection: { dispose: vi.fn(), subscribe: vi.fn(), service: vi.fn() },
  createConnection: vi.fn(),
  world: { dispose: vi.fn(), fixedDelta: 1 / 120, update: vi.fn() },
  renderer: { dispose: vi.fn() },
  runtime: {
    dispose: vi.fn(),
    connected: true,
    find: vi.fn<() => { dispose(): Promise<void> }[]>(() => []),
  },
  serve: vi.fn(),
}));
vi.mock("../src/connections/rcl.js", () => ({
  RclConnection: { create: mocks.createConnection },
}));
vi.mock("@drawcall/physics-rapier", () => ({
  setupWorld: async () => mocks.world,
}));
vi.mock("../src/cli/webgpu.js", () => ({
  createRenderer: async () => mocks.renderer,
}));
vi.mock("../src/runtime.js", () => ({
  ROS2: class {
    constructor() {
      return mocks.runtime;
    }
  },
}));
vi.mock("../src/devices/robot.js", () => ({ findRobot: () => ({ base: {} }) }));
vi.mock("../src/cli/lichtblick.js", () => ({ serveLichtblick: mocks.serve }));
import { sim } from "../src/cli/sim.js";
import { snapshot } from "../src/cli/snapshot.js";

beforeEach(() => {
  vi.clearAllMocks();
  vi.useFakeTimers();
  mocks.createConnection.mockResolvedValue(mocks.connection);
  mocks.connection.subscribe.mockReturnValue(vi.fn());
  vi.spyOn(console, "error").mockImplementation(() => {});
});
afterEach(() => {
  vi.useRealTimers();
  vi.restoreAllMocks();
});

it("disposes the snapshot connection and timeout when subscribing fails", async () => {
  mocks.connection.subscribe.mockImplementationOnce(() => {
    throw new Error("subscribe failed");
  });
  await expect(
    snapshot.parseAsync(["/image", "--out", "/tmp/unused.jpg"], {
      from: "user",
    }),
  ).rejects.toThrow("subscribe failed");
  expect(mocks.connection.dispose).toHaveBeenCalledOnce();
  expect(vi.getTimerCount()).toBe(0);
});

it("disposes the snapshot connection on timeout", async () => {
  const result = snapshot.parseAsync(["/image", "--out", "/tmp/unused.jpg"], {
    from: "user",
  });
  const failure = expect(result).rejects.toThrow("No image");
  await vi.advanceTimersByTimeAsync(10_000);
  await failure;
  expect(mocks.connection.dispose).toHaveBeenCalledOnce();
});

it("releases physics and rendering when native connection setup fails", async () => {
  mocks.createConnection.mockRejectedValueOnce(new Error("native failed"));
  await expect(
    sim.parseAsync(["missing-scene.ts"], { from: "user" }),
  ).rejects.toThrow("native failed");
  expect(mocks.renderer.dispose).toHaveBeenCalledOnce();
  expect(mocks.world.dispose).toHaveBeenCalledOnce();
});

it("releases acquired resources when the scene cannot be imported", async () => {
  await expect(
    sim.parseAsync(["missing-scene.ts"], { from: "user" }),
  ).rejects.toThrow();
  expect(mocks.runtime.dispose).toHaveBeenCalledOnce();
  expect(mocks.connection.dispose).toHaveBeenCalledOnce();
  expect(mocks.renderer.dispose).toHaveBeenCalledOnce();
  expect(mocks.world.dispose).toHaveBeenCalledOnce();
});

it("stops during viewer startup and removes timers and signal listeners", async () => {
  const before = process.listenerCount("SIGTERM");
  let signal: AbortSignal | undefined;
  mocks.serve.mockImplementation((_layout, _file, received: AbortSignal) => {
    signal = received;
    return new Promise((_resolve, reject) => {
      received.addEventListener("abort", () => {
        setTimeout(() => reject(received.reason), 25);
      });
    });
  });
  const result = sim.parseAsync(
    [new URL("./cli.test.ts", import.meta.url).pathname, "--viewer"],
    { from: "user" },
  );
  await vi.waitFor(() => expect(mocks.serve).toHaveBeenCalled());
  process.emit("SIGTERM");
  const exited = new Error("exited");
  const exit = vi.spyOn(process, "exit").mockImplementation(() => {
    throw exited;
  });
  const failure = expect(result).rejects.toBe(exited);
  await vi.advanceTimersByTimeAsync(0);
  expect(exit).not.toHaveBeenCalled();
  expect(mocks.renderer.dispose).not.toHaveBeenCalled();
  await vi.advanceTimersByTimeAsync(25);
  await failure;
  expect(exit).toHaveBeenCalledWith(0);
  expect(signal?.aborted).toBe(true);
  expect(process.listenerCount("SIGTERM")).toBe(before);
  expect(vi.getTimerCount()).toBe(0);
  expect(mocks.runtime.dispose).toHaveBeenCalledOnce();
  expect(mocks.renderer.dispose).toHaveBeenCalledOnce();
  expect(mocks.world.dispose).toHaveBeenCalledOnce();
});

it("preserves setup and cleanup failures while releasing all resources", async () => {
  const setup = new Error("native failed");
  const cleanup = new Error("renderer failed");
  mocks.createConnection.mockRejectedValueOnce(setup);
  mocks.renderer.dispose.mockImplementationOnce(() => {
    throw cleanup;
  });
  await expect(
    sim.parseAsync(["missing-scene.ts"], { from: "user" }),
  ).rejects.toMatchObject({ error: cleanup, suppressed: setup });
  expect(mocks.world.dispose).toHaveBeenCalledOnce();
});

it("preserves snapshot subscription and disposal failures", async () => {
  const subscribe = new Error("subscribe failed");
  const dispose = new Error("dispose failed");
  mocks.connection.subscribe.mockImplementationOnce(() => {
    throw subscribe;
  });
  mocks.connection.dispose.mockImplementationOnce(() => {
    throw dispose;
  });
  await expect(
    snapshot.parseAsync(["/image", "--out", "/tmp/unused.jpg"], {
      from: "user",
    }),
  ).rejects.toMatchObject({ error: dispose, suppressed: subscribe });
  expect(vi.getTimerCount()).toBe(0);
});

it("joins camera readback before releasing the renderer even when runtime disposal fails", async () => {
  let finish: (() => void) | undefined;
  const readback = new Promise<void>((resolve) => {
    finish = resolve;
  });
  const camera = { dispose: vi.fn(() => readback) };
  mocks.runtime.find.mockReturnValueOnce([camera]);
  const disposal = new Error("runtime disposal failed");
  mocks.runtime.dispose.mockImplementationOnce(() => {
    throw disposal;
  });
  const result = sim.parseAsync(["missing-scene.ts"], { from: "user" });
  const failure = expect(result).rejects.toMatchObject({ error: disposal });
  await vi.waitFor(() => expect(camera.dispose).toHaveBeenCalledOnce());
  expect(mocks.renderer.dispose).not.toHaveBeenCalled();
  if (!finish) throw new Error("Readback was not initialized");
  finish();
  await failure;
  expect(mocks.renderer.dispose).toHaveBeenCalledOnce();
});

it("reports viewer startup failure once while disposing the simulation", async () => {
  const startup = new Error("viewer failed");
  mocks.serve.mockRejectedValueOnce(startup);
  await expect(
    sim.parseAsync(
      [new URL("./cli.test.ts", import.meta.url).pathname, "--viewer"],
      { from: "user" },
    ),
  ).rejects.toBe(startup);
  expect(mocks.renderer.dispose).toHaveBeenCalledOnce();
});

it("surfaces viewer cleanup failure after cancellation", async () => {
  const cleanup = new Error("temporary directory removal failed");
  mocks.serve.mockImplementation((_layout, _file, signal: AbortSignal) => {
    return new Promise((_resolve, reject) => {
      signal.addEventListener("abort", () => reject(cleanup));
    });
  });
  const result = sim.parseAsync(
    [new URL("./cli.test.ts", import.meta.url).pathname, "--viewer"],
    { from: "user" },
  );
  const failure = expect(result).rejects.toBe(cleanup);
  await vi.waitFor(() => expect(mocks.serve).toHaveBeenCalled());
  process.emit("SIGTERM");
  await failure;
  expect(mocks.renderer.dispose).toHaveBeenCalledOnce();
});

it("observes every camera failure while waiting for all readbacks before renderer disposal", async () => {
  let finish: (() => void) | undefined;
  const slow = new Promise<void>((resolve) => {
    finish = resolve;
  });
  const failed = new Error("readback failed");
  const cameras = [
    { dispose: vi.fn(() => slow) },
    {
      dispose: vi.fn(async () => {
        throw failed;
      }),
    },
  ];
  mocks.runtime.find.mockReturnValueOnce(cameras);
  const result = sim.parseAsync(["missing-scene.ts"], { from: "user" });
  const failure = expect(result).rejects.toMatchObject({
    error: expect.objectContaining({ errors: [failed] }),
  });
  await vi.waitFor(() => {
    for (const camera of cameras) expect(camera.dispose).toHaveBeenCalledOnce();
  });
  expect(mocks.renderer.dispose).not.toHaveBeenCalled();
  if (!finish) throw new Error("Readback did not initialize");
  finish();
  await failure;
  expect(mocks.renderer.dispose).toHaveBeenCalledOnce();
});
