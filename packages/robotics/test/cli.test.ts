import { afterEach, beforeEach, expect, it, vi } from "vitest";

const mocks = vi.hoisted(() => ({
  connection: { dispose: vi.fn(), subscribe: vi.fn(), service: vi.fn() },
  createConnection: vi.fn(),
  createRuntime: vi.fn(),
  world: { time: 0, dispose: vi.fn(), fixedDelta: 1 / 120, update: vi.fn() },
  renderer: { dispose: vi.fn() },
  runtime: {
    disposeAsync: vi.fn<() => Promise<void>>(),
    connected: true,
    publish: vi.fn(),
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
      return mocks.createRuntime();
    }
  },
}));
vi.mock("../src/devices/robot.js", () => ({
  findRobot: () => ({ base: {}, beforeStep() {} }),
}));
vi.mock("../src/urdf/view.js", () => ({
  modelView: () => ({
    topic: "/robot/robot_description",
    frame: "base_footprint",
    distance: 2,
    target: [0, 0, 0],
  }),
}));
vi.mock("../src/cli/lichtblick.js", () => ({ serveLichtblick: mocks.serve }));
import { sim } from "../src/cli/sim.js";
import { snapshot } from "../src/cli/snapshot.js";

beforeEach(() => {
  vi.clearAllMocks();
  vi.useFakeTimers();
  mocks.createConnection.mockResolvedValue(mocks.connection);
  mocks.createRuntime.mockReturnValue(mocks.runtime);
  mocks.runtime.disposeAsync.mockResolvedValue(undefined);
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

it("releases the connection when runtime construction fails before taking ownership", async () => {
  mocks.createRuntime.mockImplementationOnce(() => {
    throw new Error("runtime setup failed");
  });
  await expect(
    sim.parseAsync(["missing-scene.ts"], { from: "user" }),
  ).rejects.toThrow("runtime setup failed");
  expect(mocks.connection.dispose).toHaveBeenCalledOnce();
  expect(mocks.runtime.disposeAsync).not.toHaveBeenCalled();
  expect(mocks.renderer.dispose).toHaveBeenCalledOnce();
  expect(mocks.world.dispose).toHaveBeenCalledOnce();
});

it("releases acquired resources when the scene cannot be imported", async () => {
  await expect(
    sim.parseAsync(["missing-scene.ts"], { from: "user" }),
  ).rejects.toThrow();
  expect(mocks.runtime.disposeAsync).toHaveBeenCalledOnce();
  expect(mocks.connection.dispose).not.toHaveBeenCalled();
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
  expect(mocks.runtime.disposeAsync).toHaveBeenCalledOnce();
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

it("waits for runtime cleanup before releasing the renderer, including on failure", async () => {
  let reject: ((error: Error) => void) | undefined;
  const cleanup = new Promise<void>((_resolve, fail) => {
    reject = fail;
  });
  mocks.runtime.disposeAsync.mockReturnValueOnce(cleanup);
  const disposal = new Error("runtime disposal failed");
  const result = sim.parseAsync(["missing-scene.ts"], { from: "user" });
  const failure = expect(result).rejects.toMatchObject({ error: disposal });
  await vi.waitFor(() =>
    expect(mocks.runtime.disposeAsync).toHaveBeenCalledOnce(),
  );
  expect(mocks.renderer.dispose).not.toHaveBeenCalled();
  if (!reject) throw new Error("Runtime cleanup did not initialize");
  reject(disposal);
  await failure;
  expect(mocks.renderer.dispose).toHaveBeenCalledOnce();
  expect(mocks.world.dispose).toHaveBeenCalledOnce();
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
