import { afterEach, beforeEach, expect, it, vi } from "vitest";

const { Evaluator, evaluator, Robot, robot, actuator, split } = vi.hoisted(
  () => {
    class Evaluator {
      finished = false;
      active = false;
      result = undefined;
      options = {};
      end = vi.fn();
    }
    const actuator = { release: vi.fn() };
    class Robot {
      private = false;
      descriptionTopic = "/robot/robot_description";
      actuators = new Map([["wheel", actuator]]);
      beforeStep = vi.fn();
    }
    return {
      Evaluator,
      evaluator: new Evaluator(),
      Robot,
      robot: new Robot(),
      actuator,
      split: vi.fn(async () => {}),
    };
  },
);
const mocks = vi.hoisted(() => ({
  connection: {
    dispose: vi.fn(),
    subscribe: vi.fn(),
    service:
      vi.fn<
        (
          type: string,
          name: string,
          handle: (request: unknown) => unknown,
        ) => void
      >(),
  },
  createConnection: vi.fn(),
  createRuntime: vi.fn(),
  world: {
    time: 0,
    dispose: vi.fn(),
    fixedDelta: 1 / 120,
    update: vi.fn(),
    onBeforeStep: vi.fn(),
  },
  loader: { loadAsync: vi.fn(), dispose: vi.fn() },
  renderer: { dispose: vi.fn() },
  runtime: {
    disposeAsync: vi.fn<() => Promise<void>>(),
    attachWorld: vi.fn(),
    attachRecording: vi.fn(),
    show: [],
    resume: vi.fn(),
    connected: true,
    publish: vi.fn(),
    sync: vi.fn(),
    find: vi.fn((type: unknown) =>
      type === Evaluator ? [evaluator] : type === Robot ? [robot] : [],
    ),
  },
  serve: vi.fn(),
}));
vi.mock("../src/connections/rcl.js", () => ({
  RclConnection: { create: mocks.createConnection },
}));
vi.mock("@drawcall/physics-mujoco", () => ({
  buildWorld: async () => mocks.world,
}));
vi.mock("../src/cli/scene.js", () => ({ sceneLoader: () => mocks.loader }));
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
vi.mock("../src/devices/robot.js", () => ({ Robot }));
vi.mock("../src/evaluation.js", async (original) => ({
  ...(await original<object>()),
  Evaluator,
}));
vi.mock("../src/cli/recorder.js", () => ({
  Recorder: class {
    ready = async () => {};
    check() {}
    write() {}
    split = split;
    async stop() {}
  },
}));
vi.mock("../src/urdf/view.js", () => ({
  modelView: () => ({
    topic: "/robot/robot_description",
    framePrefix: "",
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
  mocks.loader.loadAsync.mockResolvedValue({ update: vi.fn() });
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

it("releases rendering before physics is built when native connection setup fails", async () => {
  mocks.createConnection.mockRejectedValueOnce(new Error("native failed"));
  await expect(
    sim.parseAsync(["scene.glts", "--record", "bag"], { from: "user" }),
  ).rejects.toThrow("native failed");
  expect(mocks.renderer.dispose).toHaveBeenCalledOnce();
  expect(mocks.world.dispose).not.toHaveBeenCalled();
});

it("releases the connection when runtime construction fails before taking ownership", async () => {
  mocks.createRuntime.mockImplementationOnce(() => {
    throw new Error("runtime setup failed");
  });
  await expect(
    sim.parseAsync(["scene.glts", "--record", "bag"], { from: "user" }),
  ).rejects.toThrow("runtime setup failed");
  expect(mocks.connection.dispose).toHaveBeenCalledOnce();
  expect(mocks.runtime.disposeAsync).not.toHaveBeenCalled();
  expect(mocks.renderer.dispose).toHaveBeenCalledOnce();
  expect(mocks.world.dispose).not.toHaveBeenCalled();
});

it("releases acquired resources when the scene cannot be loaded", async () => {
  mocks.loader.loadAsync.mockRejectedValueOnce(new Error("bad scene"));
  await expect(
    sim.parseAsync(["scene.glts", "--record", "bag"], { from: "user" }),
  ).rejects.toThrow("bad scene");
  expect(mocks.loader.dispose).toHaveBeenCalledOnce();
  expect(mocks.createConnection).not.toHaveBeenCalled();
  expect(mocks.createRuntime).not.toHaveBeenCalled();
  expect(mocks.renderer.dispose).toHaveBeenCalledOnce();
  expect(mocks.world.dispose).not.toHaveBeenCalled();
});

it("stops during viewer startup and removes timers and signal listeners", async () => {
  const before = process.listenerCount("SIGTERM");
  let signal: AbortSignal | undefined;
  mocks.serve.mockImplementation(
    (_layout, _source, { signal: received }: { signal: AbortSignal }) => {
      signal = received;
      return new Promise((_resolve, reject) => {
        received.addEventListener("abort", () => {
          setTimeout(() => reject(received.reason), 25);
        });
      });
    },
  );
  const result = sim.parseAsync(["scene.glts", "--viewer", "--record", "bag"], {
    from: "user",
  });
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
    sim.parseAsync(["scene.glts", "--record", "bag"], { from: "user" }),
  ).rejects.toMatchObject({ error: cleanup, suppressed: setup });
  expect(mocks.world.dispose).not.toHaveBeenCalled();
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
  mocks.runtime.attachWorld.mockImplementationOnce(() => {
    throw new Error("attach failed");
  });
  const disposal = new Error("runtime disposal failed");
  const result = sim.parseAsync(["scene.glts", "--record", "bag"], {
    from: "user",
  });
  const failure = expect(result).rejects.toMatchObject({ error: disposal });
  await vi.waitFor(() =>
    expect(mocks.runtime.disposeAsync).toHaveBeenCalledOnce(),
  );
  expect(mocks.renderer.dispose).not.toHaveBeenCalled();
  if (!reject) throw new Error("Runtime cleanup did not initialize");
  reject(disposal);
  await failure;
  expect(mocks.renderer.dispose).toHaveBeenCalledOnce();
});

it("reports viewer startup failure once while disposing the simulation", async () => {
  const startup = new Error("viewer failed");
  mocks.serve.mockRejectedValueOnce(startup);
  await expect(
    sim.parseAsync(["scene.glts", "--viewer", "--record", "bag"], {
      from: "user",
    }),
  ).rejects.toBe(startup);
  expect(mocks.renderer.dispose).toHaveBeenCalledOnce();
});

it("surfaces viewer cleanup failure after cancellation", async () => {
  const cleanup = new Error("temporary directory removal failed");
  mocks.serve.mockImplementation(
    (_layout, _source, { signal }: { signal: AbortSignal }) => {
      return new Promise((_resolve, reject) => {
        signal.addEventListener("abort", () => reject(cleanup));
      });
    },
  );
  const result = sim.parseAsync(["scene.glts", "--viewer", "--record", "bag"], {
    from: "user",
  });
  const failure = expect(result).rejects.toBe(cleanup);
  await vi.waitFor(() => expect(mocks.serve).toHaveBeenCalled());
  process.emit("SIGTERM");
  await failure;
  expect(mocks.renderer.dispose).toHaveBeenCalledOnce();
});

it("releases the actuators at a pause and at the next play, splits the recording at a pause, and ends the evaluation on quitting", async () => {
  const exited = new Error("exited");
  const exit = vi.spyOn(process, "exit").mockImplementation(() => {
    throw exited;
  });
  const result = sim.parseAsync(["scene.glts", "--record", "bag"], {
    from: "user",
  });
  const failure = expect(result).rejects.toBe(exited);
  const handler = async (name: string) => {
    await vi.waitFor(() =>
      expect(mocks.connection.service.mock.calls.map((c) => c[1])).toContain(
        name,
      ),
    );
    const call = mocks.connection.service.mock.calls.find((c) => c[1] === name);
    if (!call) throw new Error(`No ${name} service`);
    return call[2];
  };
  const set = await handler("/sim/set_simulation_state");
  expect(await set({ state: { state: 0 } })).toMatchObject({
    result: { result: 0 },
  });
  expect(await set({ state: { state: 2 } })).toMatchObject({
    result: { result: 1 },
  });
  expect(actuator.release).toHaveBeenCalledOnce();
  expect(split).toHaveBeenCalledOnce();
  expect(await set({ state: { state: 2 } })).toMatchObject({
    result: { result: 101 },
  });
  expect(await set({ state: { state: 1 } })).toMatchObject({
    result: { result: 1 },
  });
  expect(actuator.release).toHaveBeenCalledTimes(2);
  expect(mocks.runtime.resume).toHaveBeenCalledOnce();
  expect(await set({ state: { state: 3 } })).toMatchObject({
    result: { result: 1 },
  });
  await vi.advanceTimersByTimeAsync(100);
  await failure;
  expect(evaluator.end).toHaveBeenCalledWith("aborted", "quit");
  expect(exit).toHaveBeenCalledWith(0);
});
