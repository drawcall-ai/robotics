import { EventEmitter } from "node:events";
import { afterEach, beforeEach, expect, it, vi } from "vitest";
const mocks = vi.hoisted(() => ({
  open: vi.fn(),
  close: vi.fn(),
  closeAllConnections: vi.fn(),
  listen: vi.fn(),
}));
const server = new EventEmitter();
vi.mock("node:fs", () => ({ existsSync: () => true }));
vi.mock("node:fs/promises", async (original) => ({
  ...(await original<typeof import("node:fs/promises")>()),
  readFile: async () => "/*LICHTBLICK_SUITE_DEFAULT_LAYOUT_PLACEHOLDER*/",
}));
vi.mock("node:http", () => ({
  createServer: () =>
    Object.assign(server, {
      listen: mocks.listen,
      address: () => ({ port: 8080 }),
      close: mocks.close,
      closeAllConnections: mocks.closeAllConnections,
    }),
}));
vi.mock("open", () => ({ default: mocks.open }));
import { lichtblickUrl, serveLichtblick } from "../src/cli/lichtblick.js";
beforeEach(() => {
  vi.clearAllMocks();
  vi.stubEnv("BROWSER", "");
  server.removeAllListeners();
  mocks.listen.mockImplementation(() =>
    queueMicrotask(() => server.emit("listening")),
  );
});
afterEach(() => vi.unstubAllEnvs());
it("closes the hosted viewer when opening the browser fails", async () => {
  mocks.open.mockRejectedValueOnce(new Error("browser failed"));
  await expect(serveLichtblick({}, { bridge: 8765 })).rejects.toThrow(
    "browser failed",
  );
  expect(mocks.close).toHaveBeenCalledOnce();
  expect(mocks.closeAllConnections).toHaveBeenCalledOnce();
});
it("closes the hosted viewer when its owner aborts", async () => {
  const owner = new AbortController();
  await serveLichtblick({}, { bridge: 8765 }, { signal: owner.signal });
  owner.abort();
  expect(mocks.close).toHaveBeenCalledOnce();
  expect(mocks.closeAllConnections).toHaveBeenCalledOnce();
});

it("closes the server and leaves no startup listeners when binding fails", async () => {
  mocks.listen.mockImplementationOnce(() =>
    queueMicrotask(() => server.emit("error", new Error("address in use"))),
  );
  await expect(serveLichtblick({}, { bridge: 8765 })).rejects.toThrow(
    "address in use",
  );
  expect(server.listenerCount("listening")).toBe(0);
  expect(server.listenerCount("error")).toBe(0);
  expect(mocks.close).toHaveBeenCalledOnce();
});
it("rejects aborted startup even if listening never completes", async () => {
  const owner = new AbortController();
  mocks.listen.mockImplementationOnce(() => owner.abort());
  await expect(
    serveLichtblick({}, { bridge: 8765 }, { signal: owner.signal }),
  ).rejects.toThrow();
  expect(server.listenerCount("listening")).toBe(0);
  expect(mocks.open).not.toHaveBeenCalled();
});

it("deep-links a bridge port, or every file of a recording as one", () => {
  const params = (url: string) => new URL(url).searchParams;
  expect(params(lichtblickUrl(8080, { bridge: 9000 })).get("ds.url")).toBe(
    "ws://127.0.0.1:9000",
  );
  const recording = params(
    lichtblickUrl(8080, { recordings: ["a.mcap", "b.mcap"] }),
  );
  expect(recording.get("ds")).toBe("remote-file");
  expect(recording.get("ds.url")).toBe(
    "http://127.0.0.1:8080/recording/0.mcap,http://127.0.0.1:8080/recording/1.mcap",
  );
});
