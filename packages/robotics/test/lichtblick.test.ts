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
import { serveLichtblick } from "../src/cli/lichtblick.js";
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
  await expect(serveLichtblick({})).rejects.toThrow("browser failed");
  expect(mocks.close).toHaveBeenCalledOnce();
  expect(mocks.closeAllConnections).toHaveBeenCalledOnce();
});
it("closes the hosted viewer when its owner aborts", async () => {
  const owner = new AbortController();
  await serveLichtblick({}, undefined, owner.signal);
  owner.abort();
  expect(mocks.close).toHaveBeenCalledOnce();
  expect(mocks.closeAllConnections).toHaveBeenCalledOnce();
});

it("closes the server and leaves no startup listeners when binding fails", async () => {
  mocks.listen.mockImplementationOnce(() =>
    queueMicrotask(() => server.emit("error", new Error("address in use"))),
  );
  await expect(serveLichtblick({})).rejects.toThrow("address in use");
  expect(server.listenerCount("listening")).toBe(0);
  expect(server.listenerCount("error")).toBe(0);
  expect(mocks.close).toHaveBeenCalledOnce();
});
it("rejects aborted startup even if listening never completes", async () => {
  const owner = new AbortController();
  mocks.listen.mockImplementationOnce(() => owner.abort());
  await expect(serveLichtblick({}, undefined, owner.signal)).rejects.toThrow();
  expect(server.listenerCount("listening")).toBe(0);
  expect(mocks.open).not.toHaveBeenCalled();
});
