import { afterEach, beforeEach, expect, it, vi } from "vitest";
const mocks = vi.hoisted(() => ({
  open: vi.fn(),
  close: vi.fn(),
  closeAllConnections: vi.fn(),
  listen: vi.fn(),
  once: vi.fn(),
  off: vi.fn(),
}));
vi.mock("node:fs", () => ({ existsSync: () => true }));
vi.mock("node:fs/promises", async (original) => ({
  ...(await original<typeof import("node:fs/promises")>()),
  readFile: async () => "/*LICHTBLICK_SUITE_DEFAULT_LAYOUT_PLACEHOLDER*/",
}));
vi.mock("node:http", () => ({
  createServer: () => ({
    once: mocks.once,
    off: mocks.off,
    listen: mocks.listen,
    close: mocks.close,
    closeAllConnections: mocks.closeAllConnections,
  }),
}));
vi.mock("open", () => ({ default: mocks.open }));
import { serveLichtblick } from "../src/cli/lichtblick.js";
beforeEach(() => {
  vi.clearAllMocks();
  vi.stubEnv("BROWSER", "");
  mocks.listen.mockImplementation((_options: unknown, listening: () => void) =>
    listening(),
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

it("removes startup listeners and closes the server when binding fails", async () => {
  const owner = new AbortController();
  const remove = vi.spyOn(owner.signal, "removeEventListener");
  mocks.listen.mockImplementationOnce(() => {
    const handler = mocks.once.mock.calls[0]?.[1];
    if (typeof handler !== "function")
      throw new Error("Missing server error listener");
    handler(new Error("address in use"));
  });
  await expect(serveLichtblick({}, undefined, owner.signal)).rejects.toThrow(
    "address in use",
  );
  expect(mocks.off).toHaveBeenCalledWith("error", expect.any(Function));
  expect(remove).toHaveBeenCalledTimes(2);
  expect(mocks.close).toHaveBeenCalledOnce();
});
it("rejects aborted startup even if listening never completes", async () => {
  const owner = new AbortController();
  mocks.listen.mockImplementationOnce(() => owner.abort());
  await expect(serveLichtblick({}, undefined, owner.signal)).rejects.toThrow();
  expect(mocks.off).toHaveBeenCalledWith("error", expect.any(Function));
  expect(mocks.open).not.toHaveBeenCalled();
});
