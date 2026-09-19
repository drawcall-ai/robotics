import {
  mkdtemp,
  mkdir,
  readFile,
  readdir,
  rm,
  writeFile,
} from "node:fs/promises";
import { existsSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { create } from "tar";
import { afterEach, beforeEach, expect, it, vi } from "vitest";

const fixture = vi.hoisted(() => ({ home: "" }));
vi.mock("node:os", async (original) => ({
  ...(await original<typeof import("node:os")>()),
  homedir: () => fixture.home,
}));
vi.mock("node:http", () => ({
  createServer: () => ({
    once() {},
    off() {},
    listen(_options: unknown, ready: () => void) {
      ready();
    },
    address: () => ({ port: 8080 }),
    closeAllConnections() {},
    close() {},
  }),
}));

vi.mock("open", () => ({ default: vi.fn() }));

const template = "/*LICHTBLICK_SUITE_DEFAULT_LAYOUT_PLACEHOLDER*/";
let cache: string;
let installed: string;
let archive: Uint8Array;

beforeEach(async () => {
  vi.resetModules();
  fixture.home = await mkdtemp(join(tmpdir(), "robotics-download-"));
  cache = join(fixture.home, ".cache", "drawcall-robotics");
  installed = join(cache, "lichtblick-1.29.1");
  const source = join(fixture.home, "source");
  await mkdir(source);
  await writeFile(join(source, "index.html"), template);
  const tarball = join(fixture.home, "viewer.tar");
  await create({ cwd: source, file: tarball }, ["index.html"]);
  archive = await readFile(tarball);
  vi.stubEnv("BROWSER", "none");
  vi.spyOn(console, "error").mockImplementation(() => {});
});

afterEach(async () => {
  vi.unstubAllGlobals();
  vi.unstubAllEnvs();
  vi.restoreAllMocks();
  await rm(fixture.home, { recursive: true, force: true });
});

function downloadStream(): ReadableStreamDefaultController<Uint8Array> {
  let input: ReadableStreamDefaultController<Uint8Array> | undefined;
  const stream = new ReadableStream<Uint8Array>({
    start(controller) {
      input = controller;
      controller.enqueue(archive);
    },
  });
  vi.stubGlobal(
    "fetch",
    vi.fn(async (_url, options: { signal?: AbortSignal }) => {
      options.signal?.addEventListener(
        "abort",
        () => input?.error(options.signal?.reason),
        { once: true },
      );
      return new Response(stream);
    }),
  );
  if (!input) throw new Error("Download stream did not initialize");
  return input;
}

async function extractedDirectory(): Promise<string> {
  let directory = "";
  await vi.waitFor(async () => {
    const directories = await readdir(cache);
    const [temporary] = directories;
    expect(directories).toHaveLength(1);
    if (!temporary) throw new Error("Download directory was not created");
    directory = join(cache, temporary);
    expect(directory).not.toBe(installed);
    expect(await readFile(join(directory, "index.html"), "utf8")).toBe(
      template,
    );
  });
  return directory;
}

it("publishes the extracted viewer only after the complete download succeeds", async () => {
  const stream = downloadStream();
  const { serveLichtblick } = await import("../src/cli/lichtblick.js");
  const startup = serveLichtblick({});
  const temporary = await extractedDirectory();
  expect(existsSync(installed)).toBe(false);
  stream.close();
  const viewer = await startup;
  try {
    expect(await readFile(join(installed, "index.html"), "utf8")).toBe(
      template,
    );
    expect(existsSync(temporary)).toBe(false);
    expect(await readdir(cache)).toEqual(["lichtblick-1.29.1"]);
  } finally {
    viewer.close();
  }
});

it("removes partial extraction before aborted startup settles without installing it", async () => {
  downloadStream();
  const owner = new AbortController();
  const { serveLichtblick } = await import("../src/cli/lichtblick.js");
  const startup = serveLichtblick({}, undefined, owner.signal);
  const failure = expect(startup).rejects.toMatchObject({ name: "AbortError" });
  const temporary = await extractedDirectory();
  owner.abort();
  await failure;
  expect(existsSync(installed)).toBe(false);
  expect(existsSync(temporary)).toBe(false);
  expect(await readdir(cache)).toEqual([]);
});
