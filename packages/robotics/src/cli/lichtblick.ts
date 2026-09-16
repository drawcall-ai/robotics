import { existsSync } from "node:fs";
import { mkdir, mkdtemp, readFile, rename, rm } from "node:fs/promises";
import { createServer } from "node:http";
import { homedir } from "node:os";
import { dirname, join } from "node:path";
import { Readable } from "node:stream";
import { pipeline } from "node:stream/promises";
import open from "open";
import send from "send";
import { extract } from "tar";

const VERSION = "1.29.1";
const PLACEHOLDER = "/*LICHTBLICK_SUITE_DEFAULT_LAYOUT_PLACEHOLDER*/";
const DIR = join(
  homedir(),
  ".cache",
  "drawcall-robotics",
  `lichtblick-${VERSION}`,
);
const PORT = 8080;
/** The launch file's foxglove_bridge. */
const FOXGLOVE = "ws://127.0.0.1:8765";
/** Where a recording is served; the extension tells Lichtblick the format. */
const RECORDING = "/recording.mcap";

/**
 * Hosts the Lichtblick web build on loopback with `layout` preloaded, deep-linked to the launch's bridge or,
 * given a `recording`, to that MCAP file served alongside, and opens it in the browser.
 */
export async function serveLichtblick(
  layout: unknown,
  recording?: string,
  signal?: AbortSignal,
): Promise<{ url: string; close(): void }> {
  if (!existsSync(join(DIR, "index.html"))) await download(signal);
  const template = await readFile(join(DIR, "index.html"), {
    encoding: "utf8",
    signal,
  });
  signal?.throwIfAborted();
  if (!template.includes(PLACEHOLDER))
    throw new Error("Lichtblick index.html has no default layout placeholder");
  const index = template.replace(PLACEHOLDER, () => JSON.stringify(layout));
  const server = createServer((request, response) => {
    const path = new URL(request.url ?? "/", "http://localhost").pathname;
    if (path === "/" || path === "/index.html") {
      response.setHeader("content-type", "text/html; charset=utf-8");
      response.end(index);
      return;
    }
    // `send` answers the range requests Lichtblick reads a recording with.
    if (recording && path === RECORDING)
      send(request, recording).pipe(response);
    else send(request, path, { root: DIR }).pipe(response);
  });
  const close = () => {
    signal?.removeEventListener("abort", close);
    server.closeAllConnections();
    server.close();
  };
  signal?.addEventListener("abort", close, { once: true });
  using resources = new DisposableStack();
  resources.defer(close);
  await new Promise<void>((resolve, reject) => {
    const finish = (error?: unknown) => {
      signal?.removeEventListener("abort", abort);
      server.off("error", finish);
      if (error !== undefined) reject(error);
      else resolve();
    };
    const abort = () => finish(signal?.reason);
    signal?.addEventListener("abort", abort, { once: true });
    server.once("error", finish);
    server.listen({ port: PORT, host: "127.0.0.1", signal }, finish);
  });
  signal?.throwIfAborted();
  const source = recording
    ? `ds=remote-file&ds.url=${encodeURIComponent(`http://127.0.0.1:${PORT}${RECORDING}`)}`
    : `ds=foxglove-websocket&ds.url=${encodeURIComponent(FOXGLOVE)}`;
  const url = `http://127.0.0.1:${PORT}/?${source}`;
  console.error(`Lichtblick: ${url}`);
  await openBrowser(url);
  signal?.throwIfAborted();
  const owner = resources.move();
  return { url, close: () => owner.dispose() };
}

async function download(signal?: AbortSignal): Promise<void> {
  const url = `https://github.com/lichtblick-suite/lichtblick/releases/download/v${VERSION}/lichtblick-web.tar.gz`;
  console.error(`Downloading Lichtblick ${VERSION} web build…`);
  const response = await fetch(url, { signal });
  if (!response.ok || !response.body)
    throw new Error(`Lichtblick download failed: ${response.status} ${url}`);
  await mkdir(dirname(DIR), { recursive: true });
  const temporary = await mkdtemp(`${DIR}-`);
  await using resources = new AsyncDisposableStack();
  resources.defer(() => rm(temporary, { recursive: true, force: true }));
  await pipeline(Readable.from(response.body), extract({ cwd: temporary }), {
    signal,
  });
  signal?.throwIfAborted();
  await rename(temporary, DIR);
}

/** `BROWSER=none` keeps the viewer served without opening it, as with Vite. */
async function openBrowser(url: string): Promise<void> {
  if (process.env.BROWSER === "none") return;
  await open(url);
}
