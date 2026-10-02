import { once } from "node:events";
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

/** What Lichtblick plays: the foxglove_bridge on a port, or the MCAP files of a recording, in order. */
export type Source = { bridge: number } | { recordings: string[] };

/** Where the `index`th file of a recording is served; the extension tells Lichtblick the format. */
const recordingPath = (index: number) => `/recording/${index}.mcap`;

/** The address of Lichtblick served on `port`, deep-linked to `source` and the served layout. */
export function lichtblickUrl(port: number, source: Source): string {
  const origin = `http://127.0.0.1:${port}`;
  // Lichtblick plays several remote files, given as a comma-separated ds.url, as one recording.
  const data =
    "bridge" in source
      ? `ds=foxglove-websocket&ds.url=${encodeURIComponent(`ws://127.0.0.1:${source.bridge}`)}`
      : `ds=remote-file&ds.url=${encodeURIComponent(source.recordings.map((_, index) => origin + recordingPath(index)).join(","))}`;
  return `${origin}/?${data}&layoutUrl=${encodeURIComponent(`${origin}/layout.json`)}`;
}

/**
 * Hosts the Lichtblick web build on loopback, on `port` or any free one, with `layout` preloaded and
 * `source` deep-linked, serving a recording's files alongside, and opens it in the browser.
 */
export async function serveLichtblick(
  layout: unknown,
  source: Source,
  { port = 0, signal }: { port?: number; signal?: AbortSignal } = {},
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
    if (path === "/layout.json") {
      response.setHeader("content-type", "application/json");
      response.setHeader("cache-control", "no-store");
      response.end(JSON.stringify(layout));
      return;
    }
    // `send` answers the range requests Lichtblick reads a recording with.
    const recording =
      "recordings" in source &&
      source.recordings.find((_, index) => path === recordingPath(index));
    if (recording) send(request, recording).pipe(response);
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
  const listening = once(server, "listening", { signal });
  server.listen({ port, host: "127.0.0.1", signal });
  await listening;
  const address = server.address();
  if (!address || typeof address === "string")
    throw new Error("Viewer has no TCP address");
  const url = lichtblickUrl(address.port, source);
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
