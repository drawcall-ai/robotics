import { spawn } from "node:child_process";
import { existsSync } from "node:fs";
import { mkdir, readFile } from "node:fs/promises";
import { createServer } from "node:http";
import { homedir } from "node:os";
import { join } from "node:path";
import send from "send";

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
 * given a `recording`, to that MCAP file served alongside.
 */
export async function serveLichtblick(
  layout: unknown,
  recording?: string,
): Promise<{ url: string; close(): void }> {
  if (!existsSync(join(DIR, "index.html"))) await download();
  const template = await readFile(join(DIR, "index.html"), "utf8");
  if (!template.includes(PLACEHOLDER))
    throw new Error("Lichtblick index.html has no default layout placeholder");
  const index = template.replace(PLACEHOLDER, JSON.stringify(layout));
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
  await new Promise<void>((resolve, reject) => {
    server.once("error", reject);
    server.listen(PORT, "127.0.0.1", resolve);
  });
  const source = recording
    ? `ds=remote-file&ds.url=${encodeURIComponent(`http://127.0.0.1:${PORT}${RECORDING}`)}`
    : `ds=foxglove-websocket&ds.url=${encodeURIComponent(FOXGLOVE)}`;
  return {
    url: `http://127.0.0.1:${PORT}/?${source}`,
    close: () => {
      server.closeAllConnections();
      server.close();
    },
  };
}

async function download(): Promise<void> {
  const url = `https://github.com/lichtblick-suite/lichtblick/releases/download/v${VERSION}/lichtblick-web.tar.gz`;
  console.error(`Downloading Lichtblick ${VERSION} web build…`);
  const response = await fetch(url);
  if (!response.ok)
    throw new Error(`Lichtblick download failed: ${response.status} ${url}`);
  const archive = new Uint8Array(await response.arrayBuffer());
  await mkdir(DIR, { recursive: true });
  await new Promise<void>((resolve, reject) => {
    const tar = spawn("tar", ["-xz", "-C", DIR], {
      stdio: ["pipe", "inherit", "inherit"],
    });
    tar.once("error", reject);
    tar.once("exit", (code) =>
      code === 0
        ? resolve()
        : reject(new Error(`tar failed with status ${code}`)),
    );
    tar.stdin.end(archive);
  });
}

/** `BROWSER=none` keeps the viewer served without opening it, as with Vite. */
export function openBrowser(url: string): void {
  if (process.env.BROWSER === "none") return;
  const child = spawn(
    process.platform === "darwin" ? "open" : "xdg-open",
    [url],
    {
      stdio: "ignore",
      detached: true,
    },
  );
  child.once("error", () =>
    console.error(`Could not open a browser; open ${url} yourself`),
  );
  child.unref();
}
