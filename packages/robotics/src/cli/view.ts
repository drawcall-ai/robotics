import { McapIndexedReader } from "@mcap/core";
import { once } from "node:events";
import { Command } from "commander";
import { open, readdir, stat } from "node:fs/promises";
import { join } from "node:path";
import { defaultLayout } from "./layout.js";
import { serveLichtblick } from "./lichtblick.js";

/** Opens a recording in Lichtblick with the layout its sensors call for, until stopped. */
export const view = new Command("view")
  .description("open a recording in Lichtblick")
  .argument("<recording>", "a rosbag2 directory or .mcap file")
  .action(run);

async function run(path: string): Promise<void> {
  const file = await recordingFile(path);
  const viewer = await serveLichtblick(await recordingLayout(file), file);
  const controller = new AbortController();
  using resources = new DisposableStack();
  resources.defer(() => viewer.close());
  resources.defer(() => controller.abort());
  await Promise.race([
    once(process, "SIGINT", { signal: controller.signal }),
    once(process, "SIGTERM", { signal: controller.signal }),
  ]);
}

/** The MCAP file of a recording: the file itself, or the one inside a rosbag2 directory. */
async function recordingFile(path: string): Promise<string> {
  if (!(await stat(path)).isDirectory()) return path;
  const files = (await readdir(path)).filter((name) => name.endsWith(".mcap"));
  const [file] = files;
  if (!file || files.length !== 1)
    throw new Error(`${path} must hold one .mcap file, found ${files.length}`);
  return join(path, file);
}

/** The default layout for the scans and images a recording holds. */
async function recordingLayout(file: string): Promise<unknown> {
  await using handle = await open(file);
  const reader = await McapIndexedReader.Initialize({
    readable: {
      size: async () => BigInt((await handle.stat()).size),
      read: async (offset, size) => {
        const buffer = new Uint8Array(Number(size));
        const { bytesRead } = await handle.read(
          buffer,
          0,
          buffer.length,
          Number(offset),
        );
        return buffer.subarray(0, bytesRead);
      },
    },
  });
  const topics = (schema: string): string[] =>
    [...reader.channelsById.values()]
      .filter(
        (channel) => reader.schemasById.get(channel.schemaId)?.name === schema,
      )
      .map((channel) => channel.topic);
  return defaultLayout(
    topics("sensor_msgs/msg/LaserScan"),
    topics("sensor_msgs/msg/CompressedImage"),
  );
}
