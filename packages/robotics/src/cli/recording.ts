import { McapIndexedReader } from "@mcap/core";
import { open, readdir, stat } from "node:fs/promises";
import { join } from "node:path";
import { defaultLayout } from "./layout.js";

/** The MCAP file of a recording: the file itself, or the one inside a rosbag2 directory. */
export async function recordingFile(path: string): Promise<string> {
  if (!(await stat(path)).isDirectory()) return path;
  const files = (await readdir(path)).filter((name) => name.endsWith(".mcap"));
  const [file] = files;
  if (!file || files.length !== 1)
    throw new Error(`${path} must hold one .mcap file, found ${files.length}`);
  return join(path, file);
}

/** The default layout for the scans and cameras a recording holds. */
export async function recordingLayout(file: string): Promise<unknown> {
  const handle = await open(file);
  try {
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
    const channels = [...reader.channelsById.values()].map((channel) => ({
      topic: channel.topic,
      schema: reader.schemasById.get(channel.schemaId)?.name,
    }));
    const lasers = channels.filter(
      (channel) => channel.schema === "sensor_msgs/msg/LaserScan",
    );
    const cameras = channels
      .filter((channel) => channel.schema === "sensor_msgs/msg/CompressedImage")
      .map(({ topic }) => ({
        name: topic.split("/").at(-3) ?? topic,
        imageTopic: topic,
        infoTopic: topic.replace(/\/image\/compressed$/, "/camera_info"),
      }));
    return defaultLayout(lasers, cameras);
  } finally {
    await handle.close();
  }
}
