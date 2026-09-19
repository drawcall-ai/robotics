import { MessageReader } from "@foxglove/rosmsg2-serialization";
import { parse } from "@foxglove/rosmsg";
import { descriptionView, type ModelView } from "../urdf/view.js";
import { once } from "node:events";
import { Command } from "commander";
import { recordingFiles, openRecording } from "./recording.js";
import { EVALUATION_TOPIC } from "../evaluation.js";
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
  const files = await recordingFiles(path);
  const [file] = files;
  if (!file || files.length !== 1)
    throw new Error(
      "Viewer requires one MCAP file; results supports split recordings",
    );
  return file;
}

/** The default layout for the scans and images a recording holds. */
async function recordingLayout(file: string): Promise<unknown> {
  await using recording = await openRecording(file);
  const { reader } = recording;
  const topics = (schema: string): string[] =>
    [...reader.channelsById.values()]
      .filter(
        (channel) => reader.schemasById.get(channel.schemaId)?.name === schema,
      )
      .map((channel) => channel.topic);
  let model: ModelView | undefined;
  const descriptions = topics("std_msgs/msg/String").filter((topic) =>
    topic.endsWith("/robot_description"),
  );
  for await (const message of reader.readMessages({ topics: descriptions })) {
    const channel = reader.channelsById.get(message.channelId);
    const schema = channel && reader.schemasById.get(channel.schemaId);
    if (
      !schema ||
      channel.messageEncoding !== "cdr" ||
      schema.encoding !== "ros2msg"
    )
      throw new Error("Unsupported robot description encoding");
    const decoder = new MessageReader(
      parse(new TextDecoder().decode(schema.data), { ros2: true }),
    );
    const envelope: unknown = decoder.readMessage(message.data);
    if (
      typeof envelope !== "object" ||
      envelope === null ||
      !("data" in envelope) ||
      typeof envelope.data !== "string"
    )
      throw new Error("Invalid robot description");
    model = descriptionView(envelope.data);
    if (model) break;
  }
  return defaultLayout(
    topics("sensor_msgs/msg/LaserScan"),
    topics("sensor_msgs/msg/CompressedImage"),
    [...reader.channelsById.values()].some(
      (channel) => channel.topic === EVALUATION_TOPIC,
    ),
    model,
  );
}
