import { descriptionView, type ModelView } from "../urdf/view.js";
import { once } from "node:events";
import { Command } from "commander";
import { recordingFiles, openRecording, readStrings } from "./recording.js";
import { EVALUATION_TOPIC } from "../evaluation.js";
import { defaultLayout } from "./layout.js";
import { serveLichtblick } from "./lichtblick.js";

/** Opens a recording in Lichtblick with the layout its sensors call for, until stopped. */
export const view = new Command("view")
  .description("open a recording in Lichtblick")
  .argument("<recording>", "a rosbag2 directory or .mcap file")
  .option(
    "--show <topic>",
    "also display a topic: a compressed image in its own panel, anything else in 3D (repeatable)",
    (topic: string, shown: string[]) => [...shown, topic],
    [],
  )
  .action(run);

async function run(path: string, options: { show: string[] }): Promise<void> {
  const file = await recordingFile(path);
  const viewer = await serveLichtblick(
    await recordingLayout(file, options.show),
    file,
  );
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

/** The default layout for the robots, scans and images a recording holds. */
async function recordingLayout(file: string, show: string[]): Promise<unknown> {
  await using recording = await openRecording(file);
  const { reader } = recording;
  const topics = (schema: string): string[] =>
    [...reader.channelsById.values()]
      .filter(
        (channel) => reader.schemasById.get(channel.schemaId)?.name === schema,
      )
      .map((channel) => channel.topic);
  const models = new Map<string, ModelView>();
  const descriptions = topics("std_msgs/msg/String").filter((topic) =>
    topic.endsWith("/robot_description"),
  );
  for await (const description of readStrings(reader, descriptions)) {
    const model = descriptionView(description);
    if (model && !models.has(model.topic)) models.set(model.topic, model);
  }
  return defaultLayout({
    scans: topics("sensor_msgs/msg/LaserScan"),
    images: topics("sensor_msgs/msg/CompressedImage"),
    calibrations: topics("sensor_msgs/msg/CameraInfo"),
    show,
    evaluation: [...reader.channelsById.values()].some(
      (channel) => channel.topic === EVALUATION_TOPIC,
    ),
    models: [...models.values()],
  });
}
