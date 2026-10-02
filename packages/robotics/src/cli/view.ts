import { descriptionView, type ModelView } from "../urdf/view.js";
import { once } from "node:events";
import type { McapIndexedReader } from "@mcap/core";
import { Command } from "commander";
import { findEpisodes, openEpisode, readStrings } from "./recording.js";
import { defaultLayout } from "./layout.js";
import { serveLichtblick } from "./lichtblick.js";

/** Opens a recording in Lichtblick with the layout its sensors call for, until stopped. */
export const view = new Command("view")
  .description("open a recording in Lichtblick")
  .argument("<recording>", "a run folder, its bag folder, or an .mcap file")
  .option(
    "--show <topic>",
    "also display a topic: a compressed image in its own panel, anything else in 3D (repeatable)",
    (topic: string, shown: string[]) => [...shown, topic],
    [],
  )
  .action(async function (
    this: Command,
    path: string,
    options: { show: string[] },
  ) {
    const episodes = await findEpisodes(path);
    const [episode] = episodes;
    if (!episode || episodes.length > 1)
      this.error(
        `error: ${path} holds ${episodes.length} runs; view one of them`,
      );
    let layout: unknown;
    let files: string[];
    {
      await using recordings = new AsyncDisposableStack();
      const opened = await openEpisode(episode, recordings);
      layout = await recordingLayout(
        opened.map(({ reader }) => reader),
        options.show,
      );
      files = opened.map(({ file }) => file);
    }
    if (!files.length) this.error(`error: ${path} has nothing recorded yet`);
    // Lichtblick plays the files of a split recording as one.
    const viewer = await serveLichtblick(layout, { recordings: files });
    const controller = new AbortController();
    using resources = new DisposableStack();
    resources.defer(() => viewer.close());
    resources.defer(() => controller.abort());
    await Promise.race([
      once(process, "SIGINT", { signal: controller.signal }),
      once(process, "SIGTERM", { signal: controller.signal }),
    ]);
  });

/** The default layout for the robots, scans and images the files of a recording hold. */
async function recordingLayout(
  readers: McapIndexedReader[],
  show: string[],
): Promise<unknown> {
  const topics = (schema: string): string[] => [
    ...new Set(
      readers.flatMap((reader) =>
        [...reader.channelsById.values()]
          .filter(
            (channel) =>
              reader.schemasById.get(channel.schemaId)?.name === schema,
          )
          .map((channel) => channel.topic),
      ),
    ),
  ];
  const models = new Map<string, ModelView>();
  const descriptions = topics("std_msgs/msg/String").filter((topic) =>
    topic.endsWith("/robot_description"),
  );
  for (const reader of readers)
    for await (const description of readStrings(reader, descriptions)) {
      const model = descriptionView(description);
      if (model && !models.has(model.topic)) models.set(model.topic, model);
    }
  return defaultLayout({
    scans: topics("sensor_msgs/msg/LaserScan"),
    images: topics("sensor_msgs/msg/CompressedImage"),
    calibrations: topics("sensor_msgs/msg/CameraInfo"),
    show,
    models: [...models.values()],
  });
}
