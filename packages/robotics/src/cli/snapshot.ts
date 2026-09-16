import { Command } from "commander";
import { writeFile } from "node:fs/promises";
import { RclConnection } from "../connections/rcl.js";

/** Writes the next compressed image on `topic` to `file`, for anyone who needs to look at what a camera sees. */
export const snapshot = new Command("snapshot")
  .description("save the next compressed image on a topic")
  .argument("<topic>")
  .requiredOption("--out <file.jpg>", "where to write the image")
  .action((topic: string, options: { out: string }) => run(topic, options.out));

async function run(topic: string, file: string): Promise<void> {
  const connection = await RclConnection.create("snapshot");
  let timer: ReturnType<typeof setTimeout> | undefined;
  let message: unknown;
  {
    using resources = new DisposableStack();
    resources.defer(() => connection.dispose());
    resources.defer(() => clearTimeout(timer));
    message = await new Promise<unknown>((resolve, reject) => {
      timer = setTimeout(
        () => reject(new Error(`No image on ${topic} within 10 s`)),
        10_000,
      );
      connection.subscribe(topic, "sensor_msgs/msg/CompressedImage", resolve);
    });
  }
  if (typeof message !== "object" || message === null || !("data" in message))
    throw new Error("Malformed CompressedImage");
  const data = message.data;
  if (!(data instanceof Uint8Array))
    throw new Error("CompressedImage data must be bytes");
  await writeFile(file, data);
  console.log(`Wrote ${file} (${data.byteLength} bytes)`);
}
