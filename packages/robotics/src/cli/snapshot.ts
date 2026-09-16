import { writeFile } from "node:fs/promises";
import { RclConnection } from "../rcl.js";

/** Writes the next compressed image on `topic` to `file`, for anyone who needs to look at what a camera sees. */
export async function snapshot(topic: string, file: string): Promise<void> {
  const connection = await RclConnection.create("snapshot");
  const message = await new Promise<unknown>((resolve, reject) => {
    const timer = setTimeout(() => {
      release();
      reject(new Error(`No image on ${topic} within 10 s`));
    }, 10_000);
    const release = connection.subscribe(
      topic,
      "sensor_msgs/msg/CompressedImage",
      (received) => {
        clearTimeout(timer);
        release();
        resolve(received);
      },
    );
  });
  connection.dispose();
  if (typeof message !== "object" || message === null || !("data" in message))
    throw new Error("Malformed CompressedImage");
  const data = message.data;
  if (!(data instanceof Uint8Array))
    throw new Error("CompressedImage data must be bytes");
  await writeFile(file, data);
  console.log(`Wrote ${file} (${data.byteLength} bytes)`);
}
