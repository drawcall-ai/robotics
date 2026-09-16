import { openBrowser, serveLichtblick } from "./lichtblick.js";
import { recordingFile, recordingLayout } from "./recording.js";

/** Opens a recording in Lichtblick with the layout its sensors call for, until stopped. */
export async function view(path: string): Promise<void> {
  const file = await recordingFile(path);
  const viewer = await serveLichtblick(await recordingLayout(file), file);
  console.error(`Lichtblick: ${viewer.url}`);
  openBrowser(viewer.url);
  await new Promise<void>((resolve) => {
    process.once("SIGINT", resolve);
    process.once("SIGTERM", resolve);
  });
  viewer.close();
}
