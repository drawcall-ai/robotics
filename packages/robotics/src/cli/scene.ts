import { resolve } from "node:path";
import { pathToFileURL } from "node:url";

/** Executes a scene module. It builds against `getROS2()` and registers its devices there. */
export async function runScene(file: string): Promise<void> {
  await import(pathToFileURL(resolve(file)).href);
}
