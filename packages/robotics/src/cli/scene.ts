import { createRequire } from "node:module";
import { pathToFileURL } from "node:url";
import { GLTSLoader } from "@drawcall/glts";
import { LoadingManager } from "three";
import "./browser.js";

/**
 * A loader for the .glts scene at `scene`. A package the scene imports resolves as the CLI's own
 * first, so the scene shares the CLI's three, physics and robotics instances, and otherwise from
 * the scene's folder.
 */
export function sceneLoader(scene: URL): GLTSLoader {
  // Node ignores import.meta.resolve's parent argument, so the scene's own packages resolve as
  // require would find them.
  const local = createRequire(scene);
  return new GLTSLoader(new LoadingManager(), {
    resolve: (specifier) => {
      try {
        return import.meta.resolve(specifier);
      } catch {
        return pathToFileURL(local.resolve(specifier)).href;
      }
    },
  });
}
