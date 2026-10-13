import { Command } from "commander";
import { createHash } from "node:crypto";
import { writeFile } from "node:fs/promises";
import { basename, dirname, extname, join, resolve } from "node:path";
import { pathToFileURL } from "node:url";
import { OfflineConnection } from "../connections/offline.js";
import { ROS2 } from "../runtime.js";
import { exportURDF } from "../urdf/export.js";

const EMBEDDED = /data:model\/stl;base64,([A-Za-z0-9+/=]+)/g;

/**
 * Prints or writes the URDF of a scene, built without a physics backend, renderer or ROS. Meshes are
 * embedded by default; `--separate` writes them beside the output, as glTF tools do.
 */
export const urdf = new Command("urdf")
  .description("print or write the URDF of the scene's robot")
  .argument("<scene>", "the scene script")
  .option("--namespace <ns>", "topic namespace", "/robot")
  .option("--robot <name>", "the robot, if the scene has several")
  .option("-o, --output <file>", "write the URDF to a file instead of stdout")
  .option(
    "-s, --separate",
    "write meshes as STL files beside the output instead of embedding them",
  )
  .action(async function (
    this: Command,
    scene: string,
    options: {
      namespace: string;
      robot?: string;
      output?: string;
      separate?: boolean;
    },
  ) {
    if (options.separate && !options.output)
      this.error("error: --separate writes files beside --output; give one");
    let xml: string;
    {
      using resources = new DisposableStack();
      const ros = new ROS2(new OfflineConnection(), {
        namespace: options.namespace,
      });
      resources.defer(() => ros.dispose());
      await import(pathToFileURL(resolve(scene)).href);
      xml = exportURDF({ ros, name: options.robot });
    }
    if (!options.output) {
      process.stdout.write(xml);
      return;
    }
    const output = resolve(options.output);
    await writeFile(
      output,
      options.separate ? await separateMeshes(xml, output) : xml,
    );
  });

/**
 * Writes every embedded mesh of `xml` beside `output` as `<name>-<hash>.stl` and points at it with an
 * absolute `file://` URI, the scheme ROS resource retrieval resolves. Identical meshes share a file.
 */
export async function separateMeshes(
  xml: string,
  output: string,
): Promise<string> {
  const name = basename(output, extname(output));
  const files = new Map<string, string>();
  const separated = xml.replace(EMBEDDED, (_, data: string) => {
    const bytes = Buffer.from(data, "base64");
    const hash = createHash("sha256").update(bytes).digest("hex").slice(0, 12);
    const file = join(dirname(output), `${name}-${hash}.stl`);
    files.set(file, data);
    return pathToFileURL(file).href;
  });
  for (const [file, data] of files)
    await writeFile(file, Buffer.from(data, "base64"));
  return separated;
}
