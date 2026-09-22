import { registry } from "@drawcall/physics";
import { Command } from "commander";
import { resolve } from "node:path";
import { pathToFileURL } from "node:url";
import { OfflineConnection } from "../connections/offline.js";
import { ROS2 } from "../runtime.js";
import { exportURDF } from "../urdf/export.js";

/** Prints the URDF of a scene, built without a physics backend, renderer or ROS. */
export const urdf = new Command("urdf")
  .description("print the URDF of the scene's robot")
  .argument("<scene>", "the scene script")
  .option("--namespace <ns>", "topic namespace", "/robot")
  .option("--robot <name>", "the robot, if the scene has several")
  .action(
    async (scene: string, options: { namespace: string; robot?: string }) => {
      using resources = new DisposableStack();
      resources.defer(() => registry.clear());
      const ros = new ROS2(new OfflineConnection(), {
        namespace: options.namespace,
      });
      resources.defer(() => ros.dispose());
      await import(pathToFileURL(resolve(scene)).href);
      process.stdout.write(exportURDF({ ros, name: options.robot }));
    },
  );
