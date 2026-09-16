import { AuthoringWorld, setDefaultWorld } from "@drawcall/physics";
import { OfflineConnection } from "../offline.js";
import { findRobot } from "../robot.js";
import { ROS2 } from "../runtime.js";
import { runScene } from "./scene.js";

/** Prints the URDF of a scene, built without a physics backend, renderer or ROS. */
export async function urdf(
  file: string,
  namespace?: string,
  robot?: string,
): Promise<void> {
  setDefaultWorld(new AuthoringWorld());
  const ros = new ROS2(new OfflineConnection(), { namespace });
  await runScene(file);
  process.stdout.write(findRobot(ros, robot).urdf());
}
