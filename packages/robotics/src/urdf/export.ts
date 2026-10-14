import type { Object3D } from "three";
import { findRobot } from "../devices/robot.js";
import { robotURDF } from "./serialize.js";

/** The URDF of the robot under `root`, or of the one named `name`, with no runtime or simulation. */
export function exportURDF(root: Object3D, name?: string): string {
  return robotURDF(findRobot(root, name));
}
