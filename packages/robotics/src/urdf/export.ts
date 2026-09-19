import { findRobot } from "../devices/robot.js";
import { getROS2, type ROS2 } from "../runtime.js";
import { robotURDF } from "./serialize.js";

/** Export the runtime's robot, or the one selected by name, without starting simulation. */
export function exportURDF({
  ros = getROS2(),
  name,
}: { ros?: ROS2; name?: string } = {}): string {
  return robotURDF(findRobot(ros, name));
}
