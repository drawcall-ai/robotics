import { stamp } from "./time.js";

export function header(time: number, frameId: string) {
  if (!frameId || frameId.startsWith("/"))
    throw new Error("ROS frame IDs must be nonempty and relative");
  return { stamp: stamp(time), frame_id: frameId };
}
