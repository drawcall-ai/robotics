import { stamp } from "./time.js";

/** A frame id is relative: TF resolves it under the node's namespace. */
export function frameId(id: string): string {
  if (!id || id.startsWith("/"))
    throw new Error("ROS frame IDs must be nonempty and relative");
  return id;
}
export function header(time: number, frame: string) {
  return { stamp: stamp(time), frame_id: frame };
}
