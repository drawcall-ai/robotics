/** A ROS name segment; URDF names, frame ids and topic segments share it. */
export const SEGMENT = "[A-Za-z_][A-Za-z0-9_]*";
const IDENTIFIER = new RegExp(`^${SEGMENT}$`);

export function identifier(kind: string, name: string): string {
  if (!IDENTIFIER.test(name)) throw new Error(`Invalid ${kind} name: ${name}`);
  return name;
}
export function stamp(seconds: number): { sec: number; nanosec: number } {
  if (!Number.isFinite(seconds) || seconds < 0)
    throw new Error("Simulation time must be finite and nonnegative");
  const nanoseconds = Math.round(seconds * 1e9);
  return { sec: Math.floor(nanoseconds / 1e9), nanosec: nanoseconds % 1e9 };
}
export function header(time: number, frame: string) {
  return { stamp: stamp(time), frame_id: frame };
}
