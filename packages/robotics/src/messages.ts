/** A ROS name segment; URDF names, frame ids and topic segments share it. */
export const SEGMENT = "[A-Za-z_][A-Za-z0-9_]*";
const IDENTIFIER = new RegExp(`^${SEGMENT}$`);

export function identifier(kind: string, name: string): string {
  if (!IDENTIFIER.test(name)) throw new Error(`Invalid ${kind} name: ${name}`);
  return name;
}
/** An absolute topic: name segments joined by `/`. */
const TOPIC = new RegExp(`^(?:/${SEGMENT})+$`);
/** Where a robot's topics go unless it names its own namespace, as a sensor outside a robot's do. */
export const NAMESPACE = "/robot";

/** `value` if it is a valid ROS namespace: `/`, or absolute name segments. */
export function namespace(value: string): string {
  if (value !== "/" && !TOPIC.test(value))
    throw new Error(`Invalid ROS namespace: ${value}`);
  return value;
}

/** The absolute topic for `name` under `within`, unless `name` is absolute already. */
export function topic(name: string, within = NAMESPACE): string {
  namespace(within);
  const absolute = name.startsWith("/")
    ? name
    : `${within === "/" ? "" : within}/${name}`;
  if (!TOPIC.test(absolute)) throw new Error(`Invalid ROS topic: ${absolute}`);
  return absolute;
}

export function stamp(seconds: number): { sec: number; nanosec: number } {
  if (!Number.isFinite(seconds) || seconds < 0)
    throw new Error("Simulation time must be finite and nonnegative");
  const nanoseconds = Math.round(seconds * 1e9);
  return { sec: Math.floor(nanoseconds / 1e9), nanosec: nanoseconds % 1e9 };
}
/** Base64 text of `bytes`, in chunks small enough to spread into `String.fromCharCode`. */
export function base64(bytes: Uint8Array): string {
  let binary = "";
  for (let offset = 0; offset < bytes.length; offset += 0x8000)
    binary += String.fromCharCode(...bytes.subarray(offset, offset + 0x8000));
  return btoa(binary);
}
export function header(time: number, frame: string) {
  return { stamp: stamp(time), frame_id: frame };
}
