/** What the simulator node and the runner that drives it over ROS agree on. */

/** How an evaluation that could not keep its real-time factor ends: the host was too busy. */
export const BEHIND = "could not keep the real-time factor";

export const START = "/sim/start_evaluation";
export const START_TYPE = "std_srvs/srv/Trigger";
export const SET_STATE = "/sim/set_simulation_state";
export const SET_STATE_TYPE = "simulation_interfaces/srv/SetSimulationState";

// simulation_interfaces constants: Result and SimulationState.
export const OK = 1;
export const UNSUPPORTED = 0;
export const FAILED = 4;
export const ALREADY_IN_TARGET_STATE = 101;
export const PLAYING = 1;
export const PAUSED = 2;
export const QUITTING = 3;

/** The value at `path` in a ROS message, such as `state.state`, or undefined. */
export function get(message: unknown, ...path: string[]): unknown {
  let value = message;
  for (const key of path)
    value =
      typeof value === "object" && value !== null
        ? Reflect.get(value, key)
        : undefined;
  return value;
}

/** A number in a ROS message; native transports deliver 64-bit integers as bigints. */
export function field(message: unknown, ...path: string[]): number {
  const value = get(message, ...path);
  if (typeof value === "bigint") return Number(value);
  if (typeof value !== "number")
    throw new Error(`Field ${path.join(".")} is missing`);
  return value;
}
