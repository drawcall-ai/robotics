export function readStatus(message: unknown) {
  if (
    typeof message !== "object" ||
    message === null ||
    !("data" in message) ||
    typeof message.data !== "string"
  ) {
    throw new Error("ROS status must contain a string data field");
  }
  const value: unknown = JSON.parse(message.data);
  if (
    typeof value !== "object" ||
    value === null ||
    !("state" in value) ||
    typeof value.state !== "string"
  ) {
    throw new Error("ROS status needs a mission state");
  }
  const number = (key: string) => {
    const result: unknown = Reflect.get(value, key);
    if (typeof result !== "number" || !Number.isFinite(result))
      throw new Error(`Invalid ROS status ${key}`);
    return result;
  };
  const waypoint = number("waypoint");
  if (!Number.isInteger(waypoint) || waypoint < 0 || waypoint > 3)
    throw new Error("Invalid ROS status waypoint");
  const goal: unknown = Reflect.get(value, "goal");
  if (
    !Array.isArray(goal) ||
    goal.length !== 2 ||
    !goal.every((n) => typeof n === "number" && Number.isFinite(n))
  ) {
    throw new Error("Invalid ROS status goal");
  }
  return {
    state: value.state.charAt(0).toUpperCase() + value.state.slice(1),
    waypoint,
    goal: goal.join(", "),
    images: number("images"),
    scans: number("scans"),
    orange: number("orange_fraction"),
    distance: number("front_distance"),
  };
}
