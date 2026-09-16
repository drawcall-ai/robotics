export function stamp(seconds: number): { sec: number; nanosec: number } {
  if (!Number.isFinite(seconds) || seconds < 0)
    throw new Error("Simulation time must be finite and nonnegative");
  const nanoseconds = Math.round(seconds * 1e9);
  return { sec: Math.floor(nanoseconds / 1e9), nanosec: nanoseconds % 1e9 };
}
/** Fires once per period of simulation time, without a burst after a gap and from scratch after the clock resets. */
export class Schedule {
  private last = -Infinity;
  readonly period: number;
  constructor(rate: number) {
    if (!Number.isFinite(rate) || rate <= 0)
      throw new Error("Sensor rate must be positive");
    this.period = 1 / rate;
  }
  due(time: number): boolean {
    if (time >= this.last && time - this.last + 1e-9 < this.period)
      return false;
    this.last = Math.floor((time + 1e-9) / this.period) * this.period;
    return true;
  }
}
