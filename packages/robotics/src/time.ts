export function stamp(seconds: number): { sec: number; nanosec: number } {
  if (!Number.isFinite(seconds) || seconds < 0)
    throw new Error("Simulation time must be finite and nonnegative");
  const nanoseconds = Math.round(seconds * 1e9);
  return { sec: Math.floor(nanoseconds / 1e9), nanosec: nanoseconds % 1e9 };
}
export class Schedule {
  private next = 0;
  readonly period: number;
  constructor(rate: number) {
    if (!Number.isFinite(rate) || rate <= 0)
      throw new Error("Sensor rate must be positive");
    this.period = 1 / rate;
  }
  due(time: number): boolean {
    if (time + 1e-9 < this.next) return false;
    this.next = (Math.floor((time + 1e-9) / this.period) + 1) * this.period;
    return true;
  }
}
