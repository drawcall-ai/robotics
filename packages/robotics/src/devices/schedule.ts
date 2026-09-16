/** Fires once per period of simulation time, without a burst after a gap and from scratch after the clock resets. */
export class Schedule {
  private last = -Infinity;
  readonly period: number;
  constructor(rate: number) {
    if (!Number.isFinite(rate) || rate <= 0)
      throw new Error("Rate must be positive");
    this.period = 1 / rate;
  }
  due(time: number): boolean {
    if (time >= this.last && time - this.last + 1e-9 < this.period)
      return false;
    this.last = Math.floor((time + 1e-9) / this.period) * this.period;
    return true;
  }
}
