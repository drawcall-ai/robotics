/** Runs every step and throws one AggregateError afterwards if any failed, so one failure never skips the rest. */
export function runAll(steps: Iterable<() => void>, message: string): void {
  const errors: unknown[] = [];
  for (const step of steps) {
    try {
      step();
    } catch (error) {
      errors.push(error);
    }
  }
  if (errors.length) throw new AggregateError(errors, message);
}
