/** Runs every task and throws one AggregateError afterwards if any failed, so one failure never skips the rest. */
export function runAll(tasks: Iterable<() => void>, message: string): void {
  const errors: unknown[] = [];
  for (const task of tasks) {
    try {
      task();
    } catch (error) {
      errors.push(error);
    }
  }
  if (errors.length) throw new AggregateError(errors, message);
}
