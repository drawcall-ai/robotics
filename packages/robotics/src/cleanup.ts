export function disposeAll(
  releases: Iterable<() => void>,
  message: string,
): void {
  const errors: unknown[] = [];
  for (const release of releases) {
    try {
      release();
    } catch (error) {
      errors.push(error);
    }
  }
  if (errors.length) throw new AggregateError(errors, message);
}
