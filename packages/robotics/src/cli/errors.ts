/**
 * A failure the user can act on, such as a missing path or an interrupt: the CLI prints its message
 * as one line, as commander prints a usage error, and exits with 2. Any other error is a crash and
 * keeps its stack.
 */
export class UserError extends Error {}
