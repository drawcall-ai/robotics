#!/usr/bin/env node
import { spawn } from "node:child_process";
import { Command, CommanderError } from "commander";
import { kill } from "./episode.js";
import { UserError } from "./errors.js";
import { cat } from "./recording.js";
import { resume, run, stop } from "./run.js";
import { sim } from "./sim.js";
import { snapshot } from "./snapshot.js";
import { urdf } from "./urdf.js";
import { view } from "./view.js";

// Runs side by side keep apart by ROS domain, which only works with discovery kept to this machine.
process.env.ROS_AUTOMATIC_DISCOVERY_RANGE = "LOCALHOST";

// rclnodejs opens ROS type-support libraries by name, which needs DYLD_LIBRARY_PATH on macOS. Every shell
// on the way strips that variable, so inside a conda or pixi environment the CLI relaunches itself with it
// and stays a transparent parent: signals go to the child and its exit status becomes ours.
const prefix = process.env.CONDA_PREFIX;
if (process.platform === "darwin" && prefix && !process.env.DYLD_LIBRARY_PATH) {
  const child = spawn(process.execPath, process.argv.slice(1), {
    stdio: "inherit",
    env: {
      ...process.env,
      DYLD_LIBRARY_PATH: `${prefix}/lib`,
    },
  });
  for (const signal of ["SIGINT", "SIGTERM"] as const)
    process.on(signal, () => child.kill(signal));
  child.on("exit", (code, signal) => process.exit(code ?? (signal ? 2 : 0)));
  await new Promise(() => undefined);
}

// A policy command after `--` belongs to run or continue, not to the parser.
const split = process.argv.indexOf("--");
const argv = split < 0 ? process.argv : process.argv.slice(0, split);
const command = split < 0 ? undefined : process.argv.slice(split + 1);

// drawcall runs this CLI through npx as one of its own subcommands, and names it so.
const invocation = process.env.DRAWCALL_INVOCATION || "npx @drawcall/robotics";

// Exit codes: 0 ran, 1 below --min-score, 2 a crash or a usage error.
const commands = [
  run(command, invocation),
  resume(command, invocation),
  stop,
  cat,
  urdf,
  view,
  snapshot,
];
const program = new Command(invocation)
  .showHelpAfterError()
  .exitOverride()
  .addCommand(sim.exitOverride(), { hidden: true });
for (const subcommand of commands)
  program.addCommand(subcommand.exitOverride());
program.parseAsync(argv).catch((error: unknown) => {
  // Commander has printed its own errors, and help exits with 0.
  if (error instanceof CommanderError)
    process.exit(error.exitCode === 0 ? 0 : 2);
  // Whatever an episode left running when it failed.
  kill();
  console.error(error instanceof UserError ? `error: ${error.message}` : error);
  process.exit(2);
});
