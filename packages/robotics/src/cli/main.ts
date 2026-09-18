import { spawn } from "node:child_process";
import { Command } from "commander";
import { results } from "./recording.js";
import { sim } from "./sim.js";
import { snapshot } from "./snapshot.js";
import { urdf } from "./urdf.js";
import { view } from "./view.js";

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
  child.on("exit", (code, signal) => process.exit(code ?? (signal ? 1 : 0)));
  await new Promise(() => undefined);
}

new Command("drawcall-robotics")
  .showHelpAfterError()
  .addCommand(sim)
  .addCommand(results)
  .addCommand(urdf)
  .addCommand(view)
  .addCommand(snapshot)
  .parseAsync(process.argv)
  .catch((error: unknown) => {
    console.error(error);
    process.exit(1);
  });
