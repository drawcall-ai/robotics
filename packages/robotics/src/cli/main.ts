import { spawn } from "node:child_process";
import { parseArgs } from "node:util";

// rclnodejs opens ROS type-support libraries by name, which needs DYLD_LIBRARY_PATH on macOS. Every shell
// on the way strips that variable, so inside a conda or pixi environment the CLI relaunches itself with it
// and stays a transparent parent: signals go to the child and its exit status becomes ours.
const prefix = process.env.CONDA_PREFIX;
if (process.platform === "darwin" && prefix && !process.env.DYLD_LIBRARY_PATH) {
  const child = spawn(process.execPath, process.argv.slice(1), {
    stdio: "inherit",
    env: { ...process.env, DYLD_LIBRARY_PATH: `${prefix}/lib` },
  });
  for (const signal of ["SIGINT", "SIGTERM"] as const)
    process.on(signal, () => child.kill(signal));
  child.on("exit", (code, signal) => process.exit(code ?? (signal ? 1 : 0)));
  await new Promise(() => undefined);
}

const usage = `drawcall-robotics <command>

  sim <scene>        run the scene as a ROS 2 node until stopped
      --namespace <ns>   topic namespace (default /robot)
      --robot <name>     the robot to report on /sim/state, if the scene has several
      --viewer           serve Lichtblick with a layout for the scene's topics and open it (BROWSER=none: serve only)
  urdf <scene>       print the URDF of the scene's robot
      --namespace <ns>   (default /robot)
      --robot <name>     the robot, if the scene has several
  view <recording>   open a rosbag2 directory or .mcap file in Lichtblick
  snapshot <topic> --out <file.jpg>   save the next compressed image
`;

async function main(argv: string[]): Promise<void> {
  const { values, positionals } = parseArgs({
    args: argv,
    allowPositionals: true,
    options: {
      namespace: { type: "string" },
      robot: { type: "string" },
      viewer: { type: "boolean", default: false },
      out: { type: "string" },
      help: { type: "boolean", default: false },
    },
  });
  const [command, target] = positionals;
  if (values.help || !command) {
    process.stdout.write(usage);
    return;
  }
  if (!target) throw new Error(`${command} needs an argument\n\n${usage}`);
  switch (command) {
    case "sim": {
      const { sim } = await import("./sim.js");
      await sim({
        scene: target,
        namespace: values.namespace,
        robot: values.robot,
        viewer: values.viewer,
      });
      return;
    }
    case "urdf": {
      const { urdf } = await import("./urdf.js");
      await urdf(target, values.namespace, values.robot);
      return;
    }
    case "view": {
      const { view } = await import("./view.js");
      await view(target);
      return;
    }
    case "snapshot": {
      if (!values.out) throw new Error("snapshot needs --out <file>");
      const { snapshot } = await import("./snapshot.js");
      await snapshot(target, values.out);
      return;
    }
    default:
      throw new Error(`Unknown command ${command}\n\n${usage}`);
  }
}

main(process.argv.slice(2)).catch((error: unknown) => {
  console.error(error instanceof Error ? error.message : error);
  process.exit(1);
});
