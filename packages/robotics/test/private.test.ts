import { mkdir, mkdtemp, readdir, rm, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { afterEach, expect, it, vi } from "vitest";
import { Evaluator, EVALUATION_TOPIC } from "../src/evaluation.js";
import { Robot } from "../src/devices/robot.js";
import { messageDefinition, PrivateRecording } from "../src/cli/private.js";
import {
  decodeMessages,
  openRecording,
  readResult,
} from "../src/cli/recording.js";
import { RevoluteJoint, RigidBody } from "@drawcall/physics";
import { JointActuator } from "../src/devices/actuator.js";
import type { ROS2 } from "../src/runtime.js";
import { simulation } from "./fixture.js";

/** A private one-joint arm; with `actuated`, its joint has a JointActuator. */
function arm(ros: ROS2, actuated = false) {
  const robot = new Robot({ ros, private: true });
  robot.name = "arm";
  const base = new RigidBody({ bodyType: "static" });
  base.name = "base_link";
  const link = new RigidBody({
    mass: 1,
    centerOfMass: [0, 0, 0],
    diagonalInertia: [1, 1, 1],
  });
  link.name = "link";
  const joint = new RevoluteJoint({ body0: base, body1: link, axis: "Z" });
  joint.name = "joint";
  if (actuated)
    joint.setDrive(
      new JointActuator({ ros, interface: "velocity", damping: 1 }),
    );
  robot.add(base, link, joint);
  return robot;
}

const temporary: string[] = [];
afterEach(async () => {
  vi.unstubAllEnvs();
  for (const dir of temporary.splice(0)) await rm(dir, { recursive: true });
});
async function directory() {
  const dir = await mkdtemp(join(tmpdir(), "private-"));
  temporary.push(dir);
  return dir;
}

it("records private messages without sending them to ROS", async () => {
  const { world, ros, connection } = await simulation();
  const written: { topic: string; time: number }[] = [];
  ros.attachRecording({
    write: (topic, _type, _message, time) => written.push({ topic, time }),
  });
  world.scene.add(arm(ros));
  const evaluation = new Evaluator({
    ros,
    private: true,
    timeout: 1,
    success: {},
    score: () => 0,
  });
  evaluation.start();
  world.update(0.01);
  expect(connection.messages.map(({ name }) => name)).not.toEqual(
    expect.arrayContaining([EVALUATION_TOPIC]),
  );
  expect(connection.messages.filter(({ name }) => name !== "/clock")).toEqual(
    [],
  );
  expect(connection.receivers.has("/robot/joint_commands")).toBe(false);
  expect(new Set(written.map(({ topic }) => topic))).toEqual(
    new Set([
      "/robot/robot_description",
      "/tf_static",
      "/tf",
      "/robot/joint_states",
      "/sim/evaluation",
      "/sim/score",
    ]),
  );
  expect(written.at(-1)?.time).toBeCloseTo(0.01);
});

it("rejects JointActuators on a private robot, which takes no commands", async () => {
  const { world, ros } = await simulation();
  world.scene.add(arm(ros, true));
  expect(() => world.update(0.01)).toThrow("takes no joint commands");
});

async function ros2Prefix() {
  const prefix = await directory();
  const msg = (pkg: string, name: string, text: string) =>
    mkdir(join(prefix, "share", pkg, "msg"), { recursive: true }).then(() =>
      writeFile(join(prefix, "share", pkg, "msg", `${name}.msg`), text),
    );
  await msg("std_msgs", "String", "string data\n");
  await msg(
    "std_msgs",
    "Header",
    "builtin_interfaces/Time stamp\nstring frame_id\n",
  );
  await msg("builtin_interfaces", "Time", "int32 sec\nuint32 nanosec\n");
  await msg(
    "test_msgs",
    "Stamped",
    "# a comment\nstd_msgs/Header header\nHeader[] more\nfloat64[<=3] v\n",
  );
  await msg("test_msgs", "Header", "uint8 KIND=1\nstring<=8 name\n");
  vi.stubEnv("AMENT_PREFIX_PATH", `/nowhere:${prefix}`);
}

it("defines a message with every type it uses, once, as rosbag2 does", async () => {
  await ros2Prefix();
  const separator = `${"=".repeat(80)}\n`;
  expect(messageDefinition("test_msgs/msg/Stamped").split(separator)).toEqual([
    "# a comment\nstd_msgs/Header header\nHeader[] more\nfloat64[<=3] v\n\n",
    "MSG: std_msgs/Header\nbuiltin_interfaces/Time stamp\nstring frame_id\n\n",
    "MSG: builtin_interfaces/Time\nint32 sec\nuint32 nanosec\n\n",
    "MSG: test_msgs/Header\nuint8 KIND=1\nstring<=8 name\n",
  ]);
});

it("writes split, readable MCAP files beside the bag", async () => {
  await ros2Prefix();
  const folder = await directory();
  const recording = new PrivateRecording();
  const snapshot = (status: string, elapsed: number) => ({
    data: JSON.stringify({
      stamp: { sec: elapsed, nanosec: 0 },
      elapsed,
      status,
      reason: "",
      metrics: { score: elapsed },
    }),
  });
  const type = "std_msgs/msg/String";
  // Written before the folder exists: kept until it does.
  recording.write(EVALUATION_TOPIC, type, snapshot("running", 0), 0);
  recording.start(folder);
  recording.write("/sim/audit", type, { data: "a" }, 0.5);
  await recording.split();
  recording.write(EVALUATION_TOPIC, type, snapshot("success", 2), 2);
  await recording.close();
  expect(() => recording.write("/sim/audit", type, { data: "b" }, 3)).toThrow(
    "closed",
  );
  expect((await readdir(folder)).sort()).toEqual([
    "private_0.mcap",
    "private_1.mcap",
  ]);
  expect((await readResult(folder)).status).toBe("success");
  await using first = await openRecording(join(folder, "private_0.mcap"));
  const messages = [];
  for await (const { topic, logTime, msg } of decodeMessages(first.reader))
    messages.push({ topic, logTime, msg });
  expect(messages.map(({ topic, logTime }) => [topic, logTime])).toEqual([
    [EVALUATION_TOPIC, 0n],
    ["/sim/audit", 500_000_000n],
  ]);
});
