import {
  mkdir,
  mkdtemp,
  readFile,
  rm,
  symlink,
  truncate,
  writeFile,
} from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { Readable } from "node:stream";
import { McapWriter } from "@mcap/core";
import { MessageWriter } from "@foxglove/rosmsg2-serialization";
import { parse } from "@foxglove/rosmsg";
import { afterEach, expect, it, vi } from "vitest";
import {
  EVALUATION_TOPIC,
  EVALUATION_TYPE,
  type EvaluationState,
} from "../src/evaluation.js";
import { cat, readResult } from "../src/cli/recording.js";
import { view } from "../src/cli/view.js";

const serve = vi.hoisted(() => vi.fn());
vi.mock("../src/cli/lichtblick.js", () => ({ serveLichtblick: serve }));

const temporary: string[] = [];
const cwd = process.cwd();
const stdin = Object.getOwnPropertyDescriptor(process, "stdin");
afterEach(async () => {
  vi.restoreAllMocks();
  process.chdir(cwd);
  if (stdin) Object.defineProperty(process, "stdin", stdin);
  for (const dir of temporary.splice(0)) await rm(dir, { recursive: true });
});
async function directory() {
  const dir = await mkdtemp(join(tmpdir(), "recording-"));
  temporary.push(dir);
  return dir;
}

const SCHEMAS: Record<string, string> = {
  [EVALUATION_TYPE]: "string data\n",
  "sensor_msgs/msg/CompressedImage": "string format\nuint8[] data\n",
  "test_msgs/msg/Blob": "uint8[] data\nint64 count\nfloat32[] values\n",
};
type Message = { topic: string; schema: string; t: number; msg: unknown };

async function record(file: string, messages: Message[]) {
  const chunks: Uint8Array[] = [];
  let size = 0n;
  const writer = new McapWriter({
    useChunks: true,
    writable: {
      position: () => size,
      write: async (data) => {
        chunks.push(data.slice());
        size += BigInt(data.length);
      },
    },
  });
  await writer.start({ profile: "ros2", library: "test" });
  const channels = new Map<string, number>();
  for (const [sequence, { topic, schema, t, msg }] of messages.entries()) {
    const definition = SCHEMAS[schema];
    if (!definition) throw new Error(`No definition for ${schema}`);
    let channelId = channels.get(topic);
    if (channelId === undefined) {
      const schemaId = await writer.registerSchema({
        name: schema,
        encoding: "ros2msg",
        data: new TextEncoder().encode(definition),
      });
      channelId = await writer.registerChannel({
        topic,
        schemaId,
        messageEncoding: "cdr",
        metadata: new Map(),
      });
      channels.set(topic, channelId);
    }
    const time = BigInt(Math.round(t * 1e9));
    await writer.addMessage({
      channelId,
      sequence,
      logTime: time,
      publishTime: time,
      data: new MessageWriter(parse(definition, { ros2: true })).writeMessage(
        msg,
      ),
    });
  }
  await writer.end();
  await writeFile(file, Buffer.concat(chunks));
}

function state(
  status: EvaluationState["status"],
  elapsed = 0,
): EvaluationState {
  return {
    stamp: { sec: elapsed, nanosec: 0 },
    elapsed,
    status,
    reason: status === "running" ? "" : "done",
    metrics: { score: elapsed * 10 },
  };
}
function evaluations(states: EvaluationState[]): Message[] {
  return states.map((snapshot) => ({
    topic: EVALUATION_TOPIC,
    schema: EVALUATION_TYPE,
    t: snapshot.stamp.sec + snapshot.stamp.nanosec / 1e9,
    msg: { data: JSON.stringify(snapshot) },
  }));
}
function log(topic: string, t: number, data: string): Message {
  return { topic, schema: EVALUATION_TYPE, t, msg: { data } };
}

it("decodes final numeric metrics offline from a split ROS MCAP recording", async () => {
  const dir = await directory();
  await record(join(dir, "run_0.mcap"), evaluations([state("running")]));
  await record(join(dir, "run_1.mcap"), evaluations([state("success", 10)]));
  expect(await readResult(dir)).toEqual(state("success", 10));
});
it("rejects incomplete recordings and backwards evaluation time", async () => {
  const dir = await directory();
  const file = join(dir, "run.mcap");
  await record(file, evaluations([state("running")]));
  await expect(readResult(file)).rejects.toThrow("Incomplete");
  await record(
    file,
    evaluations([
      { ...state("running", 2), elapsed: 1 },
      { ...state("success", 3), elapsed: 0 },
    ]),
  );
  await expect(readResult(file)).rejects.toThrow("backwards");
});
it("rejects missing score and changes after completion", async () => {
  const dir = await directory();
  const file = join(dir, "run.mcap");
  await record(file, evaluations([{ ...state("success"), metrics: {} }]));
  await expect(readResult(file)).rejects.toThrow("score is missing");
  await record(file, evaluations([state("success"), state("success", 1)]));
  await expect(readResult(file)).rejects.toThrow("after its terminal");
});

/** A run folder with a split bag: /sim/evaluation and /rosout at 1..4 s, /chatter at 2 s. */
async function run() {
  const folder = await directory();
  await mkdir(join(folder, "bag"));
  await record(join(folder, "bag", "bag_0.mcap"), [
    log("/sim/evaluation", 1, "e1"),
    log("/rosout", 1.5, "r1"),
    log("/chatter", 2, "c"),
  ]);
  await record(join(folder, "bag", "bag_1.mcap"), [
    log("/sim/evaluation", 3, "e3"),
    log("/rosout", 4, "r4"),
  ]);
  return folder;
}
async function lines(...args: string[]): Promise<unknown[]> {
  const printed: unknown[] = [];
  vi.spyOn(console, "log").mockImplementation((line: string) => {
    printed.push(JSON.parse(line));
  });
  await cat.parseAsync(args, { from: "user" });
  return printed;
}
function pipe(input: string) {
  Object.defineProperty(process, "stdin", {
    value: Readable.from([input]),
    configurable: true,
  });
}

it("prints the evaluation and rosout of runs/latest by default", async () => {
  const folder = await run();
  const dir = await directory();
  await mkdir(join(dir, "runs"));
  await symlink(folder, join(dir, "runs", "latest"));
  process.chdir(dir);
  pipe("");
  expect(await lines()).toEqual([
    { t: 1, topic: "/sim/evaluation", msg: { data: "e1" } },
    { t: 1.5, topic: "/rosout", msg: { data: "r1" } },
    { t: 3, topic: "/sim/evaluation", msg: { data: "e3" } },
    { t: 4, topic: "/rosout", msg: { data: "r4" } },
  ]);
});

it("filters topics and a time window counted back from the end", async () => {
  const folder = await run();
  expect(await lines(folder, "--topics", "/chatter,/rosout")).toEqual([
    { t: 1.5, topic: "/rosout", msg: { data: "r1" } },
    { t: 2, topic: "/chatter", msg: { data: "c" } },
    { t: 4, topic: "/rosout", msg: { data: "r4" } },
  ]);
  expect(await lines(folder, "--start", "-2.5", "--end", "3")).toEqual([
    { t: 1.5, topic: "/rosout", msg: { data: "r1" } },
    { t: 3, topic: "/sim/evaluation", msg: { data: "e3" } },
  ]);
});

it("labels each line with its run when reading several folders from stdin", async () => {
  const a = await run();
  const b = await run();
  pipe(`${a}\n${b}\n`);
  const printed = await lines("--start", "-0.5");
  expect(printed).toEqual([
    { t: 4, topic: "/rosout", msg: { data: "r4" }, run: a },
    { t: 4, topic: "/rosout", msg: { data: "r4" }, run: b },
  ]);
  expect(await lines(a, b, "--start", "-0.5")).toEqual(printed);
});

it("skips the last file of a paused run's bag, which is still being recorded", async () => {
  const folder = await run();
  const file = join(folder, "bag", "bag_2.mcap");
  await record(file, [log("/rosout", 5, "r5")]);
  await truncate(file, 100);
  const errors = vi.spyOn(console, "error").mockImplementation(() => {});
  expect(await lines(folder, "--start", "-0.5")).toEqual([
    { t: 4, topic: "/rosout", msg: { data: "r4" } },
  ]);
  expect(errors).toHaveBeenCalledWith(`skipping ${file}: still being recorded`);
  await truncate(join(folder, "bag", "bag_0.mcap"), 100);
  await expect(lines(folder)).rejects.toThrow("MCAP magic");
});

it("reads each episode of a batch folder as its own run", async () => {
  const batch = await directory();
  for (const seed of ["2", "10"]) {
    await mkdir(join(batch, seed, "bag"), { recursive: true });
    await record(join(batch, seed, "bag", "bag_0.mcap"), [
      log("/rosout", 1, seed),
    ]);
  }
  expect(await lines(batch)).toEqual([
    { t: 1, topic: "/rosout", msg: { data: "2" }, run: join(batch, "2") },
    { t: 1, topic: "/rosout", msg: { data: "10" }, run: join(batch, "10") },
  ]);
});

it("writes compressed images next to a given file and summarizes byte arrays", async () => {
  const folder = await directory();
  const jpeg = Uint8Array.from([0xff, 0xd8, 0xff, 0xd9]);
  const file = join(folder, "bag.mcap");
  await record(file, [
    {
      topic: "/camera/compressed",
      schema: "sensor_msgs/msg/CompressedImage",
      t: 1.25,
      msg: { format: "rgb8; jpeg compressed bgr8", data: jpeg },
    },
    {
      topic: "/blob",
      schema: "test_msgs/msg/Blob",
      t: 2,
      msg: {
        data: new Uint8Array(65),
        count: 7n,
        values: Float32Array.from([0.5]),
      },
    },
    {
      topic: "/blob",
      schema: "test_msgs/msg/Blob",
      t: 3,
      msg: { data: Uint8Array.from([1, 2]), count: 0n, values: [] },
    },
  ]);
  const image = join(folder, "images", "1.250.jpeg");
  expect(await lines(file, "--topics", "/camera/compressed,/blob")).toEqual([
    { t: 1.25, topic: "/camera/compressed", image },
    {
      t: 2,
      topic: "/blob",
      msg: { data: { bytes: 65 }, count: 7, values: [0.5] },
    },
    { t: 3, topic: "/blob", msg: { data: [1, 2], count: 0, values: [] } },
  ]);
  expect(new Uint8Array(await readFile(image))).toEqual(jpeg);
});

it("views a split recording from its run or bag folder, without the file still being recorded", async () => {
  const folder = await run();
  const live = join(folder, "bag", "bag_2.mcap");
  await record(live, [log("/rosout", 5, "r5")]);
  await truncate(live, 100);
  const errors = vi.spyOn(console, "error").mockImplementation(() => {});
  const files = ["bag_0.mcap", "bag_1.mcap"].map((name) =>
    join(folder, "bag", name),
  );
  for (const path of [folder, join(folder, "bag")]) {
    serve.mockReset().mockResolvedValue({ url: "", close: () => {} });
    const viewing = view.parseAsync([path], { from: "user" });
    await vi.waitFor(() => expect(serve).toHaveBeenCalled());
    process.emit("SIGINT");
    await viewing;
    expect(serve).toHaveBeenCalledWith(expect.anything(), {
      recordings: files,
    });
  }
  expect(errors).toHaveBeenCalledWith(`skipping ${live}: still being recorded`);
});
