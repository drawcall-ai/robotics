import { Command, InvalidArgumentError } from "commander";
import { McapIndexedReader } from "@mcap/core";
import { loadDecompressHandlers } from "@mcap/support";
import { MessageReader } from "@foxglove/rosmsg2-serialization";
import { parse } from "@foxglove/rosmsg";
import { existsSync } from "node:fs";
import { mkdir, open, readdir, stat, writeFile } from "node:fs/promises";
import { basename, dirname, join, resolve } from "node:path";
import { text } from "node:stream/consumers";
import {
  EVALUATION_STATUSES,
  EVALUATION_TOPIC,
  type EvaluationState,
  type EvaluationStatus,
} from "../evaluation.js";
import { UserError } from "./errors.js";

export async function recordingFiles(path: string): Promise<string[]> {
  if (!(await stat(path)).isDirectory()) return [path];
  const files = (await readdir(path, { recursive: true }))
    .filter((name) => name.endsWith(".mcap"))
    .sort((a, b) => a.localeCompare(b, "en", { numeric: true }));
  if (!files.length) throw new UserError(`${path} contains no MCAP files`);
  return files.map((file) => join(path, file));
}
export async function openRecording(path: string) {
  const handle = await open(path);
  try {
    const reader = await McapIndexedReader.Initialize({
      decompressHandlers: await loadDecompressHandlers(),
      readable: {
        size: async () => BigInt((await handle.stat()).size),
        read: async (offset, size) => {
          const buffer = new Uint8Array(Number(size));
          const { bytesRead } = await handle.read(
            buffer,
            0,
            buffer.length,
            Number(offset),
          );
          if (bytesRead !== buffer.length)
            throw new Error(`Truncated MCAP: ${path}`);
          return buffer;
        },
      },
    });
    return { reader, [Symbol.asyncDispose]: () => handle.close() };
  } catch (error) {
    await handle.close();
    throw error;
  }
}

/** A recorded message, decoded. */
export type Decoded = {
  topic: string;
  schema: string;
  logTime: bigint;
  msg: unknown;
};

/** Decode every `ros2msg`/cdr message recorded on `topics` (all topics if omitted), in log order. */
export async function* decodeMessages(
  reader: McapIndexedReader,
  options: { topics?: string[]; startTime?: bigint; endTime?: bigint } = {},
): AsyncGenerator<Decoded> {
  const decoders = new Map<number, MessageReader>();
  for await (const message of reader.readMessages({
    ...options,
    validateCrcs: true,
  })) {
    const channel = reader.channelsById.get(message.channelId);
    const schema = channel && reader.schemasById.get(channel.schemaId);
    if (
      !schema ||
      schema.encoding !== "ros2msg" ||
      channel.messageEncoding !== "cdr"
    )
      throw new Error(
        `Unsupported schema or encoding on ${channel?.topic ?? "a channel"}`,
      );
    let decoder = decoders.get(schema.id);
    if (!decoder) {
      decoder = new MessageReader(
        parse(new TextDecoder().decode(schema.data), { ros2: true }),
      );
      decoders.set(schema.id, decoder);
    }
    yield {
      topic: channel.topic,
      schema: schema.name,
      logTime: message.logTime,
      msg: decoder.readMessage(message.data),
    };
  }
}

/** The text of every `std_msgs/msg/String` recorded on `topics`, in log order. */
export async function* readStrings(
  reader: McapIndexedReader,
  topics: string[],
): AsyncGenerator<string> {
  for await (const { topic, schema, msg } of decodeMessages(reader, {
    topics,
  })) {
    if (schema !== "std_msgs/msg/String")
      throw new Error(`Expected std_msgs/msg/String on ${topic}`);
    if (
      typeof msg !== "object" ||
      msg === null ||
      !("data" in msg) ||
      typeof msg.data !== "string"
    )
      throw new Error(`Invalid string message on ${topic}`);
    yield msg.data;
  }
}

/** Read every evaluation snapshot to validate the single-episode contract, including split bags. */
export async function readResult(path: string): Promise<EvaluationState> {
  let last: EvaluationState | undefined;
  for (const file of await recordingFiles(path)) {
    await using recording = await openRecording(file);
    for await (const text of readStrings(recording.reader, [
      EVALUATION_TOPIC,
    ])) {
      const next = evaluationState(JSON.parse(text));
      if (last && next.elapsed < last.elapsed)
        throw new Error("Evaluation time moved backwards");
      if (
        last &&
        last.status !== "running" &&
        JSON.stringify(last) !== JSON.stringify(next)
      )
        throw new Error("Evaluation changed after its terminal snapshot");
      last = next;
    }
  }
  if (!last) throw new Error("Recording contains no evaluation snapshots");
  if (last.status === "running")
    throw new Error("Incomplete evaluation: terminal snapshot is missing");
  return last;
}

function evaluationState(value: unknown): EvaluationState {
  if (typeof value !== "object" || value === null)
    throw new Error("Invalid evaluation snapshot");
  if (
    !("elapsed" in value) ||
    typeof value.elapsed !== "number" ||
    !Number.isFinite(value.elapsed) ||
    value.elapsed < 0 ||
    !("status" in value) ||
    !isStatus(value.status) ||
    !("reason" in value) ||
    typeof value.reason !== "string" ||
    !("stamp" in value) ||
    typeof value.stamp !== "object" ||
    value.stamp === null ||
    !("sec" in value.stamp) ||
    typeof value.stamp.sec !== "number" ||
    !Number.isInteger(value.stamp.sec) ||
    value.stamp.sec < 0 ||
    !("nanosec" in value.stamp) ||
    typeof value.stamp.nanosec !== "number" ||
    !Number.isInteger(value.stamp.nanosec) ||
    value.stamp.nanosec < 0 ||
    value.stamp.nanosec >= 1e9 ||
    !("metrics" in value) ||
    typeof value.metrics !== "object" ||
    value.metrics === null ||
    Array.isArray(value.metrics)
  )
    throw new Error("Invalid evaluation snapshot");
  const metrics: Record<string, number> = {};
  for (const [name, metric] of Object.entries(value.metrics)) {
    if (!name || typeof metric !== "number" || !Number.isFinite(metric))
      throw new Error("Invalid evaluation metric");
    metrics[name] = metric;
  }
  if (!("score" in metrics)) throw new Error("Evaluation score is missing");
  return {
    stamp: { sec: value.stamp.sec, nanosec: value.stamp.nanosec },
    elapsed: value.elapsed,
    status: value.status,
    reason: value.reason,
    metrics,
  };
}
function isStatus(value: unknown): value is EvaluationStatus {
  return EVALUATION_STATUSES.some((status) => status === value);
}

const IMAGE_TYPE = "sensor_msgs/msg/CompressedImage";

export const cat = new Command("cat")
  .description("print the recorded messages of runs as JSON lines")
  .argument(
    "[paths...]",
    "run folders, folders of runs, or MCAP files; default: stdin lines, else runs/latest",
  )
  .option(
    "--topics <topics>",
    "comma-separated topics",
    "/sim/evaluation,/rosout",
  )
  .option(
    "--start <s>",
    "first log time in sim seconds; negative counts back from the end",
    seconds,
  )
  .option(
    "--end <s>",
    "last log time in sim seconds; negative counts back from the end",
    seconds,
  )
  .action(
    async (
      args: string[],
      options: { topics: string; start?: number; end?: number },
    ) => {
      const paths = args.length ? args : await inputPaths();
      const episodes = (await Promise.all(paths.map(findEpisodes))).flat();
      const topics = options.topics.split(",");
      for (const episode of episodes)
        for await (const line of catEpisode(episode, { ...options, topics }))
          console.log(
            JSON.stringify(
              episodes.length > 1 ? { ...line, run: episode.run } : line,
              plain,
            ),
          );
    },
  );

function seconds(value: string): number {
  const result = Number(value);
  if (!Number.isFinite(result)) throw new InvalidArgumentError("not a number");
  return result;
}

/** Paths listed one per line on a piped stdin, else the latest run. */
async function inputPaths(): Promise<string[]> {
  if (process.stdin.isTTY) return ["runs/latest"];
  const lines = (await text(process.stdin))
    .split("\n")
    .map((line) => line.trim())
    .filter(Boolean);
  return lines.length ? lines : ["runs/latest"];
}

/**
 * What `cat` reads as one run: an episode folder's `bag/`, or a single MCAP file.
 * Images go to `images/` in the episode folder, or next to the file.
 */
export type Episode = {
  run: string;
  files: string[];
  images: string;
  bag: boolean;
};

/**
 * A file, an episode's `bag/` folder, or every episode folder (one containing `bag/`) in a folder,
 * in numeric name order.
 */
export async function findEpisodes(path: string): Promise<Episode[]> {
  if (!existsSync(path)) throw new UserError(`${path} does not exist`);
  if (!(await stat(path)).isDirectory())
    return [
      {
        run: path,
        files: [path],
        images: join(dirname(path), "images"),
        bag: false,
      },
    ];
  if (basename(resolve(path)) === "bag") return [await episode(dirname(path))];
  const folders = (
    await readdir(path, { recursive: true, withFileTypes: true })
  )
    .filter((entry) => entry.isDirectory() && entry.name === "bag")
    .map((entry) => entry.parentPath)
    .sort((a, b) => a.localeCompare(b, "en", { numeric: true }));
  if (!folders.length) throw new UserError(`${path} contains no bag folder`);
  return Promise.all(folders.map(episode));
}

async function episode(run: string): Promise<Episode> {
  return {
    run,
    files: await recordingFiles(join(run, "bag")),
    images: join(run, "images"),
    bag: true,
  };
}

/**
 * Opens an episode's MCAP files into `recordings`, in name order. rosbag2 splits the bag at each
 * pause and writes the index of its last file only at the end, so a last file still being
 * recorded is skipped with a note.
 */
export async function openEpisode(
  episode: Episode,
  recordings: AsyncDisposableStack,
): Promise<{ file: string; reader: McapIndexedReader }[]> {
  const opened = [];
  for (const [index, file] of episode.files.entries()) {
    const recording = await openRecording(file).catch((error: unknown) => {
      if (!episode.bag || index < episode.files.length - 1) throw error;
      console.error(`skipping ${file}: still being recorded`);
    });
    if (recording)
      opened.push({ file, reader: recordings.use(recording).reader });
  }
  return opened;
}

/** The messages on `topics` in an episode's MCAP files, read file by file in name order. */
async function* catEpisode(
  episode: Episode,
  options: { topics: string[]; start?: number; end?: number },
): AsyncGenerator<{ t: number; topic: string; msg?: unknown; image?: string }> {
  await using recordings = new AsyncDisposableStack();
  const opened = await openEpisode(episode, recordings);
  let last = 0n;
  for (const { file, reader } of opened) {
    const end = reader.statistics?.messageEndTime;
    if (end === undefined) throw new Error(`${file} has no statistics`);
    if (end > last) last = end;
  }
  const time = (s: number | undefined) =>
    s === undefined
      ? undefined
      : BigInt(Math.round(s * 1e9)) + (s < 0 ? last : 0n);
  for (const { reader } of opened)
    for await (const { topic, schema, logTime, msg } of decodeMessages(reader, {
      topics: options.topics,
      startTime: time(options.start),
      endTime: time(options.end),
    })) {
      const t = Number(logTime) / 1e9;
      yield schema === IMAGE_TYPE
        ? { t, topic, image: await writeImage(episode.images, t, msg) }
        : { t, topic, msg };
    }
}

/** Write a CompressedImage to `<images>/<t>.<format>` and return its path. */
async function writeImage(
  images: string,
  t: number,
  msg: unknown,
): Promise<string> {
  if (
    typeof msg !== "object" ||
    msg === null ||
    !("format" in msg) ||
    typeof msg.format !== "string" ||
    !("data" in msg) ||
    !(msg.data instanceof Uint8Array)
  )
    throw new Error("Invalid CompressedImage");
  const { format } = msg;
  const extension = ["jpeg", "png"].find((name) => format.includes(name));
  if (!extension) throw new Error(`Unsupported image format ${format}`);
  const path = join(images, `${t.toFixed(3)}.${extension}`);
  await mkdir(images, { recursive: true });
  await writeFile(path, msg.data);
  return path;
}

type TypedArray =
  | Int8Array
  | Uint8Array
  | Uint8ClampedArray
  | Int16Array
  | Uint16Array
  | Int32Array
  | Uint32Array
  | Float32Array
  | Float64Array
  | BigInt64Array
  | BigUint64Array;
function isTypedArray(value: unknown): value is TypedArray {
  return ArrayBuffer.isView(value) && !(value instanceof DataView);
}

/** JSON replacer: bigints become numbers, typed arrays arrays, and byte arrays over 64 a size. */
function plain(_key: string, value: unknown): unknown {
  if (typeof value === "bigint") return Number(value);
  if (!isTypedArray(value)) return value;
  if (value.BYTES_PER_ELEMENT === 1 && value.length > 64)
    return { bytes: value.length };
  return Array.from<number | bigint>(value);
}
