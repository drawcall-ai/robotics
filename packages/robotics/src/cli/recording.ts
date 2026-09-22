import { Command } from "commander";
import { McapIndexedReader } from "@mcap/core";
import { loadDecompressHandlers } from "@mcap/support";
import { MessageReader } from "@foxglove/rosmsg2-serialization";
import { parse } from "@foxglove/rosmsg";
import { open, readdir, stat } from "node:fs/promises";
import { join } from "node:path";
import {
  EVALUATION_STATUSES,
  EVALUATION_TOPIC,
  type EvaluationState,
  type EvaluationStatus,
} from "../evaluation.js";

export async function recordingFiles(path: string): Promise<string[]> {
  if (!(await stat(path)).isDirectory()) return [path];
  const files = (await readdir(path))
    .filter((name) => name.endsWith(".mcap"))
    .sort((a, b) => a.localeCompare(b, "en", { numeric: true }));
  if (!files.length) throw new Error(`${path} contains no MCAP files`);
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

/** The text of every `std_msgs/msg/String` recorded on `topics`, in log order. */
export async function* readStrings(
  reader: McapIndexedReader,
  topics: string[],
): AsyncGenerator<string> {
  const decoders = new Map<number, MessageReader>();
  for await (const message of reader.readMessages({
    topics,
    validateCrcs: true,
  })) {
    const channel = reader.channelsById.get(message.channelId);
    const schema = channel && reader.schemasById.get(channel.schemaId);
    if (
      !schema ||
      schema.name !== "std_msgs/msg/String" ||
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
    const envelope: unknown = decoder.readMessage(message.data);
    if (
      typeof envelope !== "object" ||
      envelope === null ||
      !("data" in envelope) ||
      typeof envelope.data !== "string"
    )
      throw new Error(`Invalid string message on ${channel.topic}`);
    yield envelope.data;
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

export const results = new Command("results")
  .description(
    "print the final evaluation result from a single-episode MCAP recording",
  )
  .argument("<recording>", "a rosbag2 directory or .mcap file")
  .action(async (path: string) => {
    const state = await readResult(path);
    console.log(
      JSON.stringify(
        {
          status: state.status,
          reason: state.reason,
          elapsed: state.elapsed,
          metrics: state.metrics,
        },
        null,
        2,
      ),
    );
  });
