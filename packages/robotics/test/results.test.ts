import { mkdtemp, rm, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { McapWriter } from "@mcap/core";
import { MessageWriter } from "@foxglove/rosmsg2-serialization";
import { parse } from "@foxglove/rosmsg";
import { afterEach, expect, it } from "vitest";
import {
  EVALUATION_TOPIC,
  EVALUATION_TYPE,
  type EvaluationState,
} from "../src/evaluation.js";
import { readResult } from "../src/cli/recording.js";
const temporary: string[] = [];
afterEach(async () => {
  for (const dir of temporary.splice(0)) await rm(dir, { recursive: true });
});
async function directory() {
  const dir = await mkdtemp(join(tmpdir(), "evaluation-"));
  temporary.push(dir);
  return dir;
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
async function record(
  file: string,
  states: (
    | EvaluationState
    | {
        stamp: EvaluationState["stamp"];
        elapsed: number;
        status: EvaluationState["status"];
        reason: string;
        metrics: { name: string; value: number }[];
      }
  )[],
) {
  const definition = "string data\n";
  const serializer = new MessageWriter(parse(definition, { ros2: true }));
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
  const schemaId = await writer.registerSchema({
    name: EVALUATION_TYPE,
    encoding: "ros2msg",
    data: new TextEncoder().encode(definition),
  });
  const channelId = await writer.registerChannel({
    topic: EVALUATION_TOPIC,
    schemaId,
    messageEncoding: "cdr",
    metadata: new Map(),
  });
  for (const [sequence, snapshot] of states.entries())
    await writer.addMessage({
      channelId,
      sequence,
      logTime: BigInt(snapshot.stamp.sec * 1e9 + snapshot.stamp.nanosec),
      publishTime: BigInt(snapshot.stamp.sec * 1e9 + snapshot.stamp.nanosec),
      data: serializer.writeMessage({ data: JSON.stringify(snapshot) }),
    });
  await writer.end();
  await writeFile(file, Buffer.concat(chunks));
}
it("decodes final numeric metrics offline from a split ROS MCAP recording", async () => {
  const dir = await directory();
  await record(join(dir, "run_0.mcap"), [state("running")]);
  await record(join(dir, "run_1.mcap"), [state("success", 10)]);
  expect(await readResult(dir)).toEqual(state("success", 10));
});
it("rejects incomplete recordings and backwards evaluation time", async () => {
  const dir = await directory();
  const file = join(dir, "run.mcap");
  await record(file, [state("running")]);
  await expect(readResult(file)).rejects.toThrow("Incomplete");
  await record(file, [
    { ...state("running", 2), elapsed: 1 },
    { ...state("success", 3), elapsed: 0 },
  ]);
  await expect(readResult(file)).rejects.toThrow("backwards");
});
it("rejects missing score and changes after completion", async () => {
  const dir = await directory();
  const file = join(dir, "run.mcap");
  await record(file, [{ ...state("success"), metrics: {} }]);
  await expect(readResult(file)).rejects.toThrow("score is missing");
  await record(file, [state("success"), state("success", 1)]);
  await expect(readResult(file)).rejects.toThrow("after its terminal");
});

it("reads existing name/value-array recordings without retaining episode identity", async () => {
  const file = join(await directory(), "run.mcap");
  await record(file, [
    { ...state("success", 10), metrics: [{ name: "score", value: 100 }] },
  ]);
  expect(await readResult(file)).toEqual(state("success", 10));
});
