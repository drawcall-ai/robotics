import {
  closeSync,
  existsSync,
  openSync,
  readFileSync,
  writeSync,
} from "node:fs";
import { join } from "node:path";
import { McapWriter } from "@mcap/core";
import { parse } from "@foxglove/rosmsg";
import { MessageWriter } from "@foxglove/rosmsg2-serialization";
import type { Recording } from "../runtime.js";

interface Entry {
  topic: string;
  type: string;
  data: Uint8Array;
  logTime: bigint;
}

interface File {
  descriptor: number;
  writer: McapWriter;
  channels: Map<string, number>;
  schemas: Map<string, number>;
}

/**
 * The privately published messages of an episode, as MCAP files beside rosbag2's in its bag
 * folder: private_0.mcap, then a new file after each split, so that everything up to a pause is
 * readable. Messages written before `start` wait for the folder.
 */
export class PrivateRecording implements Recording {
  private folder?: string;
  private queued: Entry[] = [];
  private file?: File;
  private files = 0;
  private closed = false;
  private failure?: unknown;
  /** McapWriter calls must not overlap: every file operation runs on this chain. */
  private chain = Promise.resolve();
  private readonly definitions = new Map<
    string,
    { text: string; writer: MessageWriter }
  >();

  write(topic: string, type: string, message: unknown, time: number): void {
    this.check();
    if (this.closed)
      throw new Error(`Cannot record ${topic}: recording is closed`);
    const data = this.definition(type).writer.writeMessage(message);
    this.queued.push({
      topic,
      type,
      data,
      logTime: BigInt(Math.round(time * 1e9)),
    });
    if (this.folder) this.drain();
  }
  /** Starts writing into `folder`, which must exist. */
  start(folder: string): void {
    if (!existsSync(folder)) throw new Error(`${folder} does not exist`);
    this.folder = folder;
    this.drain();
  }
  /** Closes the current file; the next message starts another. */
  async split(): Promise<void> {
    this.run(() => this.end());
    await this.chain;
    this.check();
  }
  /** Writes what is queued and closes the last file. */
  async close(): Promise<void> {
    if (!this.closed && this.folder) {
      this.drain();
      this.run(() => this.end());
    }
    this.closed = true;
    await this.chain;
    this.check();
  }
  check(): void {
    if (this.failure) throw this.failure;
  }
  private drain(): void {
    const entries = this.queued.splice(0);
    this.run(async () => {
      for (const entry of entries) await this.add(entry);
    });
  }
  private run(task: () => Promise<void>): void {
    this.chain = this.chain.then(task).catch((error: unknown) => {
      this.failure ??= error;
    });
  }
  private async add({ topic, type, data, logTime }: Entry): Promise<void> {
    const file = (this.file ??= await this.create());
    let channel = file.channels.get(topic);
    if (channel === undefined) {
      let schema = file.schemas.get(type);
      if (schema === undefined) {
        schema = await file.writer.registerSchema({
          name: type,
          encoding: "ros2msg",
          data: new TextEncoder().encode(this.definition(type).text),
        });
        file.schemas.set(type, schema);
      }
      channel = await file.writer.registerChannel({
        topic,
        schemaId: schema,
        messageEncoding: "cdr",
        metadata: new Map(),
      });
      file.channels.set(topic, channel);
    }
    await file.writer.addMessage({
      channelId: channel,
      sequence: 0,
      logTime,
      publishTime: logTime,
      data,
    });
  }
  private async create(): Promise<File> {
    if (!this.folder) throw new Error("Private recording has not started");
    const descriptor = openSync(
      join(this.folder, `private_${this.files++}.mcap`),
      "wx",
    );
    let offset = 0n;
    const writer = new McapWriter({
      writable: {
        position: () => offset,
        write: async (buffer) => {
          writeSync(descriptor, buffer);
          offset += BigInt(buffer.byteLength);
        },
      },
    });
    await writer.start({ profile: "ros2", library: "drawcall-robotics" });
    return { descriptor, writer, channels: new Map(), schemas: new Map() };
  }
  private async end(): Promise<void> {
    const file = this.file;
    if (!file) return;
    this.file = undefined;
    try {
      await file.writer.end();
    } finally {
      closeSync(file.descriptor);
    }
  }
  private definition(type: string) {
    let definition = this.definitions.get(type);
    if (!definition) {
      const text = messageDefinition(type);
      definition = {
        text,
        writer: new MessageWriter(parse(text, { ros2: true })),
      };
      this.definitions.set(type, definition);
    }
    return definition;
  }
}

const PRIMITIVES = new Set([
  "bool",
  "byte",
  "char",
  "float32",
  "float64",
  "int8",
  "uint8",
  "int16",
  "uint16",
  "int32",
  "uint32",
  "int64",
  "uint64",
  "string",
  "wstring",
]);

/**
 * The full definition of a message type, such as `sensor_msgs/msg/JointState`, as rosbag2 records
 * it: its .msg file from the ROS installation, then each type it uses, once, after a separator.
 */
export function messageDefinition(type: string): string {
  const seen = new Set<string>();
  const parts: string[] = [];
  const visit = (name: string) => {
    if (seen.has(name)) return;
    seen.add(name);
    const [pkg = "", message = ""] = name.split("/");
    const text = msgFile(pkg, message);
    parts.push(
      parts.length
        ? `${"=".repeat(80)}\nMSG: ${pkg}/${message}\n${text}`
        : text,
    );
    for (const line of text.split("\n")) {
      const field = line.replace(/#.*/, "").trim().split(/\s+/)[0];
      if (!field) continue;
      const base = field.replace(/\[.*\]$/, "").replace(/<=\d+$/, "");
      if (PRIMITIVES.has(base)) continue;
      const segments = base.split("/");
      visit(
        segments.length === 1
          ? `${pkg}/${base}`
          : `${segments[0]}/${segments[segments.length - 1]}`,
      );
    }
  };
  const [pkg, kind, message] = type.split("/");
  if (!pkg || kind !== "msg" || !message)
    throw new Error(`Not a message type: ${type}`);
  visit(`${pkg}/${message}`);
  return parts.join("\n");
}

function msgFile(pkg: string, message: string): string {
  const prefixes = (process.env.AMENT_PREFIX_PATH ?? "").split(":");
  for (const prefix of prefixes.filter(Boolean)) {
    const path = join(prefix, "share", pkg, "msg", `${message}.msg`);
    if (existsSync(path)) return readFileSync(path, "utf8");
  }
  throw new Error(
    `No definition of ${pkg}/msg/${message} under AMENT_PREFIX_PATH; source ROS 2 first`,
  );
}
