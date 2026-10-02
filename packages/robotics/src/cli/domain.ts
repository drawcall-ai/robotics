import {
  linkSync,
  mkdirSync,
  readFileSync,
  renameSync,
  rmSync,
  writeFileSync,
} from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";

/**
 * ROS domains for runs side by side on one machine. A run claims one by creating its lock file,
 * which holds the pid of the process that keeps the domain busy; a lock whose process is gone is
 * stale and claimed again. Domains 1-101 are safe on every platform; 0, the default, is left to
 * interactive use.
 */
export function claim(pid = process.pid): number {
  mkdirSync(locks(), { recursive: true });
  for (let domain = 1; domain <= 101; domain++) {
    const file = lock(domain);
    if (create(file, pid) || takeover(file, pid)) return domain;
  }
  throw new Error("Every ROS domain from 1 to 101 is in use");
}

/** Hands a claimed domain to the process that now keeps it busy. */
export function hold(domain: number, pid: number): void {
  writeFileSync(lock(domain), String(pid));
}

/** Whether a process exists; one owned by another user does too. */
export function alive(pid: number): boolean {
  if (!Number.isInteger(pid) || pid <= 0) return false;
  try {
    process.kill(pid, 0);
    return true;
  } catch (error) {
    return error instanceof Error && "code" in error && error.code === "EPERM";
  }
}

/**
 * Claims a stale lock. Of the processes that found it stale, the one that moves it aside claims
 * the domain; one that moved a fresh lock instead, created since, puts it back.
 */
function takeover(file: string, pid: number): boolean {
  const stale = read(file);
  // Empty while its creator is still writing it.
  if (stale === undefined || stale === "" || alive(Number(stale))) return false;
  const aside = `${file}.${process.pid}`;
  try {
    renameSync(file, aside);
  } catch (error) {
    if (code(error) === "ENOENT") return false;
    throw error;
  }
  if (read(aside) !== stale) {
    linkSync(aside, file);
    rmSync(aside);
    return false;
  }
  rmSync(aside);
  return create(file, pid);
}

function create(file: string, pid: number): boolean {
  try {
    writeFileSync(file, String(pid), { flag: "wx" });
    return true;
  } catch (error) {
    if (code(error) === "EEXIST") return false;
    throw error;
  }
}

function read(file: string): string | undefined {
  try {
    return readFileSync(file, "utf8");
  } catch (error) {
    if (code(error) === "ENOENT") return undefined;
    throw error;
  }
}

function code(error: unknown): unknown {
  return error instanceof Error && "code" in error ? error.code : undefined;
}

function locks(): string {
  return join(tmpdir(), "drawcall-robotics");
}

function lock(domain: number): string {
  return join(locks(), `domain-${domain}`);
}
