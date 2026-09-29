import { Quaternion, Vector3 } from "three";
import type { Kind } from "./pieces.ts";

export const deskHeight = 0.74;
const files = "abcdefgh";

/** Black's starting position; the robot plays black from the rank 8 side. */
export const setup: Record<string, Kind> = {};
for (const [index, kind] of (
  [
    "rook",
    "knight",
    "bishop",
    "queen",
    "king",
    "bishop",
    "knight",
    "rook",
  ] as const
).entries()) {
  setup[`${files[index]}8`] = kind;
  setup[`${files[index]}7`] = "pawn";
}
/** The black set, one entry per piece. */
export const pieces: Kind[] = Object.values(setup);

export interface Board {
  /** Square size, metres. */
  s: number;
  border: number;
  thickness: number;
  /** Centre of the board top in the robot's ROS frame, +X forward, +Y left. */
  x: number;
  y: number;
  /** Turn of the files axis from the robot's -Y, radians counterclockwise. */
  yaw: number;
}

/**
 * The centre of a square in the robot's ROS frame. Rank 8 is nearest the robot and the a-file is
 * on its right, as black sees the board, so a8 is a light square on the right.
 */
export function squareCenter(board: Board, name: string): [number, number] {
  const file = files.indexOf(name[0] ?? "");
  const rank = Number(name[1]);
  if (file < 0 || !(rank >= 1 && rank <= 8))
    throw new Error(`Bad square ${name}`);
  return boardPoint(board, (file - 3.5) * board.s, (4.5 - rank) * board.s);
}

/** ROS position of board coordinates: `u` along the files a to h, `v` from rank 8 to rank 1. */
export function boardPoint(
  board: Board,
  u: number,
  v: number,
): [number, number] {
  const c = Math.cos(board.yaw);
  const s = Math.sin(board.yaw);
  // Files run towards the robot's left (+Y), ranks away from it (+X).
  return [board.x + v * c - u * s, board.y + u * c + v * s];
}

/** Board coordinates of a ROS position; the inverse of `boardPoint`. */
export function boardCoordinates(
  board: Board,
  x: number,
  y: number,
): [number, number] {
  const c = Math.cos(board.yaw);
  const s = Math.sin(board.yaw);
  const dx = x - board.x;
  const dy = y - board.y;
  return [-dx * s + dy * c, dx * c + dy * s];
}

/** The square a ROS position lies well inside of, or undefined near a line or off the board. */
export function squareAt(
  board: Board,
  x: number,
  y: number,
  margin: number,
): string | undefined {
  const [u, v] = boardCoordinates(board, x, y);
  const file = Math.floor(u / board.s + 4);
  const rank = 8 - Math.floor(v / board.s + 4);
  if (file < 0 || file > 7 || rank < 1 || rank > 8) return undefined;
  const du = u - (file - 3.5) * board.s;
  const dv = v - (4.5 - rank) * board.s;
  if (Math.max(Math.abs(du), Math.abs(dv)) > board.s / 2 - margin)
    return undefined;
  return `${files[file]}${rank}`;
}

export const squares = [...files].flatMap((file) =>
  [1, 2, 3, 4, 5, 6, 7, 8].map((rank) => `${file}${rank}`),
);

/**
 * Where the SO101 reaches every square centre with its fingers straight down: out to 30 cm from
 * the base and no closer than 11 cm in front of it, where the arm would fold into itself.
 */
export function reachable(board: Board): boolean {
  return squares.every((name) => {
    const [x, y] = squareCenter(board, name);
    return x >= 0.11 && Math.hypot(x, y) <= 0.3;
  });
}

/** Three.js world position of a ROS position on the desk, and back. */
export function world(x: number, y: number, height: number): Vector3 {
  return new Vector3(-y, height, -x);
}
export function ros(position: Vector3): [number, number] {
  return [-position.z, -position.x];
}

/** How far a body's up axis leans, radians. */
export function tilt(rotation: Quaternion): number {
  return new Vector3(0, 1, 0)
    .applyQuaternion(rotation)
    .angleTo(new Vector3(0, 1, 0));
}

/** A deterministic random source for a printed seed, so any episode can be replayed. */
export function seeded(seed: number): () => number {
  let state = seed >>> 0;
  return () => {
    state = (state + 0x6d2b79f5) >>> 0;
    let t = state;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
