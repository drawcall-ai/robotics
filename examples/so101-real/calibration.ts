// What the real arm's readings hide: how far each joint really is from what its servo reports,
// where the camera really sits, and how the gripper reading maps to the jaw.
//
// Ground truth: a policy must not read this file.
import type { Calibration } from "./servo.ts";

const deg = Math.PI / 180;

/**
 * wrist_roll's zero error, degrees. LeRobot sets the roll's zero at the middle of a full-turn
 * range, so it is arbitrary; with every calibration image near roll 0 it trades off exactly
 * against the camera's turn about the roll axis. 65° puts the jaws where the real photos show
 * them: the white fixed jaw's tip near pixel (735, 670), the black moving jaw to its left.
 */
export const ROLL_ERROR = 65;
/**
 * Wrist camera, OpenCV frame (x right, y down, z forward) in gripper_link, metres, row-major, as
 * the hand-eye calibration of REAL_OBSERVATIONS §3 found it for a roll zero error of 0. The
 * camera is mounted at this turned back by ROLL_ERROR about the roll axis, so views at any
 * reading are the calibrated ones.
 */
export const CAMERA = [
  0.1523, -0.7021, -0.6956, 0.1006,
  -0.9879, -0.0883, -0.1273, 0.0191,
  0.028, 0.7065, -0.7071, -0.0294,
  0, 0, 0, 1,
];

/** Jaw angle, radians, where the jaws touch: gripper reading 0. */
export const GRIPPER_CLOSED = -0.19;
/** Jaw travel from reading 0 to 100: 130° of motor rotation. */
export const GRIPPER_TRAVEL = 130 * deg;

/**
 * Zero errors, degrees, in the order pan, lift, elbow, wrist_flex, wrist_roll: the offsets the
 * camera calibration fitted where it was valid (upper arm near vertical), true = reading + offset.
 */
const OFFSET = [0, 4.4, -10.8, 5.2, ROLL_ERROR];
/**
 * The part the calibration could not see: grows with the shoulder's reading from 5° to 35° and
 * stays there, lowering the extended arm by about 4 cm near the table (§5).
 */
const EXTENDED = [0, 10, 0, 0, 0];
const share = (lift: number) => Math.min(1, Math.max(0, (lift - 5) / 30));

export const calibration: Calibration = {
  angles(readings) {
    const g = share(readings[1]!);
    return readings.map((r, i) =>
      i === 5
        ? GRIPPER_CLOSED + (r / 100) * GRIPPER_TRAVEL
        : (r + OFFSET[i]! + g * EXTENDED[i]!) * deg,
    );
  },
  readings(angles) {
    // The shoulder's reading decides the error: solve for it first, by fixed-point iteration.
    let lift = angles[1]! / deg - OFFSET[1]!;
    for (let k = 0; k < 8; k++)
      lift = angles[1]! / deg - OFFSET[1]! - share(lift) * EXTENDED[1]!;
    const g = share(lift);
    return angles.map((a, i) =>
      i === 5
        ? ((a - GRIPPER_CLOSED) / GRIPPER_TRAVEL) * 100
        : a / deg - OFFSET[i]! - g * EXTENDED[i]!,
    );
  },
};
