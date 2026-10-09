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
export const ROLL_ERROR = 100;
/**
 * Wrist camera, OpenCV frame (x right, y down, z forward) in gripper_link, metres, row-major, as
 * the real arm's hand-eye calibration found it for a roll zero error of 0 (2026-10-08 evening,
 * 12 board views, 3–6 px; within 0.8° and 3 mm of REAL_OBSERVATIONS §3). The
 * camera is mounted at this turned back by ROLL_ERROR about the roll axis, so views at any
 * reading are the calibrated ones.
 */
export const CAMERA = [
  0.1476, -0.7005, -0.6983, 0.0999,
  -0.9889, -0.0942, -0.1144, 0.0165,
  0.0144, 0.7074, -0.7066, -0.03,
  0, 0, 0, 1,
];

/** Jaw angle, radians, where the jaws touch: gripper reading 0. */
export const GRIPPER_CLOSED = -0.19;
/** Jaw travel from reading 0 to 100: 130° of motor rotation. */
export const GRIPPER_TRAVEL = 130 * deg;

/**
 * Zero errors, degrees, in the order pan, lift, elbow, wrist_flex, wrist_roll: what the real arm's
 * careful calibration fitted (12 board views in free air, upper arm near vertical; readings in
 * today's calibration), model = reading + offset. REAL_OBSERVATIONS §1's earlier, rougher fit
 * (+4.4, −10.8, +5.2 before the rest-pose shift) was 9° off on the elbow.
 */
const FITTED = [0, -0.09, 1.55, -0.71, ROLL_ERROR];
/**
 * Folded near the base the real arm reaches less far down than the calibration says: board touches
 * with the elbow at 56–58° (the d8 row) met the board 5–7 mm higher than at elbow ≤ 46°, where the
 * calibration is right to about a millimetre (the solver's D1/D2 touches, 2026-10-09). The shoulder
 * sits 2.2° further back than its reading says, ramped in as the elbow reading goes from 47° to 56°.
 */
const FOLDED = 2.2;
const folded = (elbow: number) => FOLDED * Math.min(1, Math.max(0, (elbow - 47) / 9));

export const calibration: Calibration = {
  angles(readings) {
    return readings.map((r, i) =>
      i === 5
        ? GRIPPER_CLOSED + (r / 100) * GRIPPER_TRAVEL
        : (r + FITTED[i]! - (i === 1 ? folded(readings[2]!) : 0)) * deg,
    );
  },
  readings(angles) {
    // The elbow's reading decides the shoulder's term, and does not depend on it.
    const elbow = angles[2]! / deg - FITTED[2]!;
    return angles.map((a, i) =>
      i === 5
        ? ((a - GRIPPER_CLOSED) / GRIPPER_TRAVEL) * 100
        : a / deg - FITTED[i]! + (i === 1 ? folded(elbow) : 0),
    );
  },
};

/**
 * The arm's unpowered rest pose, folded so that it holds itself, as readings: where the real arm
 * sits when its torque is off and where the simulation starts.
 */
export const REST = [-5.7, -94.5, 89.5, 16.9, -5.6, 5.2];
/** The rest pose's true joint angles, radians. */
export const restAngles = () => calibration.angles(REST);
