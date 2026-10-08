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
 * camera calibration fitted where it was valid (upper arm near vertical, wrist 40–75°),
 * model = reading + offset.
 */
const FITTED = [0, 4.4, -10.8, 5.2, ROLL_ERROR];
/**
 * How far each joint's link bends away from its encoder's reading, degrees per N·m of the torque
 * it passes on to the link (`load.ts`): gear backlash and printed-plastic flex, which the encoder
 * on the servo's output cannot see. Holding the extended arm in the air sags it by a few more
 * degrees than in the calibration poses; resting on the table, the link sits higher than the
 * readings suggest. Fitted to the contacts of REAL_OBSERVATIONS §5.
 */
const BENDING = [0, 8, 8, 0, 0];
/** The link torques, N·m, in the calibration poses: there the fitted offsets hold. */
const REFERENCE = [0, -0.54, -0.42, 0, 0];
/** Zero errors with the links unloaded: true = reading + offset − bending·torque. */
const OFFSET = FITTED.map((f, i) => f + BENDING[i]! * REFERENCE[i]!);

/**
 * What the bending does not explain: hanging free far out, the real shoulder sits about 4° lower
 * than the calibrated model says (the near-table photos of §5; D). It ramps in with the lift
 * reading from 5° to 35° and only while the arm hangs: it fades as the gripper comes to rest on
 * the table, board or queen.
 */
const HANGING = [0, 4, 0, 0, 0];
const share = (lift: number) => Math.min(1, Math.max(0, (lift - 5) / 30));

export const calibration: Calibration = {
  angles(readings, { torques, hanging }) {
    const g = share(readings[1]!) * hanging[1]!;
    return readings.map((r, i) =>
      i === 5
        ? GRIPPER_CLOSED + (r / 100) * GRIPPER_TRAVEL
        : (r + OFFSET[i]! - BENDING[i]! * torques[i]! + g * HANGING[i]!) * deg,
    );
  },
  readings(angles, { torques, hanging }) {
    const plain = angles.map((a, i) => a / deg - OFFSET[i]! + BENDING[i]! * torques[i]!);
    // The shoulder's reading decides the hanging term: solve for it by fixed-point iteration.
    let lift = plain[1]!;
    for (let k = 0; k < 8; k++) lift = plain[1]! - share(lift) * hanging[1]! * HANGING[1]!;
    const g = share(lift) * hanging[1]!;
    return angles.map((a, i) =>
      i === 5 ? ((a - GRIPPER_CLOSED) / GRIPPER_TRAVEL) * 100 : plain[i]! - g * HANGING[i]!,
    );
  },
};
