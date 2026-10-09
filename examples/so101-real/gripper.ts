import type { JointActuatorOptions } from "@drawcall/robotics";

// Simplified CAD pads. The gripping faces and distal ends retain the mesh dimensions: the real
// printed gripper (v47, 2026-10-04) keeps the stock SO-101 fixed jaw (tip at z −104.4 mm, gripping
// face at x −7.9 mm), and the moving jaw is the stock one.
/**
 * How far the real jaws sit further up gripper_link's z than the CAD ones, relative to the
 * calibrated camera. The solver's ten real board touches (2026-10-09) met the board 4.6 mm higher
 * than CAD jaws would; in the touch photos the closed tips appear about 3 mm short of the CAD
 * jaws'. 4 mm splits the two, each within its ±1 mm. The whole gripper body and the moving jaw are
 * shifted; the roll axis and the camera stay.
 */
export const JAW_SHIFT = 0.004;
export const fixedPads = [
  { size: [0.02, 0.02, 0.022], position: [-0.018, 0, -0.05] },
  { size: [0.008, 0.012, 0.0488], position: [-0.0119, 0, -0.08] },
] as const;
export const movingPads = [
  { size: [0.008, 0.045, 0.012], position: [-0.0083, -0.0595, 0.0189] },
] as const;

/**
 * A current-limited grip: a fifteenth of the servo's 3Nm stall torque, enough that it will not
 * crush what it picks up. At 0.05Nm the jaws gave way at 2.6 times the cube's weight and let it
 * slide out of a carry; this holds it at about nine times.
 */
export const gripperDrive: JointActuatorOptions = {
  interface: "position",
  stiffness: 20,
  damping: 0.2,
  maxForce: 0.2,
  maxVelocity: 1.5,
};
