import type { JointActuatorOptions } from "@drawcall/robotics";

// Simplified CAD pads. The gripping faces and distal ends retain the mesh dimensions: the real
// printed gripper (v47, 2026-10-04) keeps the stock SO-101 fixed jaw (tip at z −104.4 mm, gripping
// face at x −7.9 mm), and the moving jaw is the stock one.
/**
 * Where the real jaws sit relative to the CAD ones, in gripper_link, metres, relative to the
 * calibrated camera. In the solver's 13 closed-jaw touch photos (2026-10-09) the closed tips meet
 * at pixel (655, 645) and the fixed jaw's apex is at (718, 660), at any opening. This puts the
 * meeting point on that pixel's ray, at the depth (119.5 mm from the camera; the photos say
 * 122 ± 5) where the ten real board touches land within −1.7…+0.2 mm. The whole gripper body and
 * the moving jaw move; the roll axis and the camera stay.
 */
export const JAW_OFFSET = [0.0067, -0.0035, 0.0031] as const;
export const fixedPads = [
  { size: [0.02, 0.02, 0.022], position: [-0.018, 0, -0.05] },
  // The distal finger, gripping face at x −7.9 mm; 14.5 mm wide, so its apex shows where the real
  // white jaw's does, 7.2 mm outside the meeting point.
  { size: [0.0145, 0.012, 0.0488], position: [-0.01515, 0, -0.08] },
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
