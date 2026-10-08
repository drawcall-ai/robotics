import type { JointActuatorOptions } from "@drawcall/robotics";

// Simplified CAD pads with the gripping faces of the meshes. The real printed jaws end about 9 mm
// short of the CAD ones: their tips touched the table 3 mm above the URDF gripper_frame point
// (−0.0079, −0.0002, −0.0981), so both pads end at z ≈ −0.095.
export const fixedPads = [
  { size: [0.02, 0.02, 0.022], position: [-0.018, 0, -0.05] },
  { size: [0.008, 0.012, 0.0394], position: [-0.0119, 0, -0.0753] },
] as const;
export const movingPads = [
  { size: [0.008, 0.0356, 0.012], position: [-0.0083, -0.0548, 0.0189] },
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
