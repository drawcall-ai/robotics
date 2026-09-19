import type { JointActuatorOptions } from "@drawcall/robotics";

// Simplified CAD pads. The gripping faces and distal ends retain the mesh dimensions.
export const fixedPads = [
  { size: [0.02, 0.02, 0.022], position: [-0.018, 0, -0.05] },
  { size: [0.008, 0.012, 0.0488], position: [-0.0119, 0, -0.08] },
] as const;
export const movingPads = [
  { size: [0.008, 0.045, 0.012], position: [-0.0083, -0.0595, 0.0189] },
] as const;

// A gentle current-limited grip avoids ejecting a 12g cube with an impulsive closure.
export const gripperDrive: JointActuatorOptions = {
  interface: "position",
  stiffness: 20,
  damping: 0.2,
  maxForce: 0.05,
  maxVelocity: 1.5,
};
