import { Quaternion, Vector3 } from "three";

export const deskHeight = 0.74;
export const objectSize = 0.022;
export const bowlRadius = 0.068;
export const bowlHeight = 0.045;
export const bowlFloor = 0.008;
export const count = 3;
export interface Placement {
  x: number;
  z: number;
  radius: number;
}

export function layout(random: () => number = Math.random): Placement[] {
  const placed: Placement[] = [];
  for (const radius of [
    Math.hypot(
      bowlRadius + 0.008,
      (bowlRadius + 0.004) * Math.tan(Math.PI / 32) + 0.0005,
    ),
    ...Array<number>(count).fill(objectSize / Math.sqrt(2)),
  ]) {
    let accepted = false;
    for (let attempt = 0; attempt < 10000; attempt++) {
      const x = (random() - 0.5) * 0.34;
      const z = -0.15 - random() * 0.15;
      // A low, downward grasp must fit the joint limits, not just the arm's full extension.
      if (Math.hypot(x, z) > 0.3 || Math.hypot(x, z) + radius > 0.365) continue;
      if (
        placed.some(
          (p) => Math.hypot(p.x - x, p.z - z) < p.radius + radius + 0.018,
        )
      )
        continue;
      placed.push({ x, z, radius });
      accepted = true;
      break;
    }
    if (!accepted)
      throw new Error("Could not sample a collision-free desk layout");
  }
  return placed;
}

/** All eight corners must be below the rim and inside the polygonal cavity. */
export function contained(
  position: Vector3,
  rotation: Quaternion,
  bowlPosition: Vector3,
  bowlRotation: Quaternion,
): boolean {
  const inverse = bowlRotation.clone().invert();
  for (const x of [-1, 1])
    for (const y of [-1, 1])
      for (const z of [-1, 1]) {
        const p = new Vector3(x, y, z)
          .multiplyScalar(objectSize / 2)
          .applyQuaternion(rotation)
          .add(position)
          .sub(bowlPosition)
          .applyQuaternion(inverse);
        if (p.y < bowlFloor - 0.002 || p.y > bowlHeight - 0.002) return false;
        for (let side = 0; side < 32; side++) {
          const angle = (side * Math.PI) / 16;
          if (
            p.x * Math.cos(angle) + p.z * Math.sin(angle) >
            bowlRadius - 0.002
          )
            return false;
        }
      }
  return true;
}
