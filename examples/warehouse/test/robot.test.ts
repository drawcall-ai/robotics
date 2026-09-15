import { afterEach, expect, test } from "vitest";
import { BoxCollider, RigidBody } from "@drawcall/physics";
import { setupWorld } from "@drawcall/physics-rapier";
import { Scene, Vector3 } from "three";
import { createRobot } from "../src/robot";

const worlds: Awaited<ReturnType<typeof setupWorld>>[] = [];
afterEach(() => {
  for (const world of worlds.splice(0)) world.dispose();
});
async function fixture() {
  const world = await setupWorld({
    gravity: [0, -9.81, 0],
    fixedDelta: 1 / 120,
    maxSubsteps: 16,
    solverIterations: 12,
  });
  worlds.push(world);
  const scene = new Scene();
  const floor = new RigidBody({ world, type: "static" });
  floor.position.y = -0.1;
  floor.add(new BoxCollider({ size: [20, 0.2, 20] }));
  scene.add(floor);
  return { world, robot: createRobot(world, scene) };
}
test("equal positive native wheel commands propel the stable chassis forward along -Z", async () => {
  const { world, robot } = await fixture();
  robot.left.motor.setTarget({ velocity: 3 });
  robot.right.motor.setTarget({ velocity: 3 });
  for (let i = 0; i < 360; i++) world.update(world.fixedDelta);
  expect(robot.chassis.position.z).toBeLessThan(-0.6);
  expect(Math.abs(robot.chassis.position.x)).toBeLessThan(0.2);
  expect(
    new Vector3(0, 1, 0).applyQuaternion(robot.chassis.quaternion).y,
  ).toBeGreaterThan(0.95);
});
test("opposite native wheel commands turn the robot in place", async () => {
  const { world, robot } = await fixture();
  robot.left.motor.setTarget({ velocity: -2 });
  robot.right.motor.setTarget({ velocity: 2 });
  for (let i = 0; i < 300; i++) world.update(world.fixedDelta);
  const forward = new Vector3(0, 0, -1).applyQuaternion(
    robot.chassis.quaternion,
  );
  expect(Math.abs(Math.atan2(-forward.x, -forward.z))).toBeGreaterThan(0.4);
  expect(
    Math.hypot(robot.chassis.position.x, robot.chassis.position.z),
  ).toBeLessThan(0.4);
});
