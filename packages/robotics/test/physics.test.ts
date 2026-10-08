// What the patched physics packages add (patches/@drawcall__physics*): contact forces and joint
// friction, as the simulator relies on them.
import { expect, it } from "vitest";
import {
  BoxCollider,
  JointDrive,
  RevoluteJoint,
  RigidBody,
} from "@drawcall/physics";
import { buildWorld } from "@drawcall/physics-mujoco";
import { defer } from "./fixture.js";

async function world(gravity: [number, number, number]) {
  const built = await buildWorld({ gravity, fixedDelta: 0.002 });
  defer(() => built.dispose());
  return built;
}

it("reports the normal force of a body resting on another as its weight", async () => {
  const physics = await world([0, -9.81, 0]);
  const floor = new RigidBody({ type: "static" });
  floor.add(new BoxCollider({ size: [1, 0.1, 1] }));
  floor.position.y = -0.05;
  const box = new RigidBody({ mass: 0.02 });
  box.add(new BoxCollider({ size: [0.03, 0.03, 0.03] }));
  box.position.y = 0.015;
  for (let step = 0; step < 500; step++) physics.update(0.002);
  const contacts = physics.contacts(box);
  expect(contacts.length).toBeGreaterThan(0);
  expect(contacts.every((contact) => contact.otherBody === floor)).toBe(true);
  // The floor pushes the box up.
  const lift = contacts.reduce((sum, c) => sum + c.force * c.normal.y, 0);
  expect(lift).toBeCloseTo(0.02 * 9.81, 3);
  // And the box pushes the floor down by as much.
  const press = physics
    .contacts(floor)
    .reduce((sum, c) => sum + c.force * c.normal.y, 0);
  expect(press).toBeCloseTo(-lift, 6);
});

it("holds a joint still against torques below its friction loss", async () => {
  const physics = await world([0, 0, 0]);
  const base = new RigidBody({ type: "static" });
  const wheel = new RigidBody({
    mass: 1,
    centerOfMass: [0, 0, 0],
    diagonalInertia: [0.01, 0.01, 0.01],
  });
  const joint = new RevoluteJoint({ body0: base, body1: wheel, axis: "Z" });
  const drive = new JointDrive({ frictionLoss: 0.4, armature: 0.01 });
  joint.setDrive(drive);
  drive.setTarget({ effort: 0.3 });
  for (let step = 0; step < 500; step++) physics.update(0.002);
  expect(Math.abs(joint.getState().position)).toBeLessThan(1e-3);
  drive.setTarget({ effort: 0.5 });
  for (let step = 0; step < 500; step++) physics.update(0.002);
  // Net 0.1 N·m on 0.02 kg·m² (inertia plus armature) for a second: about 2.5 rad.
  expect(joint.getState().position).toBeGreaterThan(2);
  expect(joint.getState().position).toBeLessThan(3);
  expect(() => new JointDrive({ frictionLoss: -1 })).toThrow();
});
