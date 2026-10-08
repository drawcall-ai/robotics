// Builder check of world.contacts: the queen standing at rest on the table reports its weight.
// Run: node evidence/weight.ts
import { BoxCollider, RigidBody } from "@drawcall/physics";
// The simulator's backend, which the example itself does not depend on.
import { buildWorld } from "../node_modules/@drawcall/robotics/node_modules/@drawcall/physics-mujoco/dist/index.js";
import { Vector3 } from "three";
import { queen } from "../pieces.ts";

const world = await buildWorld({
  gravity: [0, -9.81, 0],
  fixedDelta: 1 / 500,
  frictionImpedanceRatio: 10,
  frictionCone: "elliptic",
});
const table = new RigidBody({ type: "static" });
table.add(new BoxCollider({ size: [0.5, 0.05, 0.5] }));
table.position.y = -0.025;
const piece = queen();
piece.position.y = 0.0005;
for (let i = 0; i < 1000; i++) world.update(world.fixedDelta);
const total = new Vector3();
for (const contact of world.contacts(piece)) total.addScaledVector(contact.normal, contact.force);
// The mass from an upward impulse's change of velocity.
const before = world.getVelocity(piece).linear.y;
world.applyImpulse(piece, new Vector3(0, 0.001, 0));
const mass = 0.001 / (world.getVelocity(piece).linear.y - before);
const weight = mass * 9.81;
console.log(
  JSON.stringify({
    contacts: world.contacts(piece).length,
    massG: +(mass * 1000).toFixed(2),
    weightN: +weight.toFixed(4),
    normalForceN: +total.y.toFixed(4),
    errorPercent: +((100 * (total.y - weight)) / weight).toFixed(2),
  }),
);
world.dispose();
