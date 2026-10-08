// The real queen task: the queen lies on the table beside the board; the robot must stand it
// upright on d8. The evaluator also records what an auditor needs afterwards, on /sim/audit and
// in its metrics: every contact of the robot with the table, the board and the queen with its
// peak contact force and the servo loads during it, peak loads and temperatures, and overheating
// trips. Both are private: recorded in the run's bag, never seen by the policy. A policy must not
// read this file.
import type { RigidBody } from "@drawcall/physics";
import { Evaluator, getROS2 } from "@drawcall/robotics";
import { Quaternion, Vector3 } from "three";
import { board, boardBody, bus, queenBody, robot, table } from "./environment.ts";
import { ORDER } from "./robot.ts";
import { deskHeight, ros, squareAt, tilt, type Board } from "./task.ts";

export { show } from "./environment.ts";

const ros2 = getROS2();
const audit = (event: Record<string, unknown>) =>
  ros2.publish(
    "/sim/audit",
    "std_msgs/msg/String",
    { data: JSON.stringify(event) },
    { private: true },
  );
const now = () => ros2.world.time;

/** Robot bodies touching the queen now. */
const holding = new Set<RigidBody>();
const contacts = { table: 0, chessboard: 0, queen: 0 };
const peak = Object.fromEntries(ORDER.map((name) => [name, 0]));
let tableLoad = 0;
let trips = 0;
bus.events.push((message) => {
  if (message.includes("overheated")) trips++;
  audit({ time: now(), event: message });
});
// A touch lasts from the first robot link touching a thing until none has for 0.1 s (sliding
// contacts flicker); it reports the links involved, the largest total contact force between
// robot and thing (N) and the largest servo loads (‰ of stall torque) while it lasted.
interface Touch {
  start: number;
  end?: number;
  links: Set<string>;
  force: number;
  loads: Record<string, number>;
}
const maxForce = { table: 0, chessboard: 0, queen: 0 };
/** The net normal force the robot exerts on `body` now, N. */
function force(body: RigidBody): number {
  const total = new Vector3();
  for (const contact of ros2.world.contacts(body))
    if (contact.otherBody.parent === robot)
      total.addScaledVector(contact.normal, contact.force);
  return total.length();
}
const things = [
  { body: table, key: "table" as const, touching: new Set<RigidBody>(), touch: undefined as Touch | undefined },
  { body: boardBody, key: "chessboard" as const, touching: new Set<RigidBody>(), touch: undefined as Touch | undefined },
  { body: queenBody, key: "queen" as const, touching: new Set<RigidBody>(), touch: undefined as Touch | undefined },
];
for (const thing of things) {
  thing.body.addEventListener("contactbegin", ({ otherBody }) => {
    if (otherBody.parent !== robot) return;
    if (thing.key === "queen") holding.add(otherBody);
    thing.touching.add(otherBody);
    if (!thing.touch) {
      contacts[thing.key]++;
      thing.touch = { start: now(), links: new Set(), force: 0, loads: {} };
      audit({ time: now(), event: "contact", with: thing.key, link: otherBody.name });
    }
    thing.touch.end = undefined;
    thing.touch.links.add(otherBody.name);
  });
  thing.body.addEventListener("contactend", ({ otherBody }) => {
    if (thing.key === "queen") holding.delete(otherBody);
    if (thing.touching.delete(otherBody) && !thing.touching.size && thing.touch)
      thing.touch.end = now();
  });
}
let beat = -Infinity;
ros2.register({
  afterStep(time: number) {
    // A heartbeat with the queen's pose and the temperatures.
    if (time - beat >= 1) {
      beat = time;
      const [x, y] = ros(queenBody.position);
      const axis = new Vector3(0, 1, 0).applyQuaternion(queenBody.quaternion);
      audit({
        time,
        event: "heartbeat",
        queen: {
          x,
          y,
          z: queenBody.position.y - deskHeight,
          tiltDeg: (tilt(queenBody.quaternion) * 180) / Math.PI,
          axis: [-axis.z, -axis.x, axis.y],
        },
        temperature: Object.fromEntries(
          bus.states.map((s, i) => [ORDER[i]!, Math.round(s.temperature * 10) / 10]),
        ),
      });
    }
    for (const { body, key, touch } of things) {
      if (!touch) continue;
      touch.force = Math.max(touch.force, force(body));
      maxForce[key] = Math.max(maxForce[key], touch.force);
    }
    bus.states.forEach((state, i) => {
      const name = ORDER[i]!;
      peak[name] = Math.max(peak[name]!, Math.abs(state.load));
      for (const { key, touch } of things) {
        if (!touch || touch.end !== undefined) continue;
        touch.loads[name] = Math.max(touch.loads[name] ?? 0, Math.abs(state.load));
        if (key === "table" && name !== "gripper")
          tableLoad = Math.max(tableLoad, Math.abs(state.load));
      }
    });
    for (const thing of things) {
      const touch = thing.touch;
      if (touch?.end === undefined || time - touch.end < 0.1) continue;
      thing.touch = undefined;
      audit({
        time: touch.end,
        event: "release",
        with: thing.key,
        duration: touch.end - touch.start,
        links: [...touch.links],
        peakForce: Math.round(touch.force * 100) / 100,
        maxLoads: touch.loads,
      });
    }
  },
  dispose() {},
});

/** The board where it is now: squares are judged on the board, not the table. */
function current(): Board {
  const [x, y] = ros(boardBody.position);
  const forward = new Vector3(0, 0, -1).applyQuaternion(boardBody.quaternion);
  return { ...board, x, y, yaw: Math.atan2(-forward.x, -forward.z) };
}
const initial = boardBody.position.clone();
let boardShift = 0;
let rest: { since: number; position: Vector3; rotation: Quaternion } | undefined;
new Evaluator({
  private: true,
  timeout: 1800,
  metrics: {
    ...Object.fromEntries(
      Object.keys(maxForce).map((key) => [
        `maxForce_${key}`,
        () => maxForce[key as keyof typeof maxForce],
      ]),
    ),
    tableContacts: () => contacts.table,
    boardContacts: () => contacts.chessboard,
    queenContacts: () => contacts.queen,
    maxArmLoadOnTable: () => tableLoad,
    ...Object.fromEntries(ORDER.map((name) => [`maxLoad_${name}`, () => peak[name]!])),
    maxTemperature: () => Math.max(...bus.states.map((s) => s.temperature)),
    thermalTrips: () => trips,
    boardShift: () => (boardShift = Math.max(boardShift, boardBody.position.distanceTo(initial))),
  },
  failure: {
    queenFell: () => queenBody.position.y < deskHeight - 0.04,
    boardMoved: () => boardShift > 0.015,
  },
  success: {
    // Upright on d8, centred within 9 mm, standing on the board top, let go of and still a
    // second later. Rest is judged by pose: contact solvers keep bodies trembling slightly.
    queenOnD8: ({ elapsed }) => {
      const [x, y] = ros(queenBody.position);
      const placed =
        holding.size === 0 &&
        squareAt(current(), x, y, 0.2 * board.s) === "d8" &&
        tilt(queenBody.quaternion) < Math.PI / 18 &&
        Math.abs(queenBody.position.y - boardBody.position.y) < 0.004;
      if (!placed) {
        rest = undefined;
        return false;
      }
      if (
        !rest ||
        rest.position.distanceTo(queenBody.position) > 0.001 ||
        rest.rotation.angleTo(queenBody.quaternion) > Math.PI / 90
      ) {
        rest = {
          since: elapsed,
          position: queenBody.position.clone(),
          rotation: queenBody.quaternion.clone(),
        };
        return false;
      }
      return elapsed - rest.since >= 1;
    },
  },
  score: ({ status }) => (status === "success" ? 100 : 0),
});
