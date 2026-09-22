import { BoxCollider, RigidBody } from "@drawcall/physics";
import { Evaluator } from "@drawcall/robotics";
import {
  AmbientLight,
  BoxGeometry,
  Color,
  CylinderGeometry,
  DirectionalLight,
  HemisphereLight,
  Mesh,
  MeshStandardMaterial,
  Scene,
} from "three";
import { arm } from "./robot.ts";
import {
  bowlFloor,
  bowlHeight,
  bowlRadius,
  contained,
  count,
  deskHeight,
  layout,
  objectSize,
} from "./task.ts";

const scene = new Scene();
scene.background = new Color("#dce4e7");
scene.add(
  new HemisphereLight("#ffffff", "#687078", 2.3),
  new AmbientLight("#ffffff", 0.7),
);
const sun = new DirectionalLight("#fff5e8", 2.5);
sun.position.set(1, 3, 2);
scene.add(sun);
const wood = new MeshStandardMaterial({ color: "#b49b7c", roughness: 0.9 });
const blue = new MeshStandardMaterial({ color: "#206cdb", roughness: 0.6 });
function box(
  body: RigidBody,
  size: [number, number, number],
  material: MeshStandardMaterial,
  position: [number, number, number] = [0, 0, 0],
  yaw = 0,
) {
  const collider = new BoxCollider({ size });
  const mesh = new Mesh(new BoxGeometry(...size), material);
  for (const part of [collider, mesh]) {
    part.position.set(...position);
    part.rotation.y = yaw;
    body.add(part);
  }
}
const desk = new RigidBody({ type: "static" });
desk.name = "desk";
desk.position.set(0, deskHeight - 0.025, -0.18);
box(desk, [0.8, 0.05, 0.7], wood);
for (const x of [-0.34, 0.34])
  for (const z of [-0.29, 0.29])
    box(desk, [0.045, deskHeight - 0.05, 0.045], wood, [x, -deskHeight / 2, z]);
const robot = arm(deskHeight);
scene.add(desk, robot);
const [bowlPlacement, ...placements] = layout();
if (!bowlPlacement) throw new Error("Missing bowl placement");
const bowl = new RigidBody({ mass: 0.18 });
bowl.name = "bowl";
bowl.position.set(bowlPlacement.x, deskHeight, bowlPlacement.z);
bowl.rotation.y = Math.random() * Math.PI * 2;
bowl.setMaterial({ staticFriction: 0.5, dynamicFriction: 0.5, restitution: 0 });
const floorMesh = new Mesh(
  new CylinderGeometry(bowlRadius + 0.004, bowlRadius + 0.004, bowlFloor, 32),
  blue,
);
floorMesh.position.y = bowlFloor / 2;
bowl.add(floorMesh);
// The floor collides as an octagon of boxes. MuJoCo resolves a box against a
// cylinder through a single contact point, so a cube landing on an edge would
// kick the bowl centimetres; box pairs get a full contact patch.
for (let strip = 0; strip < 4; strip++) {
  const collider = new BoxCollider({
    size: [0.14, bowlFloor, 0.14 * Math.tan(Math.PI / 8)],
  });
  collider.position.y = bowlFloor / 2;
  collider.rotation.y = (strip * Math.PI) / 4;
  bowl.add(collider);
}
for (let side = 0; side < 32; side++) {
  const angle = (side * Math.PI) / 16;
  const radius = bowlRadius + 0.004;
  box(
    bowl,
    [0.008, bowlHeight, 2 * radius * Math.tan(Math.PI / 32) + 0.001],
    blue,
    [radius * Math.cos(angle), bowlHeight / 2, radius * Math.sin(angle)],
    -angle,
  );
}
scene.add(bowl);
const objects = placements.map((p, index) => {
  const body = new RigidBody({ mass: 0.012 });
  body.name = `object_${index + 1}`;
  body.position.set(p.x, deskHeight + objectSize / 2 + 0.0005, p.z);
  body.rotation.y = Math.random() * Math.PI * 2;
  body.setMaterial({
    staticFriction: 0.7,
    dynamicFriction: 0.7,
    restitution: 0,
  });
  box(
    body,
    [objectSize, objectSize, objectSize],
    new MeshStandardMaterial({
      color: ["#ed3434", "#37b957", "#e435b3"][index],
      roughness: 0.8,
    }),
  );
  scene.add(body);
  return body;
});
const touchingRobot = new Map<RigidBody, Set<RigidBody>>();
for (const object of objects) {
  const contacts = new Set<RigidBody>();
  touchingRobot.set(object, contacts);
  object.addEventListener("contactbegin", ({ otherBody }) => {
    if (otherBody.parent === robot) contacts.add(otherBody);
  });
  object.addEventListener("contactend", ({ otherBody }) =>
    contacts.delete(otherBody),
  );
}
const initialPosition = bowl.position.clone();
const initialRotation = bowl.quaternion.clone();
let displacement = 0,
  rotation = 0,
  settledSince: number | undefined;
new Evaluator({
  timeout: 240,
  metrics: {
    objectsInBowl: () =>
      objects.filter((o) =>
        contained(o.position, o.quaternion, bowl.position, bowl.quaternion),
      ).length,
    peakBowlDisplacement: () =>
      (displacement = Math.max(
        displacement,
        bowl.position.distanceTo(initialPosition),
      )),
    peakBowlRotation: () =>
      (rotation = Math.max(rotation, bowl.quaternion.angleTo(initialRotation))),
  },
  failure: {
    bowlMoved: () => displacement > 0.02,
    bowlRotated: () => rotation > Math.PI / 18,
    objectDropped: () => objects.some((o) => o.position.y < deskHeight - 0.04),
  },
  success: {
    allObjectsSettled: ({ elapsed, metrics }) => {
      const still =
        metrics.objectsInBowl === count &&
        bowl.getVelocity().linear.length() < 0.015 &&
        bowl.getVelocity().angular.length() < 0.15 &&
        objects.every(
          (o) =>
            o.getVelocity().linear.length() < 0.015 &&
            o.getVelocity().angular.length() < 0.15 &&
            touchingRobot.get(o)?.size === 0,
        );
      if (!still) {
        settledSince = undefined;
        return false;
      }
      settledSince ??= elapsed;
      return elapsed - settledSince >= 1.5;
    },
  },
  score: ({ metrics, status }) =>
    status === "success"
      ? 100
      : ((90 * (metrics.objectsInBowl ?? 0)) / count) *
        Math.max(0, 1 - displacement / 0.02),
});
