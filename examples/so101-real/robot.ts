import { readFileSync } from "node:fs";
import {
  BoxCollider,
  MeshCollider,
  RevoluteJoint,
  RigidBody,
} from "@drawcall/physics";
import { CameraSensor, Robot } from "@drawcall/robotics";
import {
  BufferGeometry,
  Float32BufferAttribute,
  Euler,
  Matrix4,
  Mesh,
  MeshStandardMaterial,
  Quaternion,
  Vector3,
} from "three";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { fixedPads, JAW_SHIFT, movingPads } from "./gripper.ts";
import { hulls } from "./hulls.ts";
import {
  calibration,
  CAMERA,
  GRIPPER_CLOSED,
  GRIPPER_TRAVEL,
  restAngles,
  ROLL_ERROR,
} from "./calibration.ts";
import { joints as authored, links as allLinks } from "./model.ts";
import { ServoBus, type ServoJoint } from "./servo.ts";

// The CAD camera mount is not the real one: the real camera sits where its calibration says.
const links = allLinks.filter((link) => !link.name.startsWith("wrist_camera"));
const joints = authored.filter((joint) => !joint.child.startsWith("wrist_camera"));
/** Calibrated ranges of the readings, LeRobot degrees (gripper 0..100). */
const RANGES: Record<string, readonly [number, number]> = {
  shoulder_pan: [-101.6, 101.6],
  shoulder_lift: [-95.36, 95.36],
  elbow_flex: [-90.35, 90.35],
  wrist_flex: [-85.9, 85.9],
  wrist_roll: [-180, 180],
  gripper: [0, 100],
};
export const ORDER = Object.keys(RANGES);

function transform(
  xyz: readonly [number, number, number],
  rpy: readonly [number, number, number],
) {
  return new Matrix4().compose(
    new Vector3(...xyz),
    new Quaternion().setFromEuler(new Euler(rpy[0], rpy[1], rpy[2], "ZYX")),
    new Vector3(1, 1, 1),
  );
}

export function arm(height: number) {
  // The true joint angles, TF and camera mount are recorded, never sent to the policy. Builder
  // checks that read them live (evidence/truth.py) run with SO101_PUBLIC_TRUTH=1.
  const robot = new Robot({ private: process.env.SO101_PUBLIC_TRUTH !== "1" });
  robot.name = "so101";
  // The real arm's printed parts are white, its moving jaw black.
  const yellow = new MeshStandardMaterial({ color: "#f2f0ec", roughness: 0.7 });
  const black = new MeshStandardMaterial({ color: "#25272b", roughness: 0.7 });
  const loader = new STLLoader();
  // Links on fixed joints, the camera mount and housing, are bolted into their parent's body
  // instead of simulated as tiny fixed-joint chains.
  const bolted = new Map<string, string>(
    joints.filter((joint) => !joint.limits).map((j) => [j.child, j.parent]),
  );
  const mass = (name: string): number =>
    (links.find((link) => link.name === name)?.mass ?? 0) +
    [...bolted]
      .filter(([, parent]) => parent === name)
      .reduce((sum, [child]) => sum + mass(child), 0);
  // Each link's body, and the link frame in that body's coordinates.
  const frames = new Map<string, { body: RigidBody; frame: Matrix4 }>();
  for (const link of links) {
    if (bolted.has(link.name)) continue;
    const body = new RigidBody({
      type: link.name === "base_link" ? "static" : "dynamic",
      mass: mass(link.name),
      canSleep: false,
    });
    body.name = link.name;
    body.setCollisionGroups({ membership: 1, filter: 3 });
    body.setMaterial({
      staticFriction: 0.8,
      dynamicFriction: 0.8,
      restitution: 0,
    });
    // The gripper's own geometry sits JAW_SHIFT up its z axis; the body, the roll axis and the
    // camera mount stay where the calibration put them.
    frames.set(link.name, {
      body,
      frame:
        link.name === "gripper_link"
          ? new Matrix4().makeTranslation(0, 0, JAW_SHIFT)
          : new Matrix4(),
    });
    robot.add(body);
  }
  function place(name: string) {
    const result = frames.get(name);
    if (!result) throw new Error(`Missing SO101 link ${name}`);
    return result;
  }
  // ROS +X forward, +Z up maps to Three -Z forward, +Y up.
  const base = place("base_link").body;
  base.setRotationFromMatrix(
    new Matrix4().set(0, -1, 0, 0, 0, 0, 1, 0, -1, 0, 0, 0, 0, 0, 0, 1),
  );
  base.position.y = height + 0.003;
  base.updateMatrix();
  const servoJoints = new Map<string, RevoluteJoint>();
  const rest = restAngles();
  const placed = new Set(["base_link"]);
  while (placed.size < links.length) {
    let progress = false;
    for (const spec of joints) {
      if (!placed.has(spec.parent) || placed.has(spec.child)) continue;
      const parent = place(spec.parent);
      const frame = parent.frame
        .clone()
        .multiply(transform(spec.xyz, spec.rpy));
      placed.add(spec.child);
      progress = true;
      if (!spec.limits) {
        frames.set(spec.child, { body: parent.body, frame });
        continue;
      }
      const child = place(spec.child).body;
      // Each link starts turned to the rest pose.
      const turn = new Matrix4().makeRotationZ(rest[ORDER.indexOf(spec.name)]!);
      child.applyMatrix4(parent.body.matrix.clone().multiply(frame).multiply(turn));
      child.updateMatrix();
      const joint = new RevoluteJoint({
        body0: parent.body,
        body1: child,
        frame0: frame,
        frame1: new Matrix4(),
        axis: "Z",
        // The real gripper opens 130° from closed; the CAD limit allowed 110°.
        // The real roll turns a full turn around its arbitrary zero.
        limits:
          spec.name === "gripper"
            ? [GRIPPER_CLOSED - 0.05, GRIPPER_CLOSED + GRIPPER_TRAVEL + 0.02]
            : spec.name === "wrist_roll"
              ? [((ROLL_ERROR - 180) * Math.PI) / 180, ((ROLL_ERROR + 180) * Math.PI) / 180]
              : spec.limits,
      });
      joint.name = spec.name;
      servoJoints.set(spec.name, joint);
      robot.add(joint);
    }
    if (!progress) throw new Error("Disconnected SO101 joint tree");
  }
  for (const link of links) {
    const { body, frame } = place(link.name);
    for (const visual of link.visuals) {
      const bytes = readFileSync(
        new URL(`./assets/${visual.file}`, import.meta.url),
      );
      const geometry = loader.parse(
        bytes.buffer.slice(
          bytes.byteOffset,
          bytes.byteOffset + bytes.byteLength,
        ),
      );
      geometry.scale(visual.scale[0], visual.scale[1], visual.scale[2]);
      const pose = frame.clone().multiply(transform(visual.xyz, visual.rpy));
      geometry.applyMatrix4(pose);
      const mesh = new Mesh(
        geometry,
        visual.material === "sts3215" || link.name === "moving_jaw_so101_v1_link"
          ? black
          : yellow,
      );
      mesh.castShadow = mesh.receiveShadow = true;
      body.add(mesh);
      // Separate jaw pads keep the grasp cavity open; a convex hull would fill it.
      if (
        link.name === "gripper_link" ||
        link.name === "moving_jaw_so101_v1_link"
      )
        continue;
      const points = hulls[visual.file];
      if (!points) throw new Error(`Missing collision hull ${visual.file}`);
      const hull = new BufferGeometry().setAttribute(
        "position",
        new Float32BufferAttribute(points, 3),
      );
      hull.scale(visual.scale[0], visual.scale[1], visual.scale[2]);
      hull.applyMatrix4(pose);
      const collider = new MeshCollider({
        approximation: "convexHull",
      }).setGeometry(hull);
      // The optional mount replaces overlapping printed wrist parts in the real assembly.
      // Its simplified collider contacts the environment, not the arm it is bolted to.
      if (bolted.has(link.name))
        collider.setCollisionGroups({ membership: 4, filter: 2 });
      body.add(collider);
    }
  }
  function box(
    name: string,
    size: readonly [number, number, number],
    position: readonly [number, number, number],
  ) {
    const collider = new BoxCollider({ size });
    collider.position.set(...position).applyMatrix4(place(name).frame);
    place(name).body.add(collider);
    return collider;
  }
  // The printed pads grip over an area, so they resist a pinched piece turning about the closing
  // axis: torsional friction, 2 cm (MuJoCo lets a held piece creep at a rate set by it, ≈1°/s
  // here). A collider's material replaces the body's, so the friction is repeated.
  const pad = { staticFriction: 0.8, dynamicFriction: 0.8, restitution: 0, torsionalFriction: 0.02 };
  box("gripper_link", [0.04, 0.035, 0.035], [0, 0, -0.022]);
  for (const { size, position } of fixedPads) box("gripper_link", size, position).setMaterial(pad);
  for (const { size, position } of movingPads)
    box("moving_jaw_so101_v1_link", size, position).setMaterial(pad);
  // Raspberry Pi Camera V2.1, 1280x960 from the binned 1640x1232 mode: fx = fy = 1061 px,
  // principal point centred, no distortion; mounted as calibrated (OpenCV frame in gripper_link).
  const camera = new CameraSensor({
    name: "wrist_camera",
    frameId: "wrist_camera_optical",
    width: 1280,
    height: 960,
    fov: (2 * Math.atan(480 / 1061) * 180) / Math.PI,
    rate: 5,
    quality: 90,
  });
  camera.near = 0.005;
  camera.updateProjectionMatrix();
  // Three.js cameras look along -Z with +Y up; OpenCV's along +Z with +Y down.
  camera.matrix
    .fromArray(CAMERA)
    .transpose()
    .premultiply(new Matrix4().makeRotationZ((-ROLL_ERROR * Math.PI) / 180))
    .multiply(new Matrix4().makeScale(1, -1, -1));
  // The calibration's rotation is rounded: take its nearest rotation, no shear.
  camera.position.setFromMatrixPosition(camera.matrix);
  camera.quaternion.setFromRotationMatrix(camera.matrix).normalize();
  const gripperBody = place("gripper_link").body;
  gripperBody.add(camera);
  // The camera board behind the lens, as an obstacle to the table and the pieces only.
  const board = new BoxCollider({ size: [0.025, 0.024, 0.012] });
  board.quaternion.copy(camera.quaternion);
  board.position
    .set(0, 0, 0.007)
    .applyQuaternion(camera.quaternion)
    .add(camera.position);
  board.setCollisionGroups({ membership: 4, filter: 2 });
  gripperBody.add(board);
  const servos: ServoJoint[] = ORDER.map((name) => ({
    name,
    joint: servoJoints.get(name)!,
    range: RANGES[name]!,
    // LeRobot limits the gripper to half the stall torque.
    torqueLimit: name === "gripper" ? 0.5 : 1,
    unitsPerDegree: name === "gripper" ? 100 / ((GRIPPER_TRAVEL * 180) / Math.PI) : 1,
    friction: name !== "gripper",
  }));
  const bus = new ServoBus(servos, calibration);
  return { robot, bus };
}
