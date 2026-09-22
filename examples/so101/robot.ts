import { readFileSync } from "node:fs";
import {
  BoxCollider,
  FixedJoint,
  MeshCollider,
  RevoluteJoint,
  RigidBody,
} from "@drawcall/physics";
import { CameraSensor, JointActuator, Robot } from "@drawcall/robotics";
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
import { fixedPads, movingPads, gripperDrive } from "./gripper.ts";
import { hulls } from "./hulls.ts";
import { joints, links } from "./model.ts";

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
  const robot = new Robot();
  robot.name = "so101";
  const bodies = new Map<string, RigidBody>();
  const yellow = new MeshStandardMaterial({
    color: "#ffd12e",
    roughness: 0.65,
  });
  const black = new MeshStandardMaterial({ color: "#25272b", roughness: 0.7 });
  const loader = new STLLoader();
  for (const link of links) {
    const body = new RigidBody({
      type: link.name === "base_link" ? "static" : "dynamic",
      mass: link.mass + (link.name === "gripper_link" ? 0.035 : 0),
      canSleep: false,
    });
    body.name = link.name;
    // The optional mount replaces overlapping printed wrist parts in the real assembly.
    // Its simplified collider contacts the environment, not the arm it is bolted to.
    body.setCollisionGroups(
      link.name.startsWith("wrist_camera")
        ? { membership: 4, filter: 2 }
        : { membership: 1, filter: 3 },
    );
    body.setMaterial({
      staticFriction: 0.8,
      dynamicFriction: 0.8,
      restitution: 0,
    });
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
      const pose = transform(visual.xyz, visual.rpy);
      geometry.applyMatrix4(pose);
      const mesh = new Mesh(
        geometry,
        visual.material === "sts3215" ? black : yellow,
      );
      mesh.castShadow = mesh.receiveShadow = true;
      body.add(mesh);
      // Separate jaw pads keep the grasp cavity open; a convex hull would fill it.
      if (
        link.name !== "gripper_link" &&
        link.name !== "moving_jaw_so101_v1_link"
      ) {
        const points = hulls[visual.file];
        if (!points) throw new Error(`Missing collision hull ${visual.file}`);
        const hull = new BufferGeometry().setAttribute(
          "position",
          new Float32BufferAttribute(points, 3),
        );
        hull.scale(visual.scale[0], visual.scale[1], visual.scale[2]);
        hull.applyMatrix4(pose);
        body.add(
          new MeshCollider({ approximation: "convexHull" }).setGeometry(hull),
        );
      }
    }
    bodies.set(link.name, body);
    robot.add(body);
  }
  function body(name: string) {
    const result = bodies.get(name);
    if (!result) throw new Error(`Missing SO101 link ${name}`);
    return result;
  }
  function box(
    name: string,
    size: readonly [number, number, number],
    position: readonly [number, number, number],
  ) {
    const collider = new BoxCollider({ size });
    collider.position.set(...position);
    body(name).add(collider);
  }
  box("gripper_link", [0.04, 0.035, 0.035], [0, 0, -0.022]);
  for (const pad of fixedPads) box("gripper_link", pad.size, pad.position);
  for (const pad of movingPads)
    box("moving_jaw_so101_v1_link", pad.size, pad.position);
  // ROS +X forward, +Z up maps to Three -Z forward, +Y up.
  const base = body("base_link");
  base.setRotationFromMatrix(
    new Matrix4().set(0, -1, 0, 0, 0, 0, 1, 0, -1, 0, 0, 0, 0, 0, 0, 1),
  );
  base.position.y = height + 0.003;
  base.updateMatrix();
  const placed = new Set(["base_link"]);
  while (placed.size < bodies.size) {
    let progress = false;
    for (const spec of joints) {
      if (!placed.has(spec.parent) || placed.has(spec.child)) continue;
      const parent = body(spec.parent),
        child = body(spec.child);
      const frame = transform(spec.xyz, spec.rpy);
      child.applyMatrix4(parent.matrix.clone().multiply(frame));
      child.updateMatrix();
      const options = {
        body0: parent,
        body1: child,
        frame0: frame,
        frame1: new Matrix4(),
      };
      const joint = spec.limits
        ? new RevoluteJoint({ ...options, axis: "Z", limits: spec.limits })
        : new FixedJoint(options);
      joint.name = spec.name;
      if (joint instanceof RevoluteJoint)
        joint.setDrive(
          new JointActuator(
            spec.name === "gripper"
              ? gripperDrive
              : {
                  interface: "position",
                  // Damping is the servo's torque-speed slope, maxForce over
                  // maxVelocity, so the motor brakes itself towards its rated
                  // speed. The old value of 20 spent the whole 3Nm budget on
                  // damping past 0.15rad/s, leaving nothing to hold a pose, and
                  // only a relay-stiff gain of 8000 hid that.
                  stiffness: 500,
                  damping: 2,
                  maxForce: 3,
                  maxVelocity: 1.5,
                },
          ),
        );
      robot.add(joint);
      placed.add(spec.child);
      progress = true;
    }
    if (!progress) throw new Error("Disconnected SO101 joint tree");
  }
  const camera = new CameraSensor({
    name: "wrist_camera",
    frameId: "wrist_camera_optical",
    width: 640,
    height: 480,
    fov: 70,
    rate: 15,
  });
  // The CAD camera housing's lens points along its local +Z.
  camera.rotation.y = Math.PI;
  camera.position.z = 0.027;
  camera.near = 0.01;
  camera.updateProjectionMatrix();
  body("wrist_camera_link").add(camera);
  // Bolt fixed camera parts into the wrist body instead of simulating tiny fixed-joint chains.
  const gripper = body("gripper_link");
  for (const name of ["wrist_camera_mount_link", "wrist_camera_link"]) {
    const part = body(name);
    const relative = gripper.matrix.clone().invert().multiply(part.matrix);
    for (const child of [...part.children]) {
      child.applyMatrix4(relative);
      if (child instanceof MeshCollider)
        child.setCollisionGroups({ membership: 4, filter: 2 });
      gripper.add(child);
    }
    for (const joint of [...robot.children])
      if (
        joint instanceof FixedJoint &&
        (joint.options.body1 === part || joint.options.body0 === part)
      ) {
        joint.dispose();
        robot.remove(joint);
      }
    part.dispose();
    robot.remove(part);
  }
  return robot;
}
