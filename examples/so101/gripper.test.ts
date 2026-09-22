import { expect, it } from "vitest";
import { buildWorld } from "@drawcall/physics-mujoco";
import {
  registry,
  BoxCollider,
  JointDrive,
  RevoluteJoint,
  RigidBody,
} from "@drawcall/physics";
import { Group, Matrix4 } from "three";
import { fixedPads, movingPads, gripperDrive } from "./gripper.ts";

it.each([0, Math.PI / 4])(
  "physically retains a cube at yaw %s after its support is removed",
  async (yaw) => {
    // Isolated gripper in its local CAD coordinates (+Z up), with no robot/controller assistance.
    const world = await buildWorld({
      gravity: [0, 0, -9.81],
      fixedDelta: 1 / 1000,
      solverIterations: 50,
      frictionImpedanceRatio: 10,
      frictionCone: "elliptic",
    });
    try {
      const scene = new Group();
      function box(
        body: RigidBody,
        size: readonly [number, number, number],
        position: readonly [number, number, number],
      ) {
        const collider = new BoxCollider({ size });
        collider.position.set(...position);
        body.add(collider);
      }
      const fixed = new RigidBody({ type: "static" });
      const jaw = new RigidBody({ mass: 0.012, canSleep: false });
      for (const pad of fixedPads) box(fixed, pad.size, pad.position);
      for (const pad of movingPads) box(jaw, pad.size, pad.position);
      for (const body of [fixed, jaw])
        body.setMaterial({
          staticFriction: 0.8,
          dynamicFriction: 0.8,
          restitution: 0,
        });
      const frame = new Matrix4()
        .makeRotationX(Math.PI / 2)
        .setPosition(0.0202, 0.0188, -0.0234);
      jaw.applyMatrix4(
        frame.clone().multiply(new Matrix4().makeRotationZ(0.4)),
      );
      const joint = new RevoluteJoint({
        body0: fixed,
        body1: jaw,
        frame0: frame,
        frame1: new Matrix4(),
        axis: "Z",
        limits: [-0.174533, 1.74533],
      });
      const drive = new JointDrive(gripperDrive);
      joint.setDrive(drive);
      const cube = new RigidBody({ mass: 0.012 });
      cube.position.set(
        -0.0079 + 0.011 * (Math.cos(yaw) + Math.sin(yaw)) + 0.001,
        0,
        -0.097,
      );
      cube.rotation.z = yaw;
      box(cube, [0.022, 0.022, 0.022], [0, 0, 0]);
      cube.setMaterial({
        staticFriction: 0.7,
        dynamicFriction: 0.7,
        restitution: 0,
      });
      const floor = new RigidBody({ type: "static" });
      floor.position.z = -0.113;
      box(floor, [0.2, 0.2, 0.01], [0, 0, 0]);
      scene.add(fixed, jaw, joint, cube, floor);
      for (let step = 0; step < 2000; step++) {
        const fraction = Math.min(1, step / 1000);
        const blend = fraction * fraction * (3 - 2 * fraction);
        drive.setTarget({ position: 0.4 - 0.574 * blend });
        world.update(1 / 1000);
      }
      floor.dispose();
      scene.remove(floor);
      const held = cube.position.clone();
      for (let step = 0; step < 10000; step++) world.update(1 / 1000);
      expect(cube.position.distanceTo(held)).toBeLessThan(0.005);
      expect(cube.position.z).toBeGreaterThan(-0.102);
      expect(joint.getState().position).toBeGreaterThan(-0.1);
    } finally {
      world.dispose();
      registry.clear();
    }
  },
);
