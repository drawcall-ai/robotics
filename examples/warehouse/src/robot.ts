import {
  BoxCollider,
  CylinderCollider,
  JointMotor,
  RevoluteJoint,
  RigidBody,
  SphereCollider,
  SphericalJoint,
} from "@drawcall/physics";
import type { PhysicsWorld } from "@drawcall/physics";
import {
  BoxGeometry,
  CylinderGeometry,
  Matrix4,
  Mesh,
  MeshStandardMaterial,
  SphereGeometry,
} from "three";
import type { Scene } from "three";

export function createRobot(world: PhysicsWorld, scene: Scene) {
  const orange = new MeshStandardMaterial({
    color: "#f7ab39",
    roughness: 0.45,
    metalness: 0.25,
  });
  const dark = new MeshStandardMaterial({ color: "#26343b", roughness: 0.65 });
  const metal = new MeshStandardMaterial({
    color: "#b9c8ca",
    metalness: 0.65,
    roughness: 0.3,
  });
  const chassis = new RigidBody({ world, mass: 12, canSleep: false });
  chassis.position.set(0, 0.36, 0);
  chassis.name = "base_link";
  chassis.add(new BoxCollider({ size: [0.66, 0.22, 0.82] }));
  const shell = new Mesh(new BoxGeometry(0.66, 0.22, 0.82), orange);
  shell.castShadow = true;
  chassis.add(shell);
  const lid = new Mesh(new BoxGeometry(0.54, 0.07, 0.62), dark);
  lid.position.y = 0.145;
  chassis.add(lid);
  const lidar = new Mesh(new CylinderGeometry(0.09, 0.09, 0.06, 24), dark);
  lidar.position.set(0, 0.21, -0.32);
  chassis.add(lidar);
  for (const x of [-0.22, 0.22]) {
    const light = new Mesh(
      new BoxGeometry(0.1, 0.035, 0.025),
      new MeshStandardMaterial({
        color: "#c9fafa",
        emissive: "#c9fafa",
        emissiveIntensity: 1.2,
      }),
    );
    light.position.set(x, 0.01, -0.422);
    chassis.add(light);
  }
  const casters: RigidBody[] = [];
  for (const z of [-0.31, 0.31]) {
    const caster = new RigidBody({ world, mass: 0.2, canSleep: false });
    caster.position.set(0, 0.09, z);
    caster.add(new SphereCollider({ radius: 0.09 }));
    caster.add(new Mesh(new SphereGeometry(0.09, 16, 12), metal));
    const joint = new SphericalJoint({
      body0: chassis,
      body1: caster,
      frame0: new Matrix4().makeTranslation(0, -0.27, z),
      frame1: new Matrix4(),
    });
    scene.add(caster, joint);
    casters.push(caster);
  }
  scene.add(chassis);
  function wheel(name: string, x: number) {
    const body = new RigidBody({ world, mass: 1, canSleep: false });
    body.name = name;
    body.position.set(x, 0.22, 0);
    body.setMaterial({
      staticFriction: 1.5,
      dynamicFriction: 1.5,
      restitution: 0,
    });
    const collider = new CylinderCollider({ radius: 0.22, height: 0.13 });
    collider.rotation.z = Math.PI / 2;
    body.add(collider);
    const tire = new Mesh(new CylinderGeometry(0.22, 0.22, 0.13, 32), dark);
    tire.rotation.z = Math.PI / 2;
    tire.castShadow = true;
    body.add(tire);
    const hub = new Mesh(new CylinderGeometry(0.105, 0.105, 0.14, 16), metal);
    hub.rotation.z = Math.PI / 2;
    body.add(hub);
    const stripe = new Mesh(new BoxGeometry(0.145, 0.025, 0.32), orange);
    body.add(stripe);
    // Orient the actuator coordinate so positive targets drive the chassis toward -Z.
    const frame0 = new Matrix4()
      .makeRotationY(Math.PI)
      .setPosition(x, -0.14, 0);
    const frame1 = new Matrix4().makeRotationY(Math.PI);
    const joint = new RevoluteJoint({
      body0: chassis,
      body1: body,
      axis: "X",
      frame0,
      frame1,
    });
    const motor = new JointMotor({ joint, damping: 30, maxForce: 10 });
    scene.add(body, joint);
    return { body, joint, motor };
  }
  const left = wheel("left_wheel", -0.45);
  const right = wheel("right_wheel", 0.45);
  return {
    chassis,
    left,
    right,
    bodies: [chassis, left.body, right.body, ...casters],
  };
}
