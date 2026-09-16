import {
  BoxCollider,
  CylinderCollider,
  JointMotor,
  RevoluteJoint,
  RigidBody,
  SphereCollider,
  SphericalJoint,
} from "@drawcall/physics";
import { CameraSensor, LaserSensor, Robot } from "@drawcall/robotics";
import {
  AmbientLight,
  BoxGeometry,
  Color,
  CylinderGeometry,
  DirectionalLight,
  Fog,
  HemisphereLight,
  Matrix4,
  Mesh,
  MeshStandardMaterial,
  Scene,
  SphereGeometry,
} from "three";

const scene = new Scene();
const bot = new Robot();
bot.name = "warehouse";
scene.add(bot);
const concrete = new MeshStandardMaterial({
  color: "#b7b9ad",
  roughness: 0.95,
});
const steel = new MeshStandardMaterial({
  color: "#465b61",
  roughness: 0.7,
  metalness: 0.3,
});
const wood = new MeshStandardMaterial({ color: "#ba8f60", roughness: 0.95 });
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

/** A static box with matching collider and mesh. */
function block(
  size: [number, number, number],
  position: [number, number, number],
  material: MeshStandardMaterial,
) {
  const body = new RigidBody({ type: "static" });
  body.position.set(...position);
  const mesh = new Mesh(new BoxGeometry(...size), material);
  mesh.castShadow = mesh.receiveShadow = true;
  body.add(new BoxCollider({ size }), mesh);
  scene.add(body);
}

function warehouse() {
  scene.background = new Color("#cbd2c9");
  scene.fog = new Fog("#cbd2c9", 20, 48);
  scene.add(
    new HemisphereLight("#ecf6ee", "#617471", 2.6),
    new AmbientLight("#ffffff", 0.35),
  );
  const sun = new DirectionalLight("#fff5dd", 3.5);
  sun.position.set(0, 12, 5);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  Object.assign(sun.shadow.camera, {
    left: -12,
    right: 12,
    top: 12,
    bottom: -12,
    near: 0.1,
    far: 40,
  });
  scene.add(sun);
  block([14, 0.2, 13], [-1, -0.1, -2], concrete);
  // Perimeter walls give the lidar returns around the loading area.
  for (const z of [-8.4, 4.4]) block([14, 1.2, 0.18], [-1, 0.6, z], concrete);
  for (const x of [-7.9, 5.9]) block([0.18, 1.2, 13], [x, 0.6, -2], concrete);
  // Two rows of racks leave the central aisle clear for the patrol.
  for (const x of [2.4, -5.6])
    for (let bay = 0; bay < 3; bay++) {
      const z = -5.6 + bay * 2.25;
      for (const dx of [-0.6, 0.6])
        for (const dz of [-0.95, 0.95])
          block([0.07, 2.5, 0.07], [x + dx, 1.25, z + dz], steel);
      for (const y of [0.16, 1.05, 1.94]) {
        block([1.3, 0.07, 2.05], [x, y, z], steel);
        for (const dz of [-0.48, 0.48])
          block([0.93, 0.5, 0.78], [x, y + 0.3, z + dz], wood);
      }
    }
  block([1.1, 0.7, 0.9], [0.5, 0.5, -7], orange);
  block([0.9, 0.6, 0.9], [-2.2, 0.45, -7.1], wood);
}

/** Differential-drive robot: chassis on two driven wheels and two free-rolling support balls. */
function assemble() {
  const chassis = new RigidBody({ mass: 12, canSleep: false });
  chassis.name = "base_link";
  chassis.position.set(0, 0.36, 0);
  const shell = new Mesh(new BoxGeometry(0.66, 0.22, 0.82), orange);
  shell.castShadow = true;
  chassis.add(new BoxCollider({ size: [0.66, 0.22, 0.82] }), shell);
  const lidar = new Mesh(new CylinderGeometry(0.09, 0.09, 0.06, 24), dark);
  lidar.position.set(0, 0.21, -0.32);
  chassis.add(lidar);
  bot.add(chassis);
  for (const z of [-0.31, 0.31]) {
    const ball = new RigidBody({ mass: 0.2, canSleep: false });
    ball.position.set(0, 0.09, z);
    ball.add(
      new SphereCollider({ radius: 0.09 }),
      new Mesh(new SphereGeometry(0.09, 16, 12), metal),
    );
    bot.add(
      ball,
      new SphericalJoint({
        body0: chassis,
        body1: ball,
        frame0: new Matrix4().makeTranslation(0, -0.27, z),
        frame1: new Matrix4(),
      }),
    );
  }
  function wheel(name: string, x: number) {
    const body = new RigidBody({ mass: 1, canSleep: false });
    body.name = name;
    body.position.set(x, 0.22, 0);
    body.setMaterial({
      staticFriction: 1.5,
      dynamicFriction: 1.5,
      restitution: 0,
    });
    const collider = new CylinderCollider({ radius: 0.22, height: 0.13 });
    const tire = new Mesh(new CylinderGeometry(0.22, 0.22, 0.13, 32), dark);
    collider.rotation.z = tire.rotation.z = Math.PI / 2;
    tire.castShadow = true;
    body.add(collider, tire);
    // Positive joint velocity drives the chassis toward -Z, its forward direction.
    const joint = new RevoluteJoint({
      body0: chassis,
      body1: body,
      axis: "X",
      frame0: new Matrix4().makeRotationY(Math.PI).setPosition(x, -0.14, 0),
      frame1: new Matrix4().makeRotationY(Math.PI),
    });
    joint.name = name;
    new JointMotor({ joint, damping: 30, maxForce: 10 });
    bot.add(body, joint);
  }
  wheel("left_wheel", -0.45);
  wheel("right_wheel", 0.45);
  return chassis;
}

warehouse();
const chassis = assemble();

const laser = new LaserSensor({
  name: "scan",
  frameId: "laser",
  samples: 361,
  range: 12,
  rate: 10,
});
laser.position.set(0, 0.24, -0.32);
const camera = new CameraSensor({
  name: "camera",
  frameId: "camera_optical",
  width: 320,
  height: 180,
  fov: 70,
  rate: 5,
});
camera.position.set(0, 0.28, -0.38);
chassis.add(laser, camera);
const overview = new CameraSensor({
  name: "overview",
  frameId: "overview",
  width: 640,
  height: 360,
  fov: 43,
  rate: 5,
});
overview.position.set(10, 11, 12);
overview.lookAt(-1.4, 0.25, -2.4);
scene.add(overview);
