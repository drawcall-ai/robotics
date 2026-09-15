import { BoxCollider, RigidBody } from "@drawcall/physics";
import type { PhysicsWorld } from "@drawcall/physics";
import {
  BoxGeometry,
  CanvasTexture,
  Mesh,
  MeshBasicMaterial,
  MeshStandardMaterial,
  PlaneGeometry,
} from "three";
import type { Scene } from "three";

export function createWarehouse(world: PhysicsWorld, scene: Scene) {
  const steel = new MeshStandardMaterial({
    color: "#465b61",
    roughness: 0.72,
    metalness: 0.3,
  });
  const orange = new MeshStandardMaterial({
    color: "#c67739",
    roughness: 0.65,
  });
  const concrete = new MeshStandardMaterial({
    color: "#b7b9ad",
    roughness: 0.95,
  });
  const wood = new MeshStandardMaterial({ color: "#ba8f60", roughness: 0.95 });
  const paint = new MeshStandardMaterial({ color: "#e9df98", roughness: 0.8 });
  function box(
    size: [number, number, number],
    position: [number, number, number],
    material: MeshStandardMaterial,
    solid = true,
  ) {
    const mesh = new Mesh(new BoxGeometry(...size), material);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    if (!solid) {
      mesh.position.set(...position);
      scene.add(mesh);
      return;
    }
    const body = new RigidBody({ world, type: "static" });
    body.position.set(...position);
    body.add(new BoxCollider({ size }), mesh);
    scene.add(body);
  }
  box([14, 0.2, 13], [-1, -0.1, -2], concrete);
  box([14, 0.12, 0.18], [-1, 0.06, -8.4], steel);
  box([0.18, 0.12, 13], [-7.9, 0.06, -2], steel);
  const line = (x: number, z: number, width: number, depth: number) =>
    box([width, 0.006, depth], [x, 0.004, z], paint, false);
  for (const x of [-1.7, 1.1, -4.1]) {
    for (let z = -5.3; z < 2; z += 0.68) line(x, z, 0.045, 0.32);
  }
  // The central aisle stays clear for the four ROS patrol waypoints.
  for (const x of [2.4, -5.6]) {
    for (let bay = 0; bay < 3; bay++) {
      const z = -5.6 + bay * 2.25;
      for (const side of [-0.6, 0.6]) {
        for (const end of [-0.95, 0.95])
          box([0.07, 2.5, 0.07], [x + side, 1.25, z + end], steel);
      }
      for (const y of [0.16, 1.05, 1.94]) {
        box([1.3, 0.07, 2.05], [x, y, z], steel);
        for (const side of [-0.63, 0.63])
          box([0.035, 0.1, 2.05], [x + side, y + 0.045, z], orange, false);
        for (const offset of [-0.48, 0.48]) {
          const height = 0.46 + ((bay + Math.round(y)) % 2) * 0.15;
          box(
            [0.93, height, 0.78],
            [x, y + 0.035 + height / 2, z + offset],
            wood,
          );
          box(
            [0.95, 0.025, 0.12],
            [x, y + 0.05 + height, z + offset],
            paint,
            false,
          );
        }
      }
      label(
        `${x > 0 ? "A" : "B"} / 0${bay + 1}`,
        x,
        2.6,
        z,
        x > 0 ? -Math.PI / 2 : Math.PI / 2,
      );
    }
  }
  box([1.3, 0.15, 1.1], [0.5, 0.075, -7], wood);
  box([1.1, 0.7, 0.9], [0.5, 0.5, -7], orange);
  box([1.1, 0.15, 1.1], [-2.2, 0.075, -7.1], wood);
  box([0.9, 0.6, 0.9], [-2.2, 0.45, -7.1], wood);
  for (let x = -3.5; x < 1.5; x += 0.24)
    box([0.11, 0.01, 0.35], [x, 0.01, 2.3], paint, false);
  label("DISPATCH  /  01", -1, 0.014, 1.65, 0, true);
  // The final approach travels +X. This marker faces that aisle beyond the goal.
  box([0.12, 0.95, 0.85], [1.4, 0.475, 0], steel);
  const dock = new Mesh(
    new PlaneGeometry(0.7, 0.62),
    new MeshBasicMaterial({ color: "#e89020" }),
  );
  dock.position.set(1.335, 0.64, 0);
  dock.rotation.y = -Math.PI / 2;
  scene.add(dock);
  for (const [i, x, z] of [
    [1, 0, -4],
    [2, -3, -4],
    [3, -3, 0],
    [4, 0, 0],
  ]) {
    label(`0${i}`, x, 0.017, z, 0, true, 0.45);
  }
  function label(
    text: string,
    x: number,
    y: number,
    z: number,
    rotation: number,
    floor = false,
    width = 1.2,
  ) {
    const canvas = document.createElement("canvas");
    canvas.width = 512;
    canvas.height = 128;
    const ctx = canvas.getContext("2d");
    if (!ctx) throw new Error("Canvas 2D is required for warehouse labels");
    ctx.fillStyle = floor ? "#b7b9ad" : "#edf0df";
    ctx.fillRect(0, 0, 512, 128);
    ctx.fillStyle = "#46534e";
    ctx.font = "bold 54px monospace";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(text, 256, 64);
    const sign = new Mesh(
      new PlaneGeometry(width, width / 4),
      new MeshStandardMaterial({
        map: new CanvasTexture(canvas),
        roughness: 1,
      }),
    );
    sign.position.set(x, y, z);
    if (floor) sign.rotation.x = -Math.PI / 2;
    else sign.rotation.y = rotation;
    scene.add(sign);
  }
}
