import { afterEach, expect, it, vi } from "vitest";
import type { PhysicsWorld } from "@drawcall/physics";
import { Euler } from "three";

let world: PhysicsWorld | undefined;
afterEach(async () => {
  world?.dispose();
  const { registry } = await import("@drawcall/physics");
  registry.clear();
  vi.restoreAllMocks();
});

it.each([0.15, 0.4, 0.785])(
  "keeps the bowl still when a cube tilted by %s rad lands on its floor",
  async (tilt) => {
    vi.resetModules();
    const { RigidBody } = await import("@drawcall/physics");
    const { ROS2, Robot } = await import("@drawcall/robotics");
    const ros = new ROS2(
      {
        connected: true,
        publish() {},
        subscribe: () => () => {},
        dispose() {},
      },
      {
        renderer: {
          render() {},
          getRenderTarget: () => null,
          setRenderTarget() {},
          readRenderTargetPixelsAsync: (_target, _x, _y, width, height) =>
            Promise.resolve(new Uint8Array(width * height * 4)),
        },
      },
    );
    await import("./scene.ts");
    const { buildWorld } = await import("@drawcall/physics-mujoco");
    world = await buildWorld({
      gravity: [0, -9.81, 0],
      fixedDelta: 1 / 1000,
      solverIterations: 50,
      frictionImpedanceRatio: 10,
      frictionCone: "elliptic",
    });
    ros.attachWorld(world);
    const bodies = ros
      .find(Robot)[0]
      ?.parent?.children.filter(
        (o): o is InstanceType<typeof RigidBody> => o instanceof RigidBody,
      );
    const bowl = bodies?.find((o) => o.name === "bowl");
    const cube = bodies?.find((o) => o.name === "object_1");
    if (!bowl || !cube) throw new Error("Missing bowl or cube");
    cube.position.copy(bowl.position).setY(bowl.position.y + 0.07);
    cube.setRotationFromEuler(new Euler(tilt, 0.3, 0));
    const start = bowl.position.clone();
    for (let step = 0; step < 2000; step++) world.update(1 / 1000);
    expect(bowl.position.distanceTo(start)).toBeLessThan(0.005);
    expect(cube.position.y - bowl.position.y).toBeLessThan(0.03);
    ros.dispose();
  },
);
