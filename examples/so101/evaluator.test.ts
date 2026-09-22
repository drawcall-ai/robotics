import { afterEach, expect, it, vi } from "vitest";
import type { ROS2 } from "@drawcall/robotics";
import type { PhysicsWorld } from "@drawcall/physics";
import { Vector3 } from "three";

let runtime: ROS2 | undefined;
let world: PhysicsWorld | undefined;
afterEach(async () => {
  runtime?.dispose();
  world?.dispose();
  const { registry } = await import("@drawcall/physics");
  registry.clear();
  vi.restoreAllMocks();
});

async function episode() {
  vi.resetModules();
  const { RigidBody } = await import("@drawcall/physics");
  const { ROS2, Evaluator, Robot } = await import("@drawcall/robotics");
  let elapsed = 0;
  const ros = new ROS2({
    connected: true,
    publish() {},
    subscribe: () => () => {},
    dispose() {},
  });
  runtime = ros;
  await import("./scene.ts");
  const { buildWorld } = await import("@drawcall/physics-mujoco");
  const clock = await buildWorld();
  Object.defineProperty(clock, "time", { get: () => elapsed });
  world = clock;
  ros.attachWorld(clock);
  const evaluator = ros.find(Evaluator)[0];
  const robot = ros.find(Robot)[0];
  if (!evaluator || !robot?.parent) throw new Error("Incomplete task scene");
  const bodies = robot.parent.children.filter(
    (o): o is InstanceType<typeof RigidBody> => o instanceof RigidBody,
  );
  const bowl = bodies.find((o) => o.name === "bowl");
  const objects = bodies.filter((o) => o.name.startsWith("object_"));
  if (!bowl || objects.length !== 3) throw new Error("Missing task objects");
  evaluator.start();
  const sample = (time: number) => {
    elapsed = time;
    evaluator.evaluate();
    return evaluator.result;
  };
  const fill = () => {
    objects.forEach((object, index) => {
      object.position
        .set((index - 1) * 0.026, 0.019, 0)
        .applyQuaternion(bowl.quaternion)
        .add(bowl.position);
      object.quaternion.copy(bowl.quaternion);
    });
  };
  return { evaluator, robot, bowl, objects, sample, fill };
}

it("requires a continuous settled interval after the robot releases the objects", async () => {
  const { evaluator, robot, objects, sample, fill } = await episode();
  fill();
  const object = objects[0];
  if (!object) throw new Error("Missing cube");
  object.dispatchEvent({ type: "contactbegin", otherBody: robot.base });
  sample(0.1);
  expect(sample(2)?.status).toBe("running");
  object.dispatchEvent({ type: "contactend", otherBody: robot.base });
  sample(2.1);
  object.setVelocity({ linear: new Vector3(0.03, 0, 0) });
  sample(3);
  object.setVelocity({ linear: new Vector3() });
  sample(3.1);
  expect(sample(4)?.status).toBe("running");
  expect(sample(4.7)?.status).toBe("success");
  expect(evaluator.result?.metrics.score).toBe(100);
});

it("retains peak bowl displacement and rejects moving the bowl beyond tolerance", async () => {
  const { bowl, sample } = await episode();
  bowl.position.x += 0.01;
  expect(sample(1)?.metrics.peakBowlDisplacement).toBeCloseTo(0.01);
  bowl.position.x -= 0.01;
  expect(sample(2)?.metrics.peakBowlDisplacement).toBeCloseTo(0.01);
  bowl.position.x += 0.021;
  expect(sample(3)).toMatchObject({ status: "failure", reason: "bowlMoved" });
});

it("fails excessive bowl rotation even if every object is inside", async () => {
  const { bowl, sample, fill } = await episode();
  bowl.rotateY(Math.PI / 16);
  fill();
  expect(sample(2)).toMatchObject({ status: "failure", reason: "bowlRotated" });
});

it("fails a dropped object", async () => {
  const { objects, sample } = await episode();
  const object = objects[0];
  if (!object) throw new Error("Missing cube");
  object.position.y = 0.6;
  expect(sample(1)).toMatchObject({
    status: "failure",
    reason: "objectDropped",
  });
});

it("times out an unfinished episode", async () => {
  const { sample } = await episode();
  expect(sample(240)).toMatchObject({ status: "timeout", reason: "timeLimit" });
});
