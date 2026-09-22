import { expect, it } from "vitest";
import { Object3D } from "three";
import { BoxCollider, RigidBody } from "@drawcall/physics";
import { LaserSensor } from "../src/devices/laser.js";
import { simulation } from "./fixture.js";

/** Five samples over the full circle: behind, right, ahead, left, behind again. */
const SAMPLES = 5;
const CLEAR = 7; // beyond the default range of 6

function wall(
  position: [number, number, number],
  size: [number, number, number],
): void {
  const body = new RigidBody({ type: "static" });
  body.position.set(...position);
  body.add(new BoxCollider({ size }));
}
/** A wall 2.9 m ahead (Three -Z) and one 1.9 m to the left (Three -X). */
function room(): void {
  wall([0, 0, -3], [4, 4, 0.2]);
  wall([-2, 0, 0], [0.2, 4, 4]);
}
function scan(ranges: unknown[]) {
  return {
    name: "/robot/scan",
    type: "sensor_msgs/msg/LaserScan",
    message: expect.objectContaining({ ranges }),
  };
}

it("measures each wall at its bearing and reports clear directions beyond range", async () => {
  const { world, ros, connection } = await simulation();
  room();
  new LaserSensor({ ros, samples: SAMPLES });
  world.update(0.01);
  expect(connection.messages).toContainEqual(
    scan([CLEAR, CLEAR, expect.closeTo(2.9, 5), expect.closeTo(1.9, 5), CLEAR]),
  );
});

it("scans in the frame of the body it sits on", async () => {
  const { world, ros, connection } = await simulation();
  room();
  const mount = new Object3D();
  mount.rotation.y = Math.PI / 2;
  mount.add(new LaserSensor({ ros, samples: SAMPLES }));
  world.update(0.01);
  expect(connection.messages).toContainEqual(
    scan([CLEAR, expect.closeTo(2.9, 5), expect.closeTo(1.9, 5), CLEAR, CLEAR]),
  );
});

it("drops returns closer than the minimum range like a real scanner", async () => {
  const { world, ros, connection } = await simulation();
  room();
  new LaserSensor({ ros, samples: SAMPLES, minRange: 2 });
  world.update(0.01);
  expect(connection.messages).toContainEqual(
    scan([CLEAR, CLEAR, expect.closeTo(2.9, 5), CLEAR, CLEAR]),
  );
});

it("describes its sweep, ranges and frame in every scan", async () => {
  const { world, ros, connection } = await simulation();
  const laser = new LaserSensor({
    ros,
    name: "rear",
    frameId: "rear_laser",
    samples: SAMPLES,
    range: 4,
    minRange: 0.5,
    rate: 5,
  });
  expect(laser.topic).toBe("/robot/rear");
  world.update(0.01);
  expect(connection.messages).toContainEqual({
    name: "/robot/rear",
    type: "sensor_msgs/msg/LaserScan",
    message: {
      header: {
        stamp: { sec: 0, nanosec: 10_000_000 },
        frame_id: "rear_laser",
      },
      angle_min: -Math.PI,
      angle_max: Math.PI,
      angle_increment: Math.PI / 2,
      time_increment: 0,
      scan_time: 0.2,
      range_min: 0.5,
      range_max: 4,
      ranges: [5, 5, 5, 5, 5],
      intensities: [],
    },
  });
});
