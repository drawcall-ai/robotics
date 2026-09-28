import { expect, it, vi } from "vitest";
import { decode } from "jpeg-js";
import { RenderTarget, Scene, Vector3 } from "three";
import { Stepping } from "../src/cli/sim.js";
import { CameraSensor } from "../src/devices/camera.js";
import { defer, simulation } from "./fixture.js";

const IMAGE = "/robot/camera/image_raw/compressed";
const RAW = "/robot/camera/image_raw";
const INFO = "/robot/camera/camera_info";

async function fixture() {
  const { world, ros, connection, renderer } = await simulation();
  const camera = new CameraSensor({ width: 320, height: 180, ros });
  new Scene().add(camera);
  return {
    world,
    ros,
    camera,
    renderer,
    connection,
    messages: connection.messages,
  };
}
function field(message: unknown, key: string): unknown {
  if (typeof message !== "object" || message === null || !(key in message))
    throw new Error(`Missing ${key}`);
  return Reflect.get(message, key);
}

it("publishes calibration that projects points like the actual zoomed and offset Three camera", async () => {
  const { camera, renderer, messages } = await fixture();
  camera.zoom = 2;
  camera.filmOffset = 3;
  camera.setViewOffset(640, 360, 30, 40, 320, 180);
  camera.updateProjectionMatrix();
  camera.afterStep(0.25);
  await renderer.flush();
  const info = messages.find((message) => message.name === INFO)?.message;
  const calibration = field(info, "k");
  if (!Array.isArray(calibration)) throw new Error("Missing calibration");
  const coefficient = (index: number): number => {
    const value: unknown = calibration[index];
    if (typeof value !== "number")
      throw new Error("Invalid camera coefficient");
    return value;
  };
  for (const point of [
    new Vector3(0, 0, -3),
    new Vector3(0.2, -0.1, -2),
    new Vector3(-0.3, 0.25, -4),
  ]) {
    const ndc = point.clone().project(camera);
    const pixelX = ((ndc.x + 1) * 320) / 2;
    const pixelY = ((1 - ndc.y) * 180) / 2;
    expect((coefficient(0) * point.x) / -point.z + coefficient(2)).toBeCloseTo(
      pixelX,
      8,
    );
    expect((coefficient(4) * -point.y) / -point.z + coefficient(5)).toBeCloseTo(
      pixelY,
      8,
    );
  }
  expect(info).toMatchObject({
    header: {
      stamp: { sec: 0, nanosec: 250_000_000 },
      frame_id: "camera_optical",
    },
    width: 320,
    height: 180,
  });
});

it("encodes the readback as a JPEG the size of the sensor", async () => {
  const { camera, renderer, messages } = await fixture();
  renderer.pixel = 200;
  camera.afterStep(0.2);
  await renderer.flush();
  const image = messages.find((message) => message.name === IMAGE)?.message;
  expect(field(image, "format")).toBe("rgb8; jpeg compressed bgr8");
  const data = field(image, "data");
  if (!(data instanceof Uint8Array))
    throw new Error("Image data must be bytes");
  const decoded = decode(data);
  expect([decoded.width, decoded.height]).toEqual([320, 180]);
  expect(decoded.data[0]).toBeCloseTo(200, -1);
});

it("publishes the readback as a raw rgb8 image beside the JPEG", async () => {
  const { camera, renderer, messages } = await fixture();
  renderer.pixel = 200;
  camera.afterStep(0.2);
  await renderer.flush();
  const image = messages.find((message) => message.name === RAW)?.message;
  expect(image).toMatchObject({
    header: { frame_id: "camera_optical" },
    width: 320,
    height: 180,
    encoding: "rgb8",
    step: 960,
  });
  const data = field(image, "data");
  if (!(data instanceof Uint8Array))
    throw new Error("Image data must be bytes");
  expect(data).toHaveLength(320 * 180 * 3);
  expect(data[0]).toBe(200);
});

it("publishes images only while someone subscribes, and calibration always", async () => {
  const { camera, renderer, connection, messages } = await fixture();
  connection.listening = false;
  camera.afterStep(0.2);
  await renderer.flush();
  expect(messages.map((message) => message.name)).not.toContain(IMAGE);
  expect(messages.map((message) => message.name)).not.toContain(RAW);
  expect(messages.some((message) => message.name.endsWith("camera_info"))).toBe(
    true,
  );
});

it("restores the previous render target and publishes nothing after rendering fails", async () => {
  const { camera, renderer, messages } = await fixture();
  const previous = new RenderTarget(128, 128);
  defer(() => previous.dispose());
  renderer.setRenderTarget(previous);
  renderer.failRender = true;
  expect(() => camera.afterStep(0.1)).toThrow("GPU render failed");
  expect(renderer.getRenderTarget()).toBe(previous);
  await renderer.flush();
  expect(messages).toHaveLength(0);
});

it("skips a sample while every readback is still in flight instead of queueing captures", async () => {
  const { camera, renderer, messages } = await fixture();
  for (const time of [0.2, 0.4, 0.6, 0.8]) camera.afterStep(time);
  await renderer.flush();
  camera.afterStep(1.0);
  await renderer.flush();
  const stamps = messages
    .filter((message) => message.name === IMAGE)
    .map((message) => field(field(message.message, "header"), "stamp"));
  expect(stamps).toEqual([
    { sec: 0, nanosec: 200_000_000 },
    { sec: 0, nanosec: 400_000_000 },
    { sec: 0, nanosec: 600_000_000 },
    { sec: 1, nanosec: 0 },
  ]);
});

it("publishes only image and calibration from a step, never a camera pose", async () => {
  const { world, camera, renderer, messages } = await fixture();
  camera.position.set(3, 4, 5);
  camera.rotation.set(0.2, 0.7, -0.1);
  world.update(0.01);
  await renderer.flush();
  expect(new Set(messages.map(({ name }) => name))).toEqual(
    new Set(["/clock", RAW, IMAGE, INFO]),
  );
});

it("drops a frame whose readback completes after disposal", async () => {
  const { ros, camera, renderer, messages } = await fixture();
  camera.afterStep(0.2);
  ros.dispose();
  await renderer.flush();
  expect(messages).toHaveLength(0);
});

it("finishes an individual camera's readback before releasing its target and drops the frame", async () => {
  const { ros, camera, renderer, messages } = await fixture();
  const dispose = vi.spyOn(RenderTarget.prototype, "dispose");
  camera.afterStep(0.2);
  const disposed = camera.dispose();
  expect(camera.dispose()).toBe(disposed);
  const finished = vi.fn();
  disposed?.then(finished);
  expect(ros.find(CameraSensor)).toEqual([]);
  // The idle targets go at once; the one read back into only once it is done.
  expect(dispose).toHaveBeenCalledTimes(2);
  await Promise.resolve();
  expect(finished).not.toHaveBeenCalled();
  await renderer.flush();
  await disposed;
  expect(finished).toHaveBeenCalledOnce();
  expect(dispose).toHaveBeenCalledTimes(3);
  expect(messages).toHaveLength(0);
  expect(ros.connected).toBe(true);
  camera.afterStep(0.4);
  expect(renderer.readbacks).toHaveLength(0);
});

it("reports a disposed camera's failed readback to its owner without faulting the runtime", async () => {
  const { ros, camera, renderer, messages } = await fixture();
  const dispose = vi.spyOn(RenderTarget.prototype, "dispose");
  vi.spyOn(renderer, "readRenderTargetPixelsAsync").mockRejectedValue(
    new Error("GPU readback failed"),
  );
  camera.afterStep(0.2);
  const failure = expect(camera.dispose()).rejects.toThrow(
    "GPU readback failed",
  );
  await renderer.flush();
  await failure;
  expect(dispose).toHaveBeenCalledTimes(3);
  expect(messages).toHaveLength(0);
  expect(ros.connected).toBe(true);
  expect(ros.error).toBeUndefined();
});

it("joins pending camera work through runtime disposal before its target is released", async () => {
  const { ros, camera, renderer, messages } = await fixture();
  const release = vi.spyOn(RenderTarget.prototype, "dispose");
  camera.afterStep(0.2);
  const completion = ros.disposeAsync();
  expect(ros.connected).toBe(false);
  expect(camera.parent).toBe(null);
  expect(release).toHaveBeenCalledTimes(2);
  await renderer.flush();
  await completion;
  expect(release).toHaveBeenCalledTimes(3);
  expect(messages).toHaveLength(0);
});

it("steps on while captures read back, holding only when three are in flight", async () => {
  const { world, ros, renderer, messages } = await fixture();
  const stepping = new Stepping(world, () => ros.busy);
  // Captures at 0.01, 0.2 and 0.4 s fill the ring; physics then waits for a readback.
  for (let i = 0; i < 3; i++) stepping.update(0.2);
  expect(world.time).toBeCloseTo(0.4);
  expect(ros.busy).toBe(true);
  stepping.update(0.1);
  expect(world.time).toBeCloseTo(0.4);
  await renderer.flush();
  expect(ros.busy).toBe(false);
  // Freed, it catches up on the time it was held for, capturing at 0.6 s on the way.
  stepping.update(0.2);
  expect(world.time).toBeGreaterThan(0.6);
  await renderer.flush();
  const stamps = messages
    .filter((message) => message.name === IMAGE)
    .map((message) => field(field(message.message, "header"), "stamp"));
  expect(stamps).toEqual([
    { sec: 0, nanosec: 10_000_000 },
    { sec: 0, nanosec: 200_000_000 },
    { sec: 0, nanosec: 400_000_000 },
    { sec: 0, nanosec: 600_000_000 },
  ]);
});
