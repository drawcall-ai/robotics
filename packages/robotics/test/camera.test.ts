import { expect, it } from "vitest";
import { decode } from "jpeg-js";
import { RenderTarget, Scene, Vector3 } from "three";
import { CameraSensor } from "../src/devices/camera.js";
import { defer, simulation } from "./fixture.js";

const IMAGE = "/robot/camera/image/compressed";
const INFO = "/robot/camera/camera_info";

async function fixture() {
  const { world, ros, connection, renderer } = await simulation();
  const camera = new CameraSensor({ width: 320, height: 180, ros });
  new Scene().add(camera);
  return { world, ros, camera, renderer, messages: connection.messages };
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
  expect(field(image, "format")).toBe("jpeg");
  const data = field(image, "data");
  if (!(data instanceof Uint8Array))
    throw new Error("Image data must be bytes");
  const decoded = decode(data);
  expect([decoded.width, decoded.height]).toEqual([320, 180]);
  expect(decoded.data[0]).toBeCloseTo(200, -1);
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

it("skips a sample while a readback is still in flight instead of queueing captures", async () => {
  const { camera, renderer, messages } = await fixture();
  camera.afterStep(0.2);
  camera.afterStep(0.4);
  await renderer.flush();
  camera.afterStep(0.6);
  await renderer.flush();
  expect(messages.filter((message) => message.name === IMAGE)).toHaveLength(2);
});

it("publishes only image and calibration from a step, never a camera pose", async () => {
  const { world, camera, renderer, messages } = await fixture();
  camera.position.set(3, 4, 5);
  camera.rotation.set(0.2, 0.7, -0.1);
  world.update(0.01);
  await renderer.flush();
  expect(new Set(messages.map(({ name }) => name))).toEqual(
    new Set(["/clock", IMAGE, INFO]),
  );
});

it("drops a frame whose readback completes after disposal", async () => {
  const { ros, camera, renderer, messages } = await fixture();
  camera.afterStep(0.2);
  ros.dispose();
  await renderer.flush();
  expect(messages).toHaveLength(0);
});
