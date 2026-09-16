import { afterEach, expect, it } from "vitest";
import { decode } from "jpeg-js";
import { RenderTarget, Scene, Vector3 } from "three";
import { ConnectionStub } from "./connection.js";
import { setupWorld } from "@drawcall/physics-rapier";
import { ROS2 } from "../src/runtime.js";
import { CameraSensor } from "../src/sensors/camera.js";
import { RendererStub } from "./renderer.js";

const cleanup: (() => void)[] = [];
afterEach(() => {
  for (const release of cleanup.splice(0).reverse()) release();
});

async function fixture() {
  const world = await setupWorld({ gravity: [0, 0, 0] });
  cleanup.push(() => world.dispose());
  const connection = new ConnectionStub();
  const { messages } = connection;
  const renderer = new RendererStub();
  const ros = new ROS2(connection, { world, renderer });
  cleanup.push(() => ros.dispose());
  const camera = new CameraSensor({ width: 320, height: 180, ros });
  new Scene().add(camera);
  return { camera, renderer, messages, ros };
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
  const info = messages.find(
    (message) => message.name === "/robot/camera/camera_info",
  )?.message;
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
  const image = messages.find(
    (message) => message.name === "/robot/camera/image/compressed",
  )?.message;
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
  cleanup.push(() => previous.dispose());
  renderer.setRenderTarget(previous);
  renderer.failRender = true;
  expect(() => camera.afterStep(0.1)).toThrow("GPU render failed");
  expect(renderer.getRenderTarget()).toBe(previous);
  await renderer.flush();
  expect(messages).toHaveLength(0);
});

it("skips samples while a readback is still in flight instead of queueing captures", async () => {
  const { camera, renderer, messages } = await fixture();
  camera.afterStep(0.2);
  camera.afterStep(0.4);
  expect(renderer.renders).toBe(1);
  await renderer.flush();
  camera.afterStep(0.6);
  await renderer.flush();
  expect(renderer.renders).toBe(2);
  expect(
    messages.filter(
      (message) => message.name === "/robot/camera/image/compressed",
    ),
  ).toHaveLength(2);
});

it("publishes only image and calibration, never a camera world pose", async () => {
  const { camera, renderer, messages } = await fixture();
  camera.position.set(3, 4, 5);
  camera.rotation.set(0.2, 0.7, -0.1);
  camera.afterStep(0.25);
  await renderer.flush();
  expect(messages.map(({ name }) => name)).toEqual([
    "/robot/camera/image/compressed",
    "/robot/camera/camera_info",
  ]);
});

it("drops a frame whose readback completes after disconnection", async () => {
  const { camera, renderer, messages, ros } = await fixture();
  camera.afterStep(0.2);
  ros.dispose();
  await renderer.flush();
  expect(messages).toHaveLength(0);
});
