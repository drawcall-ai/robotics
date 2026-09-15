import { afterEach, expect, it, vi } from "vitest";
import {
  Scene,
  Vector3,
  Vector4,
  WebGLRenderer,
  WebGLRenderTarget,
  WebGLCubeRenderTarget,
} from "three";
import type { Connection } from "../src/connection.js";
import { setupWorld } from "@drawcall/physics-rapier";
import { ROS2 } from "../src/runtime.js";
import { CameraSensor } from "../src/camera.js";

vi.mock("three", async (importOriginal) => {
  const three = await importOriginal<typeof import("three")>();
  return {
    ...three,
    WebGLRenderer: class {
      xr = { enabled: true };
      private target: WebGLRenderTarget | null = null;
      private face = 0;
      private mip = 0;
      private scissorTest = true;
      private viewport = new three.Vector4(0, 0, 1024, 768);
      private canvasViewport = this.viewport.clone();
      getRenderTarget() {
        return this.target;
      }
      getActiveCubeFace() {
        return this.face;
      }
      getActiveMipmapLevel() {
        return this.mip;
      }
      getScissorTest() {
        return this.scissorTest;
      }
      setScissorTest(value: boolean) {
        this.scissorTest = value;
      }
      getViewport(value: Vector4) {
        return value.copy(this.canvasViewport);
      }
      getCurrentViewport(value: Vector4) {
        return value.copy(this.viewport);
      }
      setViewport(value: Vector4) {
        this.viewport.copy(value);
      }
      getScissor(value: Vector4) {
        return value.copy(this.canvasViewport);
      }
      setScissor() {}
      setRenderTarget(target: WebGLRenderTarget | null, face = 0, mip = 0) {
        this.target = target;
        this.face = face;
        this.mip = mip;
        this.viewport.copy(target?.viewport ?? this.canvasViewport);
      }
      clear() {}
      render() {}
      readRenderTargetPixels() {}
    },
  };
});

const cleanup: (() => void)[] = [];
afterEach(() => {
  for (const release of cleanup.splice(0).reverse()) release();
  vi.restoreAllMocks();
  vi.unstubAllGlobals();
});

async function fixture() {
  vi.stubGlobal("document", {
    createElement: () => ({
      width: 0,
      height: 0,
      getContext: () => ({
        createImageData: (width: number, height: number) => ({
          data: new Uint8ClampedArray(width * height * 4),
        }),
        putImageData() {},
      }),
      toDataURL: () => "data:image/jpeg;base64,cGl4ZWxz",
    }),
  });
  const world = await setupWorld({ gravity: [0, 0, 0] });
  cleanup.push(() => world.dispose());
  const messages: { name: string; message: unknown }[] = [];
  const connection: Connection = {
    connected: true,
    dropped: 0,
    publish(name, _type, message) {
      messages.push({ name, message });
    },
    subscribe() {
      return () => {};
    },
    dispose() {},
  };
  const renderer = new WebGLRenderer();
  const ros = new ROS2(connection, { world, scene: new Scene(), renderer });
  cleanup.push(() => ros.dispose());
  const camera = new CameraSensor({ width: 320, height: 180, ros });
  return { camera, renderer, messages };
}

it("publishes calibration that projects points like the actual zoomed and offset Three camera", async () => {
  const { camera, messages } = await fixture();
  camera.zoom = 2;
  camera.filmOffset = 3;
  camera.setViewOffset(640, 360, 30, 40, 320, 180);
  camera.updateProjectionMatrix();
  camera.afterStep(0.25);
  const info = messages.find(
    (message) => message.name === "/robot/camera/camera_info",
  )?.message;
  if (
    typeof info !== "object" ||
    info === null ||
    !("k" in info) ||
    !Array.isArray(info.k)
  )
    throw new Error("Missing camera calibration");
  const calibration = info.k;
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

it("restores the previous render target, face, mip, viewport, scissor and XR after rendering fails", async () => {
  const { camera, renderer, messages } = await fixture();
  const previous = new WebGLCubeRenderTarget(128);
  cleanup.push(() => previous.dispose());
  previous.viewport.set(10, 20, 90, 60);
  renderer.setRenderTarget(previous, 4, 2);
  renderer.setScissorTest(true);
  renderer.xr.enabled = true;
  vi.spyOn(renderer, "render").mockImplementationOnce(() => {
    throw new Error("GPU render failed");
  });
  expect(() => camera.afterStep(0.1)).toThrow("GPU render failed");
  expect(renderer.getRenderTarget()).toBe(previous);
  expect(renderer.getActiveCubeFace()).toBe(4);
  expect(renderer.getActiveMipmapLevel()).toBe(2);
  expect(renderer.getCurrentViewport(new Vector4()).toArray()).toEqual([
    10, 20, 90, 60,
  ]);
  expect(renderer.getScissorTest()).toBe(true);
  expect(renderer.xr.enabled).toBe(true);
  expect(messages).toHaveLength(0);
});
