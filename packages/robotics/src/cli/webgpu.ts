import { create, globals } from "webgpu";
import { WebGPURenderer } from "three/webgpu";

Object.assign(globalThis, globals);
// The renderer schedules its internal loop through browser globals that Node lacks.
Object.assign(globalThis, {
  self: globalThis,
  requestAnimationFrame: (callback: FrameRequestCallback) =>
    setTimeout(() => callback(performance.now()), 16),
  cancelAnimationFrame: (handle: number) => clearTimeout(handle),
});

// Dawn frees its instance once JavaScript drops the last reference and its event pump then crashes,
// so the GPU, adapter and device stay referenced for the life of the process.
const gpu: { instance: GPU; adapter?: GPUAdapter; device?: GPUDevice } = {
  instance: create([]),
};

/** A Three.js renderer on Dawn. */
export async function createRenderer(): Promise<WebGPURenderer> {
  const adapter = await gpu.instance.requestAdapter();
  if (!adapter)
    throw new Error(
      "No WebGPU adapter; install a Vulkan driver such as mesa-vulkan-drivers",
    );
  gpu.adapter = adapter;
  const device = await adapter.requestDevice();
  gpu.device = device;
  const renderer = new WebGPURenderer({ device, canvas: new HeadlessCanvas() });
  using cleanup = new DisposableStack();
  cleanup.defer(() => renderer.dispose());
  await renderer.init();
  renderer.setSize(1, 1, false);
  // Lights that cast shadows get them; the others cost nothing.
  renderer.shadowMap.enabled = true;
  cleanup.move();
  return renderer;
}

// Three requires canvas dimensions even when all rendering uses offscreen targets.
// Dawn supplies no presentation surface, so attempts to use one fail explicitly.
class HeadlessCanvas extends EventTarget implements OffscreenCanvas {
  width = 1;
  height = 1;
  oncontextlost = null;
  oncontextrestored = null;

  getContext(): never {
    throw new Error(
      "Headless rendering has no canvas context; use a render target",
    );
  }

  async convertToBlob(): Promise<never> {
    throw new Error(
      "Headless rendering has no canvas image; read a render target",
    );
  }

  transferToImageBitmap(): never {
    throw new Error(
      "Headless rendering has no canvas image; read a render target",
    );
  }
}
