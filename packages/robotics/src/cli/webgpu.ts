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

/** A Three.js renderer on Dawn. Call it only after Rapier is instantiated: the reverse order crashes Node. */
export async function createRenderer(): Promise<WebGPURenderer> {
  const adapter = await gpu.instance.requestAdapter();
  if (!adapter)
    throw new Error(
      "No WebGPU adapter; install a Vulkan driver such as mesa-vulkan-drivers",
    );
  gpu.adapter = adapter;
  const device = await adapter.requestDevice();
  gpu.device = device;
  // Dawn has no canvas or swapchain. Sensors only render into targets, so the renderer gets inert stand-ins.
  const context = {
    configure() {},
    getCurrentTexture(): GPUTexture {
      throw new Error("Headless rendering has no canvas to present to");
    },
  } as unknown as GPUCanvasContext;
  const canvas = {
    width: 1,
    height: 1,
    clientWidth: 1,
    clientHeight: 1,
    style: {},
    addEventListener() {},
    removeEventListener() {},
    setAttribute() {},
    getContext: () => context,
  } as unknown as HTMLCanvasElement;
  const renderer = new WebGPURenderer({ device, canvas, context });
  await renderer.init();
  renderer.setSize(1, 1, false);
  return renderer;
}
