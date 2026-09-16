import type { RenderTarget, Texture } from "three";
import type { CaptureRenderer } from "../src/runtime.js";

/** Records render-target switching and returns a constant pixel value on readback. */
export class RendererStub implements CaptureRenderer {
  target: RenderTarget<Texture> | null = null;
  renders = 0;
  pixel = 7;
  failRender = false;
  readbacks: (() => void)[] = [];
  render(): void {
    if (this.failRender) throw new Error("GPU render failed");
    this.renders++;
  }
  getRenderTarget() {
    return this.target;
  }
  setRenderTarget(target: RenderTarget<Texture> | null): void {
    this.target = target;
  }
  readRenderTargetPixelsAsync(
    _target: RenderTarget,
    _x: number,
    _y: number,
    width: number,
    height: number,
  ): Promise<Uint8Array> {
    return new Promise((resolve) => {
      this.readbacks.push(() =>
        resolve(new Uint8Array(width * height * 4).fill(this.pixel)),
      );
    });
  }
  /** Completes every pending readback. */
  async flush(): Promise<void> {
    for (const resolve of this.readbacks.splice(0)) resolve();
    await new Promise((resolve) => setTimeout(resolve, 0));
  }
}
