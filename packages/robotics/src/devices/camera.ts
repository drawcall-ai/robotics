import { encode } from "jpeg-js";
import {
  PerspectiveCamera,
  RenderTarget,
  SRGBColorSpace,
  type Object3D,
} from "three";
import { header, identifier } from "../messages.js";
import { getROS2, type CaptureRenderer, type ROS2 } from "../runtime.js";
import { Schedule } from "./schedule.js";
import { robotOf } from "./robot.js";

export class CameraSensor extends PerspectiveCamera {
  readonly ros: ROS2;
  readonly width: number;
  readonly height: number;
  readonly frameId: string;
  private readonly schedule: Schedule;
  private readonly quality: number;
  private readonly target: RenderTarget;
  private readonly unregister: () => void;
  private pending?: Promise<void>;
  private disposed = false;
  constructor(
    options: {
      name?: string;
      width?: number;
      height?: number;
      fov?: number;
      rate?: number;
      quality?: number;
      frameId?: string;
      ros?: ROS2;
    } = {},
  ) {
    const width = options.width ?? 320;
    const height = options.height ?? 180;
    const fov = options.fov ?? 70;
    super(fov, width / height, 0.05, 100);
    if (![width, height].every((n) => Number.isInteger(n) && n > 0))
      throw new Error("Camera dimensions must be positive integers");
    if (!(fov > 0 && fov < 180))
      throw new Error("Camera FOV must be between 0 and 180 degrees");
    const quality = options.quality ?? 80;
    if (!(quality >= 1 && quality <= 100))
      throw new Error("JPEG quality must lie in [1,100]");
    this.name = options.name ?? "camera";
    this.width = width;
    this.height = height;
    this.frameId = identifier(
      "frame",
      options.frameId ?? `${this.name}_optical`,
    );
    this.schedule = new Schedule(options.rate ?? 5);
    this.quality = quality;
    this.target = new RenderTarget(width, height);
    this.target.texture.colorSpace = SRGBColorSpace;
    this.ros = options.ros ?? getROS2();
    this.unregister = this.ros.register(this);
  }
  /** `sensor_msgs/Image`, `rgb8`, as image_transport's raw transport names it. */
  get rawTopic(): string {
    return this.topic("image_raw");
  }
  /** `sensor_msgs/CompressedImage`, JPEG, as image_transport's compressed transport names it. */
  get compressedTopic(): string {
    return this.topic("image_raw/compressed");
  }
  get infoTopic(): string {
    return this.topic("camera_info");
  }
  private topic(name: string): string {
    return this.ros.topic(`${this.name}/${name}`, robotOf(this)?.namespace);
  }
  get busy(): boolean {
    return this.pending !== undefined;
  }
  afterStep(time: number): void {
    // A capture still being read back means the GPU is behind; this sample is skipped rather than queued.
    // The simulator waits while a device is busy, so only loops that step regardless skip samples.
    if (this.disposed || !this.schedule.due(time) || this.pending) return;
    const renderer = this.ros.renderer;
    if (!renderer) throw new Error(`Camera ${this.name} needs a renderer`);
    // A camera captures the scene it is part of.
    let root: Object3D = this;
    while (root.parent) root = root.parent;
    if (root === this) throw new Error(`Camera ${this.name} is not in a scene`);
    const previous = renderer.getRenderTarget();
    try {
      root.updateMatrixWorld(true);
      renderer.setRenderTarget(this.target);
      renderer.render(root, this);
    } finally {
      renderer.setRenderTarget(previous);
    }
    this.pending = this.readback(time, renderer).finally(() => {
      this.pending = undefined;
      if (this.disposed) this.target.dispose();
    });
  }
  private async readback(
    time: number,
    renderer: CaptureRenderer,
  ): Promise<void> {
    try {
      const pixels = await renderer.readRenderTargetPixelsAsync(
        this.target,
        0,
        0,
        this.width,
        this.height,
      );
      this.publish(time, pixels);
    } catch (error) {
      if (this.disposed) throw error;
      this.ros.fail(error);
    }
  }
  private publish(time: number, pixels: ArrayBufferView): void {
    // Disconnection during readback drops the frame; the runtime already stopped.
    if (this.disposed || !this.ros.connected) return;
    if (!(pixels instanceof Uint8Array))
      throw new Error("Camera readback must produce 8-bit pixels");
    const { width, height } = this;
    const stamped = header(
      time,
      robotOf(this)?.frame(this.frameId) ?? this.frameId,
    );
    // The readback is RGBA; a camera has no alpha.
    const rgb = new Uint8Array(width * height * 3);
    for (let i = 0, j = 0; j < rgb.length; i += 4, j += 3) {
      rgb[j] = pixels[i] ?? 0;
      rgb[j + 1] = pixels[i + 1] ?? 0;
      rgb[j + 2] = pixels[i + 2] ?? 0;
    }
    this.ros.publish(this.rawTopic, "sensor_msgs/msg/Image", {
      header: stamped,
      height,
      width,
      encoding: "rgb8",
      is_bigendian: 0,
      step: width * 3,
      data: rgb,
    });
    const jpeg = encode({ data: pixels, width, height }, this.quality);
    this.ros.publish(this.compressedTopic, "sensor_msgs/msg/CompressedImage", {
      header: stamped,
      format: "rgb8; jpeg compressed bgr8",
      data: new Uint8Array(jpeg.data),
    });
    const projection = this.projectionMatrix.elements;
    const fx = (projection[0] * width) / 2;
    const fy = (projection[5] * height) / 2;
    const cx = ((1 - projection[8]) * width) / 2;
    const cy = ((1 + projection[9]) * height) / 2;
    this.ros.publish(this.infoTopic, "sensor_msgs/msg/CameraInfo", {
      header: stamped,
      width,
      height,
      distortion_model: "plumb_bob",
      d: [0, 0, 0, 0, 0],
      k: [fx, 0, cx, 0, fy, cy, 0, 0, 1],
      r: [1, 0, 0, 0, 1, 0, 0, 0, 1],
      p: [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0],
      binning_x: 0,
      binning_y: 0,
      roi: { x_offset: 0, y_offset: 0, height: 0, width: 0, do_rectify: false },
    });
  }
  /** Detaches immediately; await pending readback cleanup before disposing the renderer. */
  dispose(): Promise<void> | undefined {
    if (this.disposed) return this.pending;
    this.disposed = true;
    this.unregister();
    if (!this.pending) this.target.dispose();
    this.removeFromParent();
    return this.pending;
  }
}
