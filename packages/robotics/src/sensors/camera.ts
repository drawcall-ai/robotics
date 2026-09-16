import { encode } from "jpeg-js";
import {
  PerspectiveCamera,
  RenderTarget,
  SRGBColorSpace,
  type Object3D,
} from "three";
import { withDevice } from "../device.js";
import { frameId, header } from "../messages.js";
import { getROS2, type ROS2 } from "../runtime.js";
import { Schedule } from "../time.js";

export class CameraSensor extends withDevice(PerspectiveCamera) {
  readonly width: number;
  readonly height: number;
  readonly frameId: string;
  private readonly schedule: Schedule;
  private readonly quality: number;
  private readonly target: RenderTarget;
  readonly imageTopic: string;
  readonly infoTopic: string;
  private pending = false;
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
    if (![width, height].every((n) => Number.isInteger(n) && n > 0))
      throw new Error("Camera dimensions must be positive integers");
    const fov = options.fov ?? 70;
    if (!(fov > 0 && fov < 180))
      throw new Error("Camera FOV must be between 0 and 180 degrees");
    const quality = options.quality ?? 80;
    if (!(quality >= 1 && quality <= 100))
      throw new Error("JPEG quality must lie in [1,100]");
    const ros = options.ros ?? getROS2();
    const name = options.name ?? "camera";
    const imageTopic = ros.topic(`${name}/image/compressed`);
    const infoTopic = ros.topic(`${name}/camera_info`);
    const frame = frameId(options.frameId ?? `${name}_optical`);
    const target = new RenderTarget(width, height);
    target.texture.colorSpace = SRGBColorSpace;
    super({ ros, args: [fov, width / height, 0.05, 100] });
    this.name = name;
    this.width = width;
    this.height = height;
    this.frameId = frame;
    this.schedule = new Schedule(options.rate ?? 5);
    this.quality = quality;
    this.target = target;
    this.imageTopic = imageTopic;
    this.infoTopic = infoTopic;
  }
  afterStep(time: number): void {
    // A capture still being read back means the GPU is behind; this sample is skipped rather than queued.
    if (!this.schedule.due(time) || this.pending) return;
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
    this.pending = true;
    renderer
      .readRenderTargetPixelsAsync(this.target, 0, 0, this.width, this.height)
      .then((pixels) => this.publish(time, pixels))
      .catch((error: unknown) => this.ros.fail(error))
      .finally(() => {
        this.pending = false;
      });
  }
  private publish(time: number, pixels: ArrayBufferView): void {
    // Disconnection during readback drops the frame; the runtime already stopped.
    if (!this.ros.connected) return;
    if (!(pixels instanceof Uint8Array))
      throw new Error("Camera readback must produce 8-bit pixels");
    const { width, height } = this;
    const jpeg = encode({ data: pixels, width, height }, this.quality);
    this.ros.publish(this.imageTopic, "sensor_msgs/msg/CompressedImage", {
      header: header(time, this.frameId),
      format: "jpeg",
      data: new Uint8Array(jpeg.data),
    });
    const projection = this.projectionMatrix.elements;
    const fx = (projection[0] * width) / 2;
    const fy = (projection[5] * height) / 2;
    const cx = ((1 - projection[8]) * width) / 2;
    const cy = ((1 + projection[9]) * height) / 2;
    this.ros.publish(this.infoTopic, "sensor_msgs/msg/CameraInfo", {
      header: header(time, this.frameId),
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
  dispose(): void {
    super.dispose();
    this.target.dispose();
    this.removeFromParent();
  }
}
