import { PerspectiveCamera, WebGLRenderTarget, SRGBColorSpace } from "three";
import { getROS2, type ROS2 } from "../runtime.js";
import { withDevice } from "../device.js";
import { Schedule } from "../time.js";
import { header } from "../messages.js";

export class CameraSensor extends withDevice(PerspectiveCamera) {
  readonly preview: HTMLCanvasElement;
  private readonly schedule: Schedule;
  private readonly frameId: string;
  private readonly topic: string;
  private readonly target: WebGLRenderTarget;
  private readonly pixels: Uint8Array;
  private readonly image: ImageData;
  private readonly context: CanvasRenderingContext2D;
  constructor(
    options: {
      name?: string;
      width?: number;
      height?: number;
      fov?: number;
      rate?: number;
      frameId?: string;
      ros?: ROS2;
    } = {},
  ) {
    const width = options.width ?? 320,
      height = options.height ?? 180;
    if (
      ![width, height].every((n) => Number.isInteger(n) && n > 0 && n <= 4096)
    )
      throw new Error("Camera dimensions must be integers in [1,4096]");
    const fov = options.fov ?? 70;
    if (!Number.isFinite(fov) || fov <= 0 || fov >= 180)
      throw new Error("Camera FOV must be between 0 and 180 degrees");
    const ros = options.ros ?? getROS2();
    const schedule = new Schedule(options.rate ?? 5);
    const topic = options.name ?? "camera";
    ros.topic(`${topic}/image/compressed`);
    const frameId = options.frameId ?? `${topic}_optical`;
    header(0, frameId);
    const target = new WebGLRenderTarget(width, height);
    target.texture.colorSpace = SRGBColorSpace;
    const pixels = new Uint8Array(width * height * 4);
    const preview = document.createElement("canvas");
    preview.width = width;
    preview.height = height;
    const context = preview.getContext("2d");
    if (!context) throw new Error("Camera capture needs a 2D canvas context");
    const image = context.createImageData(width, height);
    super({ ros, args: [fov, width / height, 0.05, 100] });
    this.schedule = schedule;
    this.topic = topic;
    this.frameId = frameId;
    this.target = target;
    this.pixels = pixels;
    this.preview = preview;
    this.context = context;
    this.image = image;
  }
  afterStep(time: number): void {
    if (!this.schedule.due(time)) return;
    const renderer = this.ros.renderer;
    const previous = renderer.getRenderTarget();
    const face = renderer.getActiveCubeFace();
    const mip = renderer.getActiveMipmapLevel();
    const scissorTest = renderer.getScissorTest();
    const xr = renderer.xr.enabled;
    const { width, height } = this.target;
    try {
      this.ros.scene.updateMatrixWorld(true);
      this.updateWorldMatrix(true, false);
      renderer.xr.enabled = false;
      renderer.setRenderTarget(this.target);
      renderer.setScissorTest(false);
      renderer.clear();
      renderer.render(this.ros.scene, this);
      renderer.readRenderTargetPixels(
        this.target,
        0,
        0,
        width,
        height,
        this.pixels,
      );
    } finally {
      renderer.setScissorTest(scissorTest);
      renderer.setRenderTarget(previous, face, mip);
      renderer.xr.enabled = xr;
    }
    for (let y = 0; y < height; y++) {
      const start = (height - 1 - y) * width * 4;
      this.image.data.set(
        this.pixels.subarray(start, start + width * 4),
        y * width * 4,
      );
    }
    this.context.putImageData(this.image, 0, 0);
    const data = this.preview.toDataURL("image/jpeg", 0.8).split(",")[1];
    if (!data) throw new Error("Camera JPEG encoding failed");
    this.ros.publish(
      `${this.topic}/image/compressed`,
      "sensor_msgs/msg/CompressedImage",
      { header: header(time, this.frameId), format: "jpeg", data },
    );
    const projection = this.projectionMatrix.elements;
    const fx = (projection[0] * width) / 2;
    const fy = (projection[5] * height) / 2;
    const cx = ((1 - projection[8]) * width) / 2;
    const cy = ((1 + projection[9]) * height) / 2;
    this.ros.publish(
      `${this.topic}/camera_info`,
      "sensor_msgs/msg/CameraInfo",
      {
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
        roi: {
          x_offset: 0,
          y_offset: 0,
          height: 0,
          width: 0,
          do_rectify: false,
        },
      },
    );
  }
  dispose(): void {
    super.dispose();
    this.target.dispose();
    this.removeFromParent();
  }
}
