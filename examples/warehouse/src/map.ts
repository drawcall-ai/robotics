export function drawMap(canvas: HTMLCanvasElement, message: unknown): void {
  if (
    typeof message !== "object" ||
    message === null ||
    !("info" in message) ||
    !("data" in message) ||
    !Array.isArray(message.data)
  )
    throw new Error("Invalid occupancy grid");
  const info = message.info;
  if (
    typeof info !== "object" ||
    info === null ||
    !("width" in info) ||
    !("height" in info) ||
    typeof info.width !== "number" ||
    typeof info.height !== "number" ||
    !Number.isInteger(info.width) ||
    !Number.isInteger(info.height) ||
    info.width <= 0 ||
    info.height <= 0 ||
    info.width * info.height > 16_777_216 ||
    message.data.length !== info.width * info.height
  )
    throw new Error("Invalid occupancy grid dimensions");
  canvas.width = info.width;
  canvas.height = info.height;
  const context = canvas.getContext("2d");
  if (!context) throw new Error("Map display requires Canvas 2D");
  const image = context.createImageData(info.width, info.height);
  for (let y = 0; y < info.height; y++) {
    for (let x = 0; x < info.width; x++) {
      const occupancy: unknown = message.data[y * info.width + x];
      if (
        typeof occupancy !== "number" ||
        !Number.isInteger(occupancy) ||
        occupancy < -1 ||
        occupancy > 100
      )
        throw new Error("Invalid occupancy probability");
      const color =
        occupancy === -1 ? 110 : Math.round(255 * (1 - occupancy / 100));
      const offset = ((info.height - 1 - y) * info.width + x) * 4;
      image.data.set([color, color, color, 255], offset);
    }
  }
  context.putImageData(image, 0, 0);
}
