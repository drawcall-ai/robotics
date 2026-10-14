// What a browser gives Three.js that Node lacks, so scenes load files as they would on a page:
// `self`, `fetch` of the `file:` URLs a scene resolves against `import.meta.url`, the progress
// events loaders report, and image decoding.
import { readFile } from "node:fs/promises";

/** A decoded image: RGBA pixels, top row first, not premultiplied. */
export class DecodedImage {
  constructor(
    readonly width: number,
    readonly height: number,
    readonly data: Uint8Array<ArrayBuffer>,
  ) {}
  close(): void {}
}

/** The progress of a load, as `FileLoader` reports it while a response streams. */
class ProgressEvent extends Event {
  readonly lengthComputable: boolean;
  readonly loaded: number;
  readonly total: number;
  constructor(type: string, init: ProgressEventInit = {}) {
    super(type, init);
    this.lengthComputable = init.lengthComputable ?? false;
    this.loaded = init.loaded ?? 0;
    this.total = init.total ?? 0;
  }
}

const fetch = globalThis.fetch;
globalThis.fetch = async (input, init) => {
  const url = input instanceof Request ? input.url : String(input);
  if (!url.startsWith("file:")) return fetch(input, init);
  return new Response(await readFile(new URL(url)));
};

Object.assign(globalThis, {
  self: globalThis,
  ProgressEvent,
  ImageBitmap: DecodedImage,
  createImageBitmap: async (
    source: unknown,
    options: ImageBitmapOptions = {},
  ): Promise<DecodedImage> => {
    if (!(source instanceof Blob))
      throw new Error("createImageBitmap takes only a Blob here");
    if (options.premultiplyAlpha === "premultiply")
      throw new Error("createImageBitmap does not premultiply here");
    if (options.resizeWidth || options.resizeHeight)
      throw new Error("createImageBitmap does not resize here");
    // sharp loads on the first image, not with every command.
    const { default: sharp } = await import("sharp");
    let image = sharp(Buffer.from(await source.arrayBuffer())).ensureAlpha();
    if (options.imageOrientation === "flipY") image = image.flip();
    const { data, info } = await image
      .raw()
      .toBuffer({ resolveWithObject: true });
    return new DecodedImage(info.width, info.height, new Uint8Array(data));
  },
});
