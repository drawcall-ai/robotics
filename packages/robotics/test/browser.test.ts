import { mkdtemp, rm, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { pathToFileURL } from "node:url";
import sharp from "sharp";
import { FileLoader, ImageBitmapLoader } from "three";
import { expect, it } from "vitest";
import { DecodedImage } from "../src/cli/browser.js";

function pixels(image: unknown): number[] {
  if (!(image instanceof DecodedImage)) throw new Error("Not a decoded image");
  return [...image.data];
}

it("loads files and decodes images from file: URLs with Three.js's own loaders", async () => {
  const directory = await mkdtemp(join(tmpdir(), "browser-"));
  try {
    await writeFile(join(directory, "note.txt"), "hello");
    // One red pixel above one blue one.
    const raw = Buffer.from([255, 0, 0, 255, 0, 0, 255, 255]);
    await sharp(raw, { raw: { width: 1, height: 2, channels: 4 } })
      .png()
      .toFile(join(directory, "image.png"));
    const url = (name: string) => pathToFileURL(join(directory, name)).href;

    expect(await new FileLoader().loadAsync(url("note.txt"))).toBe("hello");
    const image = await new ImageBitmapLoader().loadAsync(url("image.png"));
    expect(pixels(image)).toEqual([...raw]);
    const flipped = await new ImageBitmapLoader()
      .setOptions({ imageOrientation: "flipY" })
      .loadAsync(url("image.png"));
    expect(pixels(flipped)).toEqual([0, 0, 255, 255, 255, 0, 0, 255]);
  } finally {
    await rm(directory, { recursive: true });
  }
});
