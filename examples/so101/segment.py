"""The object at a point in an image, from a pretrained promptable segmentation model.

MobileSAM (Apache-2.0), exported to ONNX as an image encoder and a single-mask decoder, as
Acly/MobileSAM on Hugging Face publishes them. Set SO101_SEGMENTER to the folder holding
them, by default models/ next to this file.
"""

import os
from pathlib import Path

import cv2
import numpy as np
import onnxruntime

FOLDER = Path(os.environ.get("SO101_SEGMENTER", Path(__file__).parent / "models"))
SIZE = 1024


class Segmenter:
    def __init__(self, folder=FOLDER):
        # As many threads as OMP_NUM_THREADS allows, else one per core.
        options = onnxruntime.SessionOptions()
        options.intra_op_num_threads = int(os.environ.get("OMP_NUM_THREADS", 0))
        self.encoder = onnxruntime.InferenceSession(str(folder / "mobile_sam_image_encoder.onnx"), options)
        self.decoder = onnxruntime.InferenceSession(str(folder / "sam_mask_decoder_single.onnx"), options)

    def __call__(self, rgb, point):
        """The mask of the object at `point`, in pixels of the RGB image."""
        return self.masks(rgb, [point])[0]

    def everything(self, rgb, spacing, skip):
        """Masks of the objects under a grid of points `spacing` pixels apart, leaving out
        points where `skip` is set and points an earlier mask already covers."""
        h, w = rgb.shape[:2]
        embedding = self.embed(rgb)
        covered = skip.copy()
        found = []
        for y in range(spacing // 2, h, spacing):
            for x in range(spacing // 2, w, spacing):
                if covered[y, x]:
                    continue
                mask = self.decode(embedding, (x, y), (h, w))
                covered |= mask
                found.append(mask)
        return found

    def masks(self, rgb, points):
        embedding = self.embed(rgb)
        return [self.decode(embedding, p, rgb.shape[:2]) for p in points]

    def embed(self, rgb):
        h, w = rgb.shape[:2]
        scale = SIZE / max(h, w)
        resized = cv2.resize(rgb, (round(w * scale), round(h * scale))).astype(np.float32)
        (embedding,) = self.encoder.run(None, {"input_image": resized})
        return embedding

    def decode(self, embedding, point, shape):
        h, w = shape
        scale = SIZE / max(h, w)
        # One point on the object, and the padding point the decoder expects without a box.
        coords = np.array([[np.asarray(point, float) * scale, [0.0, 0.0]]], np.float32)
        labels = np.array([[1, -1]], np.float32)
        masks, _, _ = self.decoder.run(None, {
            "image_embeddings": embedding,
            "point_coords": coords,
            "point_labels": labels,
            "mask_input": np.zeros((1, 1, 256, 256), np.float32),
            "has_mask_input": np.zeros(1, np.float32),
            "orig_im_size": np.array([h, w], np.float32),
        })
        return masks[0, 0] > 0
