"""Chess pieces in an image, from a pretrained YOLOv8 detector exported to ONNX.

The model is not part of the example: set SO101_DETECTOR to its file, by default
models/pieces.onnx next to this one. It takes a 736x960 letterboxed RGB image and scores
twelve classes, a colour and a kind each; the colour is ignored here.
"""

import os
from pathlib import Path

import cv2
import numpy as np

KINDS = ["bishop", "king", "knight", "pawn", "queen", "rook"]
SIZE = (736, 960)
MODEL = Path(os.environ.get("SO101_DETECTOR", Path(__file__).parent / "models" / "pieces.onnx"))


class Detection:
    def __init__(self, scores, box):
        # Each kind's score, the better of its two colours.
        self.scores = scores
        self.kind = KINDS[int(np.argmax(scores))]
        self.score = float(np.max(scores))
        # Left, top, right, bottom in image pixels.
        self.box = np.asarray(box, float)

    @property
    def center(self):
        return (self.box[:2] + self.box[2:]) / 2


class Detector:
    def __init__(self, path=MODEL):
        self.net = cv2.dnn.readNetFromONNX(str(path))
        # As many threads as OMP_NUM_THREADS allows, else OpenCV's default.
        if "OMP_NUM_THREADS" in os.environ:
            cv2.setNumThreads(int(os.environ["OMP_NUM_THREADS"]))

    def __call__(self, rgb, confidence=0.4, overlap=0.5):
        """Pieces in an RGB image, best first."""
        h, w = rgb.shape[:2]
        scale = min(SIZE[0] / h, SIZE[1] / w)
        size = (round(w * scale), round(h * scale))
        top, left = (SIZE[0] - size[1]) // 2, (SIZE[1] - size[0]) // 2
        padded = np.full((*SIZE, 3), 114, np.uint8)
        padded[top : top + size[1], left : left + size[0]] = cv2.resize(rgb, size)
        self.net.setInput(cv2.dnn.blobFromImage(padded, 1 / 255))
        rows = self.net.forward()[0].T
        kinds = np.maximum(rows[:, 4:10], rows[:, 10:16])
        scores = kinds.max(1)
        rows, kinds, scores = rows[scores >= confidence], kinds[scores >= confidence], scores[scores >= confidence]
        boxes = np.c_[rows[:, :2] - rows[:, 2:4] / 2, rows[:, 2:4]]
        keep = np.array(cv2.dnn.NMSBoxes(boxes.tolist(), scores.tolist(), confidence, overlap), int).reshape(-1)
        offset = np.array([left, top, 0, 0])
        return [Detection(kinds[i], np.r_[(boxes[i] - offset)[:2], (boxes[i] - offset)[:2] + boxes[i, 2:]] / scale) for i in keep]
