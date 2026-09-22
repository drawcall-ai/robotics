"""Color segmentation and calibrated wrist-camera ray projection."""

import math

import cv2
import numpy as np

from kinematics import CAMERA, fk


def color_masks(rgb):
    """Pixels of each painted object, by its scene color."""
    a = rgb.astype(float) / 255
    r, g, b = a[:, :, 0], a[:, :, 1], a[:, :, 2]
    return {
        "red": (r > 0.30) & (r > g * 1.7) & (r > b * 1.6),
        "green": (g > 0.25) & (g > r * 1.35) & (g > b * 1.25),
        "magenta": (r > 0.30) & (b > 0.25) & (r > g * 1.6) & (b > g * 1.5),
        "bowl": (b > 0.25) & (b > r * 1.6) & (b > g * 1.25),
    }


def detect(rgb, q, angles=None):
    """Intersect color pixel rays with known tabletop/object-height planes."""
    a = rgb.astype(float) / 255
    masks = color_masks(rgb)
    camera = fk(q) @ CAMERA
    result = {}
    for name, mask in masks.items():
        if name != "bowl":
            count, labels, stats, _ = cv2.connectedComponentsWithStats(
                mask.astype(np.uint8), 8
            )
            if count < 2:
                continue
            index = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
            mask = labels == index
        if name != "bowl":
            brightness = np.max(a, axis=2)
            mask &= brightness >= np.percentile(brightness[mask], 90) * 0.94
        yy, xx = np.where(mask)
        if len(xx) < 25:
            continue
        # Housing +Z is forward; image right is housing -X.
        f = rgb.shape[0] / (2 * math.tan(math.radians(35)))
        rays = (
            np.stack(
                [
                    -(xx - rgb.shape[1] / 2) / f,
                    -(yy - rgb.shape[0] / 2) / f,
                    np.ones(len(xx)),
                ],
                axis=1,
            )
            @ camera[:3, :3].T
        )
        height = 0.020 if name == "bowl" else 0.019
        scale = (height - camera[2, 3]) / rays[:, 2]
        points = camera[:3, 3] + rays * scale[:, None]
        valid = (
            (scale > 0) & (np.abs(points[:, 0]) < 0.5) & (np.abs(points[:, 1]) < 0.5)
        )
        if np.count_nonzero(valid) < 25:
            continue
        xy = points[valid, :2]
        center = (
            (np.percentile(xy, 5, axis=0) + np.percentile(xy, 95, axis=0)) / 2
            if name == "bowl"
            else np.median(xy, axis=0)
        )
        if name != "bowl":
            rectangle = cv2.minAreaRect(xy.astype(np.float32))
            center = np.array(rectangle[0])
            if angles is not None:
                angles[name] = math.radians(rectangle[2])
        result[name] = center
    return result
