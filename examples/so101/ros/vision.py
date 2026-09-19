"""Color segmentation and calibrated wrist-camera ray projection."""

import math

import cv2
import numpy as np

from kinematics import CAMERA, fk


def detect(rgb, q, angles=None):
    """Intersect color pixel rays with known tabletop/object-height planes."""
    a = rgb.astype(float) / 255
    r, g, b = a[:, :, 0], a[:, :, 1], a[:, :, 2]
    masks = {
        "red": (r > 0.30) & (r > g * 1.7) & (r > b * 1.6),
        "green": (g > 0.25) & (g > r * 1.35) & (g > b * 1.25),
        "magenta": (r > 0.30) & (b > 0.25) & (r > g * 1.6) & (b > g * 1.5),
        "bowl": (b > 0.25) & (b > r * 1.6) & (b > g * 1.25),
    }
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


def held_color(rgb, color):
    """A retained cube stays large beside the fingertips after the arm lifts.

    These image bounds are fixed by the wrist-camera/pad hardware transform.
    A tabletop cube recedes below this region during a 9cm lift.
    """
    a = rgb.astype(float) / 255
    r, g, b = a[:, :, 0], a[:, :, 1], a[:, :, 2]
    masks = {
        "red": (r > 0.30) & (r > g * 1.7) & (r > b * 1.6),
        "green": (g > 0.25) & (g > r * 1.35) & (g > b * 1.25),
        "magenta": (r > 0.30) & (b > 0.25) & (r > g * 1.6) & (b > g * 1.5),
    }
    mask = masks[color]
    ys, xs = np.where(mask)
    if len(xs) < 700:
        return False
    return np.count_nonzero(mask[20:190, 240:430]) > 700 and np.median(ys) < 195


def cube_in_gripper(rgb, color, with_normal=False):
    """Recover a visible 22mm top face using rigid wrist-camera calibration.

    No arm FK or assumed table height enters this estimate. Reject occluded
    faces instead of inventing a square from a partial colored region.
    """
    a = rgb.astype(float) / 255
    r, g, b = a[:, :, 0], a[:, :, 1], a[:, :, 2]
    masks = {
        "red": (r > 0.30) & (r > g * 1.7) & (r > b * 1.6),
        "green": (g > 0.25) & (g > r * 1.35) & (g > b * 1.25),
        "magenta": (r > 0.30) & (b > 0.25) & (r > g * 1.6) & (b > g * 1.5),
    }
    mask = masks[color]
    if np.count_nonzero(mask) < 100:
        return None
    brightness = a.max(axis=2)
    mask &= brightness >= np.percentile(brightness[mask], 90) * 0.94
    contours, _ = cv2.findContours(
        mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    hull = cv2.convexHull(contour)
    corners = cv2.approxPolyDP(hull, 0.025 * cv2.arcLength(hull, True), True)
    if len(corners) != 4 or cv2.contourArea(corners) < 100:
        return None
    pixels = corners[:, 0].astype(np.float64)
    center = pixels.mean(axis=0)
    pixels = pixels[
        np.argsort(np.arctan2(pixels[:, 1] - center[1], pixels[:, 0] - center[0]))
    ]
    square = np.array(
        [[-0.011, -0.011, 0], [0.011, -0.011, 0], [0.011, 0.011, 0], [-0.011, 0.011, 0]]
    )
    f = rgb.shape[0] / (2 * math.tan(math.radians(35)))
    intrinsic = np.array(
        [[f, 0, rgb.shape[1] / 2], [0, f, rgb.shape[0] / 2], [0, 0, 1.0]]
    )
    ok, rotations, translations, errors = cv2.solvePnPGeneric(
        square, pixels, intrinsic, None, flags=cv2.SOLVEPNP_IPPE
    )
    if not ok:
        return None
    index = int(np.argmin(errors))
    if float(errors[index, 0]) > 2.0:
        return None
    optical = translations[index][:, 0]
    if optical[2] <= 0:
        return None
    center = CAMERA[:3, 3] + CAMERA[:3, :3] @ (optical * [-1, -1, 1])
    rotation, _ = cv2.Rodrigues(rotations[index])
    normal = CAMERA[:3, :3] @ (rotation[:, 2] * [-1, -1, 1])
    if normal[2] < 0:
        normal = -normal
    return (center, normal) if with_normal else center
