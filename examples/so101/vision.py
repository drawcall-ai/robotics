"""Calibrated machine vision for the chess task: the board, and where pieces stand.

Everything is measured in the robot's base frame. The wrist camera's intrinsics are its
calibration, its pose comes from the joint positions, and the robot stands on the desk,
so the desk plane is known from the robot description. Nothing here is told the board's
size, thickness, colours or place:

- the board top is the plane on which several views of it agree;
- the median of all views, resampled onto that plane, is the empty board, because every
  view sees a piece in a different place except where it stands;
- the board is the 8x8 checker pattern in that image;
- a piece is where the rays through its detections in several views meet.
"""

import math

import cv2
import numpy as np

# The wrist camera's calibration: 640x480 pixels, 70 degrees vertical field of view.
FOV = math.radians(70)
FILES = "abcdefgh"
# Plane images: half-millimetre pixels over everything the arm reaches.
RESOLUTION = 0.0005
AREA = ((0.03, 0.34), (-0.29, 0.29))


class Grid:
    """Pixels on a horizontal plane: row r, column c is x = x0 + r*res, y = y0 + c*res."""

    def __init__(self, z, res=RESOLUTION, area=AREA):
        (self.x0, x1), (self.y0, y1) = area
        self.z = z
        self.res = res
        self.shape = (int((x1 - self.x0) / res), int((y1 - self.y0) / res))

    def point(self, r, c):
        return np.stack([self.x0 + np.asarray(r) * self.res, self.y0 + np.asarray(c) * self.res], -1)

    def pixel(self, xy):
        """Row and column of a point."""
        xy = np.asarray(xy, float)
        return (xy[..., 0] - self.x0) / self.res, (xy[..., 1] - self.y0) / self.res


class View:
    """One settled wrist-camera image, the camera's pose, and the pixels the gripper covers."""

    def __init__(self, arm, q, rgb):
        self.rgb = rgb.astype(np.float32) / 255
        h, w = rgb.shape[:2]
        self.f = h / (2 * math.tan(FOV / 2))
        self.K = np.array([[self.f, 0, w / 2], [0, self.f, h / 2], [0, 0, 1]])
        self.pose = arm.fk(q) @ arm.camera
        self.center = self.pose[:3, 3]
        own = np.zeros((h, w), np.uint8)
        for triangles in arm.body_surfaces(q):
            local = (triangles - self.center) @ self.pose[:3, :3]
            local = local[np.all(local[..., 2] > 0.005, axis=1)]
            uv = self.f * local[..., :2] / local[..., 2:] + (w / 2, h / 2)
            cv2.fillPoly(own, np.round(uv * 16).astype(np.int32), 1, cv2.LINE_8, 4)
        self.own = cv2.dilate(
            cv2.morphologyEx(own, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8)), np.ones((5, 5), np.uint8)
        ).astype(bool)

    def project(self, points):
        """Pixel coordinates and depths of base-frame points."""
        local = (np.asarray(points, float) - self.center) @ self.pose[:3, :3]
        return self.f * local[..., :2] / local[..., 2:3] + self.K[:2, 2], local[..., 2]

    def ray(self, uv):
        """Unit direction of the ray through pixels, in the base frame."""
        uv = np.asarray(uv, float)
        local = np.stack([(uv[..., 0] - self.K[0, 2]) / self.f, (uv[..., 1] - self.K[1, 2]) / self.f, np.ones(uv.shape[:-1])], -1)
        direction = local @ self.pose[:3, :3].T
        return direction / np.linalg.norm(direction, axis=-1, keepdims=True)

    def homography(self, grid):
        """Maps grid pixels (c, r) to image pixels."""
        plane = np.array([[0, grid.res, grid.x0], [grid.res, 0, grid.y0], [0, 0, grid.z]], float)
        plane[:, 2] -= self.center
        return self.K @ self.pose[:3, :3].T @ plane

    def ortho(self, grid, image=None):
        """The image resampled onto the grid's plane; NaN where the view does not see it."""
        image = self.rgb if image is None else image
        size = (grid.shape[1], grid.shape[0])
        H = self.homography(grid)
        warped = cv2.warpPerspective(image, H, size, flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
        seen = cv2.warpPerspective(
            (~self.own).astype(np.uint8), H, size, flags=cv2.INTER_NEAREST | cv2.WARP_INVERSE_MAP
        ).astype(bool)
        warped = warped.reshape(grid.shape + (-1,))
        warped[~seen] = np.nan
        return warped

    def unortho(self, grid, image):
        """A grid image rendered into this view; NaN where the grid does not reach."""
        h, w = self.rgb.shape[:2]
        filled = np.nan_to_num(image, nan=-1.0).astype(np.float32)
        out = cv2.warpPerspective(filled, self.homography(grid), (w, h), borderValue=-1.0).reshape(h, w, -1)
        out[np.any(out < -0.01, axis=2)] = np.nan
        return out


def gray(image):
    return image @ np.array([0.299, 0.587, 0.114], np.float32)


def gradient(image):
    """Gradients of a grey image with NaN holes, zero next to the holes."""
    seen = np.isfinite(image)
    filled = np.where(seen, image, np.nanmean(image) if seen.any() else 0).astype(np.float32)
    gx = cv2.Sobel(filled, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(filled, cv2.CV_32F, 0, 1, ksize=3)
    inside = cv2.erode(seen.astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool)
    return gx * inside, gy * inside


def agreement(views, z, res):
    """How well the views' edges line up on the plane at height z."""
    images = []
    for view in views:
        gx, gy = gradient(gray(view.ortho(Grid(z, res))))
        images.append(cv2.GaussianBlur(np.hypot(gx, gy), (0, 0), 1.0))
    scores = []
    for i in range(len(images)):
        for j in range(i + 1, len(images)):
            a, b = images[i], images[j]
            both = (a > 0) & (b > 0)
            if both.sum() < 2000:
                continue
            a, b = a[both] - a[both].mean(), b[both] - b[both].mean()
            scores.append(float(a @ b / math.sqrt((a @ a) * (b @ b) + 1e-12)))
    return float(np.mean(scores)) if scores else 0.0


def plane_height(views, low, high):
    """Height between low and high of the plane on which all views see the same pattern."""
    coarse = np.arange(low, high, 0.001)
    best = coarse[int(np.argmax([agreement(views, z, 0.001) for z in coarse]))]
    fine = np.arange(best - 0.0015, best + 0.0016, 0.00025)
    return float(fine[int(np.argmax([agreement(views, z, RESOLUTION) for z in fine]))])


class Board:
    """A chessboard in the base frame: square size, top height, centre and axes.

    Rank 8 is the edge nearest the robot and the a-file is on its right, as black sees the
    board. `u` runs along the files a to h, `v` from rank 8 towards rank 1.
    """

    def __init__(self, s, z, origin, u, v):
        self.s = s
        self.z = z
        self.origin = np.asarray(origin, float)
        self.u, self.v = np.asarray(u, float), np.asarray(v, float)

    @property
    def yaw(self):
        """Heading of the ranks, from rank 8 towards rank 1."""
        return math.atan2(self.v[1], self.v[0])

    def point(self, u, v):
        return self.origin + np.multiply.outer(u, self.u) + np.multiply.outer(v, self.v)

    def center(self, square):
        return self.point((FILES.index(square[0]) - 3.5) * self.s, (4.5 - int(square[1])) * self.s)

    def coordinates(self, xy):
        d = np.asarray(xy, float) - self.origin
        return d @ self.u, d @ self.v

    def square(self, xy, margin=0.0):
        """The square holding xy at least `margin` in from its edges, or None."""
        u, v = self.coordinates(xy)
        file, index = math.floor(u / self.s + 4), math.floor(v / self.s + 4)
        if not (0 <= file < 8 and 0 <= index < 8):
            return None
        du, dv = u - (file - 3.5) * self.s, v - (index - 3.5) * self.s
        if max(abs(du), abs(dv)) > self.s / 2 - margin:
            return None
        return f"{FILES[file]}{8 - index}"

    @staticmethod
    def squares():
        return [f"{f}{r}" for f in FILES for r in range(1, 9)]


def comb(profile, period, lines=9):
    """Best contrast of `lines` equally spaced samples of a profile against the samples
    midway between them, and where the first one lies."""
    positions = np.arange(0, len(profile) - (lines - 1) * period - 1, 0.25)
    if len(positions) == 0:
        return -1.0, 0.0
    index = np.arange(len(profile))
    on = sum(np.interp(positions + k * period, index, profile) for k in range(lines))
    off = sum(np.interp(positions + (k + 0.5) * period, index, profile) for k in range(lines - 1))
    total = on - off * lines / (lines - 1)
    best = int(np.argmax(total))
    return float(total[best]), float(positions[best])


def fit_board(image, grid):
    """The 8x8 checker pattern in a plane image: its two line families, then its corners."""
    g = gray(image)
    gx, gy = gradient(g)
    weight = gx * gx + gy * gy
    angle = np.arctan2(gy, gx)
    theta = 0.25 * math.atan2(float((weight * np.sin(4 * angle)).sum()), float((weight * np.cos(4 * angle)).sum()))
    rows, cols = np.indices(g.shape)
    normals = [np.array([math.cos(theta), math.sin(theta)]), np.array([-math.sin(theta), math.cos(theta)])]
    # Each line family's edge strength projected onto its normal, in half-pixel bins.
    profiles = []
    for n in normals:
        a = cols * n[0] + rows * n[1]
        low = a.min()
        bins = np.round((a - low) * 2).astype(int).ravel()
        profile = np.bincount(bins, np.abs(gx * n[0] + gy * n[1]).ravel()) / 2
        profiles.append((cv2.GaussianBlur(profile.reshape(1, -1), (0, 0), 1.0).ravel(), low))
    best = None
    for period in np.arange(0.017, 0.04, 0.0001) / grid.res * 2:
        (sa, pa), (sb, pb) = (comb(p, period) for p, _ in profiles)
        if best is None or sa + sb > best[0]:
            best = (sa + sb, period, pa, pb)
    _, period, pa, pb = best
    step = period / 2
    starts = (pa / 2 + profiles[0][1], pb / 2 + profiles[1][1])
    guesses = [(i, j, (starts[0] + i * step) * normals[0] + (starts[1] + j * step) * normals[1]) for i in range(1, 8) for j in range(1, 8)]
    filled = np.where(np.isfinite(g), g, 0).astype(np.float32)
    refined = cv2.cornerSubPix(
        filled, np.array([p for _, _, p in guesses], np.float32).reshape(-1, 1, 2), (5, 5), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01),
    ).reshape(-1, 2)
    # Keep corners that moved little and show a checker saddle; pieces hide the others.
    r = int(max(3, step * 0.3))
    keep = []
    for (i, j, guess), point in zip(guesses, refined):
        x, y = int(round(point[0])), int(round(point[1]))
        if np.linalg.norm(point - guess) > 0.25 * step or not (r <= y < g.shape[0] - r and r <= x < g.shape[1] - r):
            continue
        quadrants = [g[y - r : y - 1, x - r : x - 1], g[y - r : y - 1, x + 2 : x + r], g[y + 2 : y + r, x + 2 : x + r], g[y + 2 : y + r, x - r : x - 1]]
        m = [np.nanmedian(q) if np.isfinite(q).any() else np.nan for q in quadrants]
        if not np.all(np.isfinite(m)) or min(abs(m[k] - m[(k + 1) % 4]) for k in range(4)) < 0.05:
            continue
        if (m[0] - m[1]) * (m[2] - m[1]) < 0 or (m[0] - m[3]) * (m[2] - m[3]) < 0:
            continue
        keep.append((i, j, point))
    if len(keep) < 12:
        raise ValueError(f"Only {len(keep)} board corners visible")
    # Least-squares similarity from line indices to base-frame points. Grid pixels are
    # mirrored against the base frame, so the indices may run either way round.
    xy = np.array([grid.point(p[1], p[0]) for _, _, p in keep]).reshape(-1)
    fits = []
    for ij in (np.array([(i - 4, j - 4) for i, j, _ in keep], float), np.array([(j - 4, i - 4) for i, j, _ in keep], float)):
        A = np.zeros((len(xy), 4))
        A[0::2] = np.c_[ij[:, 0], -ij[:, 1], np.ones(len(ij)), np.zeros(len(ij))]
        A[1::2] = np.c_[ij[:, 1], ij[:, 0], np.zeros(len(ij)), np.ones(len(ij))]
        solution, *_ = np.linalg.lstsq(A, xy, rcond=None)
        fits.append((np.linalg.norm(A @ solution - xy) / math.sqrt(len(keep)), solution))
    _, (a, b, ox, oy) = min(fits, key=lambda fit: fit[0])
    size = math.hypot(a, b)
    e = np.array([a, b]) / size
    # Ranks run away from the robot, files to its left.
    v = max((e, -e, np.array([-e[1], e[0]]), np.array([e[1], -e[0]])), key=lambda d: d[0])
    return Board(size, grid.z, (ox, oy), (-v[1], v[0]), v), len(keep)


class Survey:
    """The board, from several high views."""

    def __init__(self, views, floor):
        self.floor = floor
        self.top = Grid(plane_height(views, floor + 0.003, floor + 0.04))
        with np.errstate(all="ignore"):
            self.plate = np.nanmedian(np.stack([v.ortho(self.top) for v in views]), axis=0)
        self.board, self.corners = fit_board(self.plate, self.top)
        # The board turned so that the a8 corner is light, as every board is set up.
        b = self.board
        u, v = b.coordinates(self.top.point(*np.indices(self.top.shape)))
        file, index = np.floor(u / b.s + 4), np.floor(v / b.s + 4)
        central = (np.maximum(np.abs(u), np.abs(v)) < 4 * b.s) & (
            np.hypot(u - (file - 3.5) * b.s, v - (index - 3.5) * b.s) < 0.25 * b.s
        )
        light = (file + 1 + (8 - index)) % 2 == 1
        seen = np.all(np.isfinite(self.plate), axis=2)
        if gray(np.median(self.plate[central & light & seen], axis=0)) < gray(np.median(self.plate[central & ~light & seen], axis=0)):
            raise ValueError("The a8 corner is dark: the board is turned the wrong way")

    def support(self, xy):
        """Height of what a piece at xy stands on: the board, or the desk beside it."""
        u, v = self.board.coordinates(xy)
        return self.board.z if max(abs(u), abs(v)) < 4 * self.board.s else self.floor


# A piece's head tops its box in views flatter than this. Past it, the far rim of the base
# can top the box instead: FIDE sets pieces on bases 40-50% as wide as they are tall, so
# the head stays higher on screen while tan(elevation) < height / base radius, at least 4.
TOP_ELEVATION = math.radians(70)


# Views at most this steep look at a piece from the side, as a player sees the board.
SIDE_ELEVATION = math.radians(45)


class Sighting:
    """A detected piece in one view: its kind and the rays through its box centre and the
    middle of its box top."""

    def __init__(self, view, detection):
        self.kind = detection.kind
        self.score = detection.score
        self.origin = view.center
        self.ray = view.ray(detection.center)
        self.top = view.ray(np.array([detection.center[0], detection.box[1]]))


def sightings(view, detections):
    """The detections that are not on the robot itself."""
    h, w = view.own.shape
    found = []
    for d in detections:
        u, v = np.clip(np.round(d.center).astype(int), 0, (w - 1, h - 1))
        if not view.own[v, u]:
            found.append(Sighting(view, d))
    return found


def height(group, support, xy):
    """How tall a piece standing at xy is: where the rays through its box tops pass its
    axis, in views flat enough to show its head there; None without such a view."""
    flat = [s for s in group if math.asin(-s.top[2]) <= TOP_ELEVATION]
    if not flat:
        return None
    # The median: one box cut off or grown by a neighbour misleads a mean.
    return float(np.median([at_axis(s.origin, s.top, xy) for s in flat]) - support)


def vote(group, allowed=None):
    """The kind the sightings of one piece agree on, each counted by its confidence, among
    the `allowed` kinds if any of them was seen. Views from the side, like the photos of
    boards a detector learns from, decide if there are any; views from high above show
    little of what tells pieces apart."""
    side = [s for s in group if math.asin(-s.ray[2]) <= SIDE_ELEVATION]
    totals = {}
    for s in side or group:
        totals[s.kind] = totals.get(s.kind, 0.0) + s.score
    kept = {k: v for k, v in totals.items() if allowed is None or k in allowed}
    return max(kept or totals, key=(kept or totals).get)


def axis(group):
    """Where a standing piece's vertical axis is. Its box centre lies on the axis in every
    view, so each view's ray through it spans a vertical plane containing the axis; the
    planes meet there, least squares, steep views counting less. None unless the views
    look from different directions."""
    A = np.zeros((2, 2))
    b = np.zeros(2)
    for s in group:
        # The plane's normal: horizontal, across the ray, as long as the ray is flat.
        n = np.array([-s.ray[1], s.ray[0]])
        A += np.outer(n, n)
        b += n * (n @ s.origin[:2])
    if np.linalg.eigvalsh(A)[0] < 0.01 * np.trace(A):
        return None
    return np.linalg.solve(A, b)


def at_axis(origin, ray, xy):
    """The height at which a ray passes closest to the vertical line through xy."""
    flat = ray[:2] @ ray[:2]
    t = (np.asarray(xy) - origin[:2]) @ ray[:2] / flat if flat > 1e-9 else 0.0
    return origin[2] + t * ray[2]


def ground(sighting, z):
    """Where a sighting's ray crosses the plane at height z."""
    return sighting.origin + sighting.ray * (z - sighting.origin[2]) / sighting.ray[2]


def locate(sighting, survey):
    """Where a single sighting's ray passes a square above what it points at."""
    s = survey.board.s
    point = ground(sighting, survey.board.z + s)
    return ground(sighting, survey.support(point[:2]) + s)


def pieces(found, survey, spread):
    """Sightings of the same piece from different views, grouped by where each points,
    and each group's meeting point, most seen first."""
    groups = []
    for s in found:
        point = locate(s, survey)[:2]
        for group in groups:
            if np.linalg.norm(point - group[0]) < spread:
                group[1].append(s)
                break
        else:
            groups.append((point, [s]))
    groups.sort(key=lambda g: -len(g[1]))
    return [(position(g, survey), g) for _, g in groups]


def position(group, survey):
    """A piece's middle from its sightings: on its axis, at the height its box centres
    show there, if seen from several directions; else a square above where one points."""
    xy = axis(group)
    if xy is None:
        return locate(max(group, key=lambda s: s.score), survey)
    return np.r_[xy, np.mean([at_axis(s.origin, s.ray, xy) for s in group])]


def outline(view, mask, z):
    """An object's mask in a view as points on the plane at height z, leaving out the
    robot's own pixels."""
    rows, cols = np.nonzero(mask & ~view.own)
    rays = view.ray(np.stack([cols, rows], -1).astype(float))
    return (view.center + rays * ((z - view.center[2]) / rays[..., 2:3]))[..., :2]


class Shape:
    """A piece's outline seen from above, measured along its principal axes."""

    def __init__(self, points):
        self.center = points.mean(axis=0)
        values, vectors = np.linalg.eigh(np.cov((points - self.center).T))
        self.axis = vectors[:, 1]
        along = (points - self.center) @ self.axis
        across = (points - self.center) @ vectors[:, 0]
        self.length = float(np.ptp(along))
        self.width = float(np.ptp(across))
        # How wide each end is: the base is a piece's widest part.
        ends = [along < along.min() + 0.2 * self.length, along > along.max() - 0.2 * self.length]
        wide = [np.ptp(across[e]) if e.sum() > 1 else 0.0 for e in ends]
        # The axis points from the head to the base.
        if wide[0] > wide[1]:
            self.axis = -self.axis
        self.base = self.center + self.axis * self.length / 2
        self.base_width = max(wide)
        # Widths along the axis, from the base, in twentieths of the length.
        from_base = (self.base - points) @ self.axis
        bins = np.clip((from_base / self.length * 20).astype(int), 0, 19)
        self.widths = np.array([np.ptp(across[bins == i]) if (bins == i).sum() > 1 else 0.0 for i in range(20)])

    def thickest(self, low, high):
        """The share of its length from the base where the piece is widest between the
        shares `low` and `high`: its axis lies highest over the surface there."""
        widths = self.widths[int(low * 20) : int(high * 20)]
        return (int(low * 20) + int(np.argmax(widths)) + 0.5) / 20

    @property
    def lying(self):
        """Elongated seen from above: lying on its side. Standing pieces look round."""
        return self.length > 1.5 * self.width


def facing(view, mask, z):
    return foot_and_facing(view, mask, z)[1]


def foot_and_facing(view, mask, z):
    """Where a piece's base is and the way its head leans, seen from above, on the plane at
    height z: the base is the middle of the widest round part of its outline, and the head
    leans from there towards the outline's centroid, which the overhanging head pulls
    forward, as a unit vector. A knight looks that way."""
    inside = (mask & ~view.own).astype(np.uint8)
    distance = cv2.distanceTransform(inside, cv2.DIST_L2, 5)
    rows, cols = np.nonzero(inside)
    base = np.array(np.unravel_index(np.argmax(distance), distance.shape)[::-1], float)
    centroid = np.array([cols.mean(), rows.mean()])
    ends = view.center + view.ray(np.stack([base, centroid])) * ((z - view.center[2]) / view.ray(np.stack([base, centroid]))[:, 2:3])
    direction = ends[1, :2] - ends[0, :2]
    return ends[0, :2], direction / (np.linalg.norm(direction) + 1e-12)


def piece_shapes(view, masks, survey, length, width):
    """The pieces in a view, from class-agnostic object masks: outlines on the surface they
    rest on that are piece-sized, `length` and `width` ranges in squares, with their masks,
    whole pieces rather than their parts.
    Board squares are too wide, the board's frame too long, the robot left out."""
    s = survey.board.s
    h, w = view.own.shape
    found, taken = [], []
    # Largest first: of a whole piece and its parts, such as its head, the whole is kept.
    for mask in sorted(masks, key=lambda m: -m.sum()):
        rows, cols = np.nonzero(mask)
        if len(rows) < 20 or (mask & view.own).sum() > 0.1 * len(rows):
            continue
        if rows.min() == 0 or cols.min() == 0 or rows.max() == h - 1 or cols.max() == w - 1:
            continue
        ray = view.ray(np.array([cols.mean(), rows.mean()]))
        point = view.center + ray * (survey.board.z - view.center[2]) / ray[2]
        support = survey.support(point[:2])
        shape = Shape(outline(view, mask, support + 0.005))
        shape = Shape(outline(view, mask, support + shape.width / 2))
        if length[0] * s < shape.length < length[1] * s and width[0] * s < shape.width < width[1] * s:
            if any((mask & t).sum() > 0.5 * len(rows) for t in taken):
                continue
            shape.foot, shape.facing = foot_and_facing(view, mask, support + shape.width / 2)
            found.append(shape)
            taken.append(mask)
    return found
