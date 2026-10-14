// The world every step of the chess task shares: a desk, lighting, the SO101 and a chessboard
// within its reach, all drawn at random from a seed, plus pieces and the checks the steps'
// evaluators build on. Each step's .glts scene builds it and adds its own pieces and evaluator.
//
// Surfaces use Drawcall Market wood textures and the lighting a market HDR environment, with
// a shadow-casting sun, so the wrist camera sees something close to a real desk.
import { BoxCollider, RigidBody } from "@drawcall/physics";
import { findRobot } from "@drawcall/robotics";
import {
  BoxGeometry,
  Color,
  DataTexture,
  DirectionalLight,
  HemisphereLight,
  LinearMipmapLinearFilter,
  Mesh,
  MeshStandardMaterial,
  type Object3D,
  PlaneGeometry,
  Quaternion,
  Scene,
  SRGBColorSpace,
  Vector3,
} from "three";
import { environmentMap, material } from "./assets.ts";
import { piece, pieceHeight, pieceMaterial, type Kind } from "./pieces.ts";
import {
  boardCoordinates,
  deskHeight,
  reachable,
  ros,
  seeded,
  setup,
  squareAt,
  tilt,
  world,
  type Board,
} from "./task.ts";

/** What the viewer shows beside the robot: the controller's annotated wrist image. */
export const show = ["/robot/perception/image/compressed"];

export interface Piece {
  body: RigidBody;
  kind: Kind;
}

/**
 * Builds the chess desk into `scene` with `arm`, the loaded robot, on it: lighting, desk and a
 * board, drawn at random from the episode's seed. Returns what the steps' evaluators build on.
 */
export async function environment(scene: Scene, arm: Object3D) {
  // Every run draws a new world; SEED=<n> replays one.
  const seed = Number(process.env.SEED ?? Math.floor(Math.random() * 2 ** 31));
  const random = seeded(seed);
  const between = (low: number, high: number) => low + random() * (high - low);
  const pick = <T>(options: readonly T[]): T =>
    options[Math.floor(random() * options.length)] as T;
  /** A palette colour, varied a little in hue, saturation and lightness. */
  function vary(hex: string, amount = 0.04): Color {
    return new Color(hex).offsetHSL(
      between(-amount, amount) / 2,
      between(-amount, amount),
      between(-amount, amount),
    );
  }
  const woods = [await material("wood-01"), await material("wood-02")];
  /** A wood texture set repeated `times` across a surface, tinted; the images are shared. */
  function wood(index: number, times: number, tint: Color, roughness: number) {
    const maps = woods[index]!;
    const repeated = Object.fromEntries(
      Object.entries(maps).map(([key, map]) => {
        const copy = map.clone();
        copy.repeat.set(times, times);
        copy.needsUpdate = true;
        return [key, copy];
      }),
    ) as typeof maps;
    return new MeshStandardMaterial({ ...repeated, color: tint, roughness });
  }

  scene.background = vary(
    pick(["#dce4e7", "#e8e4dc", "#cfd6db", "#b8bec4"]),
    0.06,
  );
  scene.environment = await environmentMap(
    "public/environment/stark-white-brutalist-digital-sky-high-contrast-minimalist-terminal-aesthetic.hdr",
  );
  scene.environmentIntensity = between(0.35, 0.9);
  scene.add(new HemisphereLight("#ffffff", "#687078", between(0.3, 1)));
  const sun = new DirectionalLight(vary("#fff5e8", 0.05), between(1.2, 3));
  sun.position
    .set(between(-1, 1), between(1.2, 2), between(-1, 1.2))
    .add(new Vector3(0, deskHeight, -0.2));
  sun.target.position.set(0, deskHeight, -0.2);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  sun.shadow.camera.left = sun.shadow.camera.bottom = -0.45;
  sun.shadow.camera.right = sun.shadow.camera.top = 0.45;
  sun.shadow.camera.near = 0.5;
  sun.shadow.camera.far = 4;
  sun.shadow.bias = -0.0003;
  sun.shadow.normalBias = 0.002;
  scene.add(sun, sun.target);

  // A wooden desk, light or dark, or a plain laminate one.
  const deskMaterial =
    random() < 0.7
      ? wood(
          Math.floor(random() * 2),
          between(1.5, 3),
          vary(pick(["#ffffff", "#d9c7b0", "#b89878"]), 0.05),
          between(0.5, 0.9),
        )
      : new MeshStandardMaterial({
          color: vary(pick(["#d8d8d4", "#8a8f94", "#e9e4da", "#5b5f63"])),
          roughness: between(0.4, 0.9),
        });
  const desk = new RigidBody({ bodyType: "static" });
  desk.name = "desk";
  desk.position.set(0, deskHeight - 0.025, -0.18);
  desk.setMaterial({
    staticFriction: 0.6,
    dynamicFriction: 0.6,
    restitution: 0,
  });
  const deskTop = new Mesh(new BoxGeometry(0.8, 0.05, 0.7), deskMaterial);
  deskTop.receiveShadow = true;
  desk.add(new BoxCollider({ size: [0.8, 0.05, 0.7] }), deskTop);
  for (const x of [-0.34, 0.34])
    for (const z of [-0.29, 0.29]) {
      const leg = new Mesh(
        new BoxGeometry(0.045, deskHeight - 0.05, 0.045),
        deskMaterial,
      );
      leg.position.set(x, -deskHeight / 2, z);
      desk.add(leg);
    }
  // The robot is an asset of its own, here on the desk; any arm its controller can drive fits.
  arm.position.y = deskHeight;
  scene.add(desk, arm);
  const robot = findRobot(scene);

  // A travel set sized for the arm: squares of 22 to 25 mm, every one within reach.
  let placement: Board | undefined;
  for (let attempt = 0; attempt < 10000 && !placement; attempt++) {
    const candidate = {
      s: between(0.022, 0.025),
      border: between(0.004, 0.016),
      thickness: between(0.006, 0.018),
      x: between(0.17, 0.22),
      y: between(-0.02, 0.02),
      yaw: (between(-1, 1) * (6 * Math.PI)) / 180,
    };
    if (reachable(candidate)) placement = candidate;
  }
  if (!placement) throw new Error("Could not place the board within reach");
  const board: Board = placement;
  const { s, border, thickness } = board;
  console.error(
    `Scene seed ${seed}: ${(s * 1000).toFixed(1)} mm squares, board ${(thickness * 1000).toFixed(1)} mm thick`,
  );
  // Inlaid wood, each square cut from the grain at its own place and the dark ones turned
  // across it, or a plain printed board.
  const width = 8 * s + 2 * border;
  const inlaid = random() < 0.7;
  const [lightColor, darkColor] = inlaid
    ? pick([
        ["#e8d2a8", "#8a5a36"],
        ["#f0dcb4", "#6b4128"],
        ["#e2c89c", "#9a6a44"],
        ["#f3e6c8", "#3f2c22"],
      ])
    : pick([
        ["#eeeed2", "#769656"],
        ["#f5f5f0", "#7a8ea3"],
        ["#f0d9b5", "#b58863"],
        ["#e6e2d6", "#6f8f72"],
      ]);
  const light = vary(lightColor!);
  const dark = vary(darkColor!);
  const frame = vary(pick(["#5b3a22", "#3e2a1a", "#7a5230", "#8b6f4e"]));
  // The simulator's decoded images carry their pixels; a browser's ImageBitmap does not.
  const grain = woods[0]!.map.image as {
    data: Uint8Array;
    width: number;
    height: number;
  };
  const offsets = Array.from({ length: 64 }, () => [random(), random()]);
  const strength = inlaid ? between(0.5, 1) : 0;
  const size = 1024;
  const pixels = new Uint8Array(size * size * 4);
  for (let j = 0; j < size; j++)
    for (let i = 0; i < size; i++) {
      // Texture u runs along -files, v along ranks away from the robot; see `task.ts`.
      const u = -((i + 0.5) / size - 0.5) * width;
      const v = ((j + 0.5) / size - 0.5) * width;
      const inside = Math.max(Math.abs(u), Math.abs(v)) < 4 * s;
      const file = Math.floor(u / s + 4);
      const index = Math.floor(v / s + 4);
      const rank = 8 - index;
      const isDark = (file + 1 + rank) % 2 === 0;
      const color = !inside ? frame : isDark ? dark : light;
      const [ox, oy] = inside ? offsets[file * 8 + index]! : [0.3, 0.7];
      let a = (u / (4 * s)) * 0.5 + ox!;
      let b = (v / (4 * s)) * 0.5 + oy!;
      if (inside && isDark) [a, b] = [b, a];
      const gx = Math.floor((((a % 1) + 1) % 1) * grain.width);
      const gy = Math.floor((((b % 1) + 1) % 1) * grain.height);
      const k = 4 * (gy * grain.width + gx);
      const luminance =
        (grain.data[k]! * 0.3 +
          grain.data[k + 1]! * 0.59 +
          grain.data[k + 2]! * 0.11) /
        255;
      const shade = 1 + strength * (luminance / 0.45 - 1) * 0.5;
      const o = 4 * (j * size + i);
      pixels[o] = Math.round(255 * Math.min(1, color.r * shade) ** (1 / 2.2));
      pixels[o + 1] = Math.round(
        255 * Math.min(1, color.g * shade) ** (1 / 2.2),
      );
      pixels[o + 2] = Math.round(
        255 * Math.min(1, color.b * shade) ** (1 / 2.2),
      );
      pixels[o + 3] = 255;
    }
  const boardTexture = new DataTexture(pixels, size, size);
  boardTexture.colorSpace = SRGBColorSpace;
  boardTexture.minFilter = LinearMipmapLinearFilter;
  boardTexture.generateMipmaps = true;
  boardTexture.anisotropy = 4;
  boardTexture.needsUpdate = true;
  const boardBody = new RigidBody({ mass: between(0.3, 0.8) });
  boardBody.name = "chessboard";
  boardBody.position.copy(world(board.x, board.y, deskHeight + thickness));
  boardBody.rotation.y = board.yaw;
  boardBody.setMaterial({
    staticFriction: 0.5,
    dynamicFriction: 0.5,
    restitution: 0,
  });
  const slab = new BoxCollider({ size: [width, thickness, width] });
  slab.position.y = -thickness / 2;
  const slabMesh = new Mesh(
    new BoxGeometry(width, thickness, width),
    wood(1, 1, frame, 0.6),
  );
  slabMesh.position.y = -thickness / 2;
  slabMesh.castShadow = slabMesh.receiveShadow = true;
  const finish = wood(
    0,
    3,
    new Color("#ffffff"),
    inlaid ? between(0.3, 0.7) : between(0.5, 0.9),
  );
  const top = new Mesh(
    new PlaneGeometry(width, width),
    new MeshStandardMaterial({
      map: boardTexture,
      normalMap: inlaid ? finish.normalMap : null,
      roughness: finish.roughness,
    }),
  );
  top.rotation.x = -Math.PI / 2;
  top.position.y = 0.0002;
  top.receiveShadow = true;
  boardBody.add(slab, slabMesh, top);
  scene.add(boardBody);

  // The black set: one finish, its proportions varied per set.
  const style = {
    s,
    height: between(0.94, 1.06),
    width: between(0.94, 1.04),
    tint: vary(pick(["#ffffff", "#c8c0b8", "#8c7060", "#707070"]), 0.03),
    roughness: between(0.6, 1.2),
    friction: between(0.4, 0.8),
  };
  const setMaterial = pieceMaterial(style);
  // Pieces start at random places and turns within reach, on the board or beside it, apart
  // enough for the gripper and never straddling the board's edge.
  const base = 0.36 * s * style.width;
  const half = 4 * s + border;
  const taken: { x: number; y: number; r: number }[] = [];

  /** Every piece scattered so far. */
  const scattered: Piece[] = [];

  /** A piece of `kind` at a random free place within reach, standing or on its side. */
  function scatter(kind: Kind, lying: boolean): Piece {
    const body = piece(kind, style, setMaterial);
    body.name = `${kind}_${taken.length + 1}`;
    const length = pieceHeight(kind, s) * style.height;
    const r = lying ? length / 2 + base : base;
    for (let attempt = 0; ; attempt++) {
      if (attempt > 10000) throw new Error("Could not scatter the pieces");
      const distance = between(0.12, lying ? 0.26 : 0.285);
      const angle = between(-1, 1) * 1.2;
      const x = distance * Math.cos(angle);
      const y = distance * Math.sin(angle);
      if (x < 0.1) continue;
      if (taken.some((o) => Math.hypot(o.x - x, o.y - y) < o.r + r + 0.006))
        continue;
      const [u, v] = boardCoordinates(board, x, y);
      const edge = Math.max(Math.abs(u), Math.abs(v));
      const onBoard = edge < half - r - 0.002;
      if (!onBoard && edge < half + r + 0.004) continue;
      taken.push({ x, y, r });
      const support = onBoard ? deskHeight + thickness : deskHeight;
      const yaw = new Quaternion().setFromAxisAngle(
        new Vector3(0, 1, 0),
        random() * Math.PI * 2,
      );
      if (!lying) {
        body.position.copy(world(x, y, support + 0.0003));
        body.quaternion.copy(yaw);
        break;
      }
      // On its side, the middle of the piece at (x, y), resting on its base's rim.
      body.quaternion
        .copy(yaw)
        .multiply(
          new Quaternion().setFromAxisAngle(new Vector3(0, 0, 1), Math.PI / 2),
        )
        .multiply(
          new Quaternion().setFromAxisAngle(
            new Vector3(0, 1, 0),
            random() * Math.PI * 2,
          ),
        );
      const axis = new Vector3(0, 1, 0).applyQuaternion(body.quaternion);
      body.position
        .copy(world(x, y, support + base + 0.0005))
        .addScaledVector(axis, -length / 2);
      break;
    }
    scene.add(body);
    const contacts = new Set<RigidBody>();
    touching.set(body, contacts);
    body.addEventListener("contactbegin", ({ otherBody }) => {
      if (otherBody.parent === robot) contacts.add(otherBody);
    });
    body.addEventListener("contactend", ({ otherBody }) =>
      contacts.delete(otherBody),
    );
    scattered.push({ body, kind });
    return { body, kind };
  }

  /** The robot bodies each piece touches. */
  const touching = new Map<RigidBody, Set<RigidBody>>();
  function still(body: RigidBody): boolean {
    const { linear, angular } = body.getVelocity();
    return linear.length() < 0.015 && angular.length() < 0.15;
  }
  /** The board where it is now: the evaluator judges squares on the board, not the desk. */
  function current(): Board {
    const [x, y] = ros(boardBody.position);
    const forward = new Vector3(0, 0, -1).applyQuaternion(boardBody.quaternion);
    return { ...board, x, y, yaw: Math.atan2(-forward.x, -forward.z) };
  }
  /** Smallest angle between two headings. */
  function turnBetween(a: number, b: number): number {
    const d = (((a - b) % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);
    return Math.min(d, 2 * Math.PI - d);
  }
  /**
   * The piece stands upright, centred on a square setup gives its kind, and, with `facing`,
   * a knight looks towards rank 1; the other pieces look the same whichever way they turn.
   * Pieces of one kind are interchangeable.
   */
  function placed({ body, kind }: Piece, facing = true): boolean {
    const [x, y] = ros(body.position);
    const now = current();
    const square = squareAt(now, x, y, 0.2 * s);
    if (
      square === undefined ||
      setup[square] !== kind ||
      tilt(body.quaternion) > Math.PI / 18 ||
      Math.abs(body.position.y - boardBody.position.y) > 0.004
    )
      return false;
    if (kind !== "knight" || !facing) return true;
    const [fx, fy] = ros(new Vector3(1, 0, 0).applyQuaternion(body.quaternion));
    return turnBetween(Math.atan2(fy, fx), now.yaw) < Math.PI / 9;
  }
  /** The farthest the board has been pushed and turned since the start. */
  const initialPosition = boardBody.position.clone();
  const initialRotation = boardBody.quaternion.clone();
  const boardPeak = { displacement: 0, rotation: 0 };
  function boardMoved(): boolean {
    boardPeak.displacement = Math.max(
      boardPeak.displacement,
      boardBody.position.distanceTo(initialPosition),
    );
    boardPeak.rotation = Math.max(
      boardPeak.rotation,
      boardBody.quaternion.angleTo(initialRotation),
    );
    return boardPeak.displacement > 0.015 || boardPeak.rotation > Math.PI / 36;
  }
  return {
    random,
    robot,
    boardBody,
    scatter,
    touching,
    still,
    placed,
    boardPeak,
    boardMoved,
  };
}
