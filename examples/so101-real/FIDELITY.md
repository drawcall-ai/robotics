# Fidelity: real SO-101 queen scene vs. simulation

Ground truth is REAL_OBSERVATIONS.md (measured 2026-10-07/08). The evidence lives in `evidence/`,
made by builder-only scripts that read the simulator's true joint angles (`truth.py`). All runs
use seed 0, the measured scene. Status: ✅ reproduced, ◐ partly, ✗ not yet.

## How it is built (simple on purpose)

- **Readings ≠ joints** (`calibration.ts`): true angle = reading + offset + share·extra, with
  offsets (0, +4.4, −10.8, +5.2, +65)° for pan, lift, elbow, wrist_flex, roll, and extra = +10° on
  the shoulder. The share rises linearly from 0 at a lift reading of 5° to 1 at 35°. The first
  four offsets are the camera calibration's fit (§1), valid where the upper arm is near vertical.
  The extra shoulder term reproduces the §5 near-table error. The +65° roll offset is invisible to
  a roll-0 camera calibration (LeRobot's roll zero is the middle of a full turn, so arbitrary). It
  puts the jaws where the real photos show them.
- **Servos** (`servo.ts`): P-only, torque = K(goal − present) capped at stall (2.94 N·m; gripper
  50 %), K = 7.9 N·m/rad at P=16 (47 load units/°). Stiction: a joint at rest ignores goal changes
  within ±τs/K, with τs = 0.42 N·m (±3° at P=16), then jumps to the goal. Present_Load =
  K(present − goal) in ‰ of stall. Heat: dT/dt = 0.3125·load² − (T − 25)/1440 s, trip at 70 °C
  (torque off), back on below 55 °C with a new goal. A pause between turns charges its wall-clock
  time, up to 120 s, as holding time.
- **Camera** (`robot.ts`): 1280×960, vertical FOV 2·atan(480/1061) (fx = fy = 1061, centred), at
  the calibrated transform of §3, turned back by the roll offset. No magenta cast.
- **Scene** (`environment.ts`): white table at the base plane (base_link +3 mm); board with
  31.4 mm squares, 15 mm frame, top 30 mm above the table, centre (29.35, 1.65) cm, yaw 2°, fold
  seam between ranks 4 and 5; queen 7.55 cm long, base 2.9 cm, ≈20 g, friction 0.35, lying from
  (23.0, 17.4) towards (26.3, 24.2) cm. d8 lands at (18.4, −0.3) cm.

## §3 Camera

| Item | Status | Evidence |
| --- | --- | --- |
| Intrinsics 1280×960, f = 1061, centred | ✅ | `robot.ts`; CameraInfo k = [1061, 0, 640, 0, 1061, 480] |
| Calibrated mount, jaws bottom-centre, fixed tip ≈ (710–735, 650–670) px | ✅ | `cmp-01-wrist40.jpg`, `cmp-03-pan-30.jpg`: black jaw left of white, as in the real images; projected fixed tip ≈ (735, 670) |
| Views in the calibration region | ✅ | `cmp-03-pan-30.jpg`: board corner within ≈40 px; `cmp-01-wrist40.jpg` board rows within ≈40 px |
| Views near the table (§5 poses) | ◐ | `cmp-26-view.jpg`, `cmp-30-sA/sB.jpg`, `cmp-20-pre.jpg`: same content and layout; queen within 30–110 px, board edge 50–175 px off (≈1–2 cm on the table); `fit.txt` |
| Calibration only valid at roll ±20° (§6.7) | ◐ | The roll offset makes any roll-0 calibration wrong about where the jaws are; views at other rolls are off by about as much as at roll 0 (see the previous row), so the solver's own calibration will degrade there. Not separately modelled. |
| Magenta cast, libcamera | ✗ skipped | Not relevant to the task |

## §5 Contact events

Commanded from above with the gripper open, as the real attempts did (`contacts.py`,
`contacts.jsonl`, `contacts-audit.txt`, `sim-<pose>-*.jpg`). *Plain*: goals = the real readings,
P=16. *Reached*: goals nudged until the readings match, like the real sag correction.

| Event | Real | Sim | Status |
| --- | --- | --- | --- |
| B (−46.5, 53.3, −49.0, 83.2, −64.7) | fixed jaw on table, model says −0.3 cm | plain: jaw on table, blocked at lift 42.1 (load −526); reached: on table, lift 44.0, the correction drives the shoulder to −1000 | ✅ |
| C (−48.7, 48.8, −44.7, 83.0, −71.7) | on table again, model +0.6 | plain: on table at lift 40.3 (load −400); reached: on table | ✅ |
| D (−48.7, 34.8, −36.0, 83.2, −71.7), "up 4 cm" from C | ≈0.5 cm above table, model +4.5 | reached (35.0, −35.9, 80.9): **0.49 cm**; plain from C: stays on the table (sag + stiction) | ✅ |
| Descent at (−44.8, 37.5, −31.4, 82.5, −62.5) | jaw apparently resting on the queen's base | jaw on the table beside the base, queen untouched | ✗ (queen pose or jaw shape off by ~1 cm) |
| Early test (0, −30.7, 93.4, 0, 0) | gripper pressed into table | elbow goal clamped to the calibrated 86.8°; tip 3.5 cm above table | ✗ (a reading beyond the calibrated range; unclear on hardware) |
| Light contacts, low forces; load misses them | yes | every touch is logged on `/sim/audit` with links, duration and servo loads | ◐ forces need PACKAGE_CHANGES §2 |

## §2 Actuators

`actuators.py`, `actuators.jsonl` (pose: pan 60, lift 22, elbow −31, wrist 78, clear of the board);
`thermal.py`, `thermal.jsonl`.

| Item | Real | Sim | Status |
| --- | --- | --- | --- |
| Holding sag, extended pose, P=16 | lift −6.8° at load 320 (hysteresis 104…344) | lift −3.4° at load 215, elbow −2.4° | ✅ gravity load matches the real mean (≈224) |
| 3° down from holding: no motion | load 336 → 212, no motion | no motion, load 215 → 27 | ✅ |
| Larger downward move: unload, then sudden drop | yes | 6° step: arm drops 6° within 0.3 s | ✅ |
| Interpolating from present position | load +320 → −168, arm drops | load 221 → 49, arm drops 1.7° | ✅ |
| Free-air lag at 4°/s | ≤ 12 raw (1°), P=32 + integral nudges | 3–4° at P=16, 1.4–1.9° at P=32, no nudges | ◐ no kinetic friction (PACKAGE_CHANGES §3) |
| P=32 halves the holding offset | yes | yes (stiffness ∝ P) | ✅ |
| Overheat → torque off → arm drops | after ≈1 h of holding at 300–360 | model: 58 min at load 330; others ≤ 35 °C at load ≤ 150. Run: pressing into the table at 1000 trips in 140 s, torque off, arm drops 5° onto the table | ✅ |
| Gripper squeeze, contact + 3 units | load 184 | load 188 | ✅ |

## §6 Failure modes the solver must be able to hit

| Mode | Status | How |
| --- | --- | --- |
| 1. Weak grip, piece slides out | ◐ | Low friction (0.35), gripper torque cap 50 %, squeeze load matches. In `grip.py` (standing queen at the real spot, `cmp-grip.jpg`) the lift came up empty rather than lifting and then slipping. |
| 2. Tool-down unreachable far out (wrist ±86) | ✅ | Goals clamped to the calibrated ranges; the grip test's best pose at 32 cm radius tilts the tool 10–20° |
| 3. Open fixed jaw lands on the piece | ✅ | Real jaw geometry and the moving jaw's 130° travel; in the grip test the jaw landed on the queen (audit: queen contact with load −1000) |
| 4. Near-table kinematic error | ✅ | §5 rows B, C, D |
| 5. Sag + dead band, jerky descents | ✅ | §2 rows |
| 6. Overheating, sudden drop | ✅ | §2 last rows |
| 7. Camera model valid only near calibration poses | ◐ | See §3 |

## Not modelled

Colour cast; load noise and the exact hysteresis shape; the real v47 gripper and camera mount
geometry (CAD gripper used, camera board as a 25×24×12 mm box); queen mass beyond "tens of grams".
