# Fidelity: real SO-101 queen scene vs. simulation

Ground truth is REAL_OBSERVATIONS.md (measured 2026-10-07/08). The evidence lives in `evidence/`,
made by builder-only scripts that read the simulator's true joint angles (`truth.py`). The robot
publishes those privately (recorded in the bag, never on the policy's ROS graph), so the scripts
run with `SO101_PUBLIC_TRUTH=1`, e.g.
`SO101_PUBLIC_TRUTH=1 pixi run drawcall-robotics run --seed 0 --real-time-factor 0 -- python3 evidence/actuators.py`.
All runs use seed 0, the measured scene. Status: ✅ reproduced, ◐ partly, ✗ not yet.

## How it is built (simple on purpose)

- **Readings ≠ joints** (`calibration.ts`): true angle = reading + offset + share·extra, with
  offsets (0, +4.4, −10.8, +5.2, +65)° for pan, lift, elbow, wrist_flex, roll, and extra = +10° on
  the shoulder. The share rises linearly from 0 at a lift reading of 5° to 1 at 35°. The first
  four offsets are the camera calibration's fit (§1), valid where the upper arm is near vertical.
  The extra shoulder term reproduces the §5 near-table error. The +65° roll offset is invisible to
  a roll-0 camera calibration (LeRobot's roll zero is the middle of a full turn, so arbitrary). It
  puts the jaws where the real photos show them.
- **Servos** (`servo.ts`): P-only, torque = K(goal − present) capped at stall (2.94 N·m; gripper
  50 %), K = 7.9 N·m/rad at P=16 (47 load units/°). Gear friction: MuJoCo joint friction
  (Coulomb) of 0.35 N·m on the arm joints, §2's ±120 load units of hysteresis; a joint at rest
  ignores goal changes within 0.35 N·m/K (2.5° at P=16). Present_Load = K(present − goal) in ‰ of
  stall. Heat: dT/dt = 0.3125·load² − (T − 25)/1440 s, trip at 70 °C (torque off), back on below
  55 °C with a new goal. A pause between turns charges its wall-clock time, up to 120 s, as holding
  time at the load the arm paused with (the package's `Device.resumed` hook).
- **What the policy sees** (`robot.ts`, `scene.ts`): the robot, the evaluator and the audit trail
  publish privately. A policy's `ros2 topic list` during a run shows only `/clock`,
  `/follower/{goal,joint_states,p_gain,status}`, `/parameter_events`, `/rosout`,
  `/robot/wrist_camera/{camera_info,image_raw,image_raw/compressed}` and rosbag2's own
  `/events/write_split`; the bag still holds `/robot/joint_states`, `/tf`, `/tf_static`,
  `/robot/robot_description`, `/sim/audit`, `/sim/evaluation` and `/sim/score`, and `cat` shows
  them.
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
| B (−46.5, 53.3, −49.0, 83.2, −64.7) | fixed jaw on table, model says −0.3 cm | plain: jaw on table, blocked at lift 41.2 (load −565), peak 15.1 N; reached: on table, lift 44.4, the correction drives the shoulder to −1000, peak 44.9 N | ✅ |
| C (−48.7, 48.8, −44.7, 83.0, −71.7) | on table again, model +0.6 | plain: on table at lift 38.4 (load −487), peak 15.1 N; reached: on table, peak 23.7 N | ✅ |
| D (−48.7, 34.8, −36.0, 83.2, −71.7), "up 4 cm" from C | ≈0.5 cm above table, model +4.5 | reached (34.6, −36.0, 82.7): **0.73 cm**; plain from C: stays on the table (sag + friction) | ✅ |
| Descent at (−44.8, 37.5, −31.4, 82.5, −62.5) | jaw apparently resting on the queen's base | jaw on the table beside the base; reached: brushes the queen for 0.06 s at 2.7 N | ✗ (queen pose or jaw shape off by ~1 cm) |
| Early test (0, −30.7, 93.4, 0, 0) | gripper pressed into table | elbow goal clamped to the calibrated 86.8°; tip 3.5 cm above table | ✗ (a reading beyond the calibrated range; unclear on hardware) |
| Light contacts, low forces; load misses them | yes | every touch is logged on `/sim/audit` with links, duration, servo loads and its peak contact force (`world.contacts`); metrics `maxForce_table`, `_chessboard`, `_queen`. Plain touches 15 N, sag-corrected ones 24–45 N; the real forces were not measured | ✅ |
| Contact force = weight at rest | — | queen standing on the table: 0.323 N = m·g (32.9 g), error 0 % (`weight.ts`) | ✅ |

## §2 Actuators

`actuators.py`, `actuators.jsonl` (pose: pan 60, lift 22, elbow −31, wrist 78, clear of the board,
the shoulder settled coming up from below, as the real holding load of 320–336 shows);
`thermal.py`, `thermal.jsonl`.

| Item | Real | Sim | Status |
| --- | --- | --- | --- |
| Holding sag, extended pose, P=16 | lift −6.8° at load 320 | lift −5.3° at load 335, elbow −4.8° | ✅ |
| Holding load by approach direction | 104…344 at an identical pose | 93 from above, 335 from below | ✅ |
| 3° down from holding: no motion | load 336 → 212, no motion | no motion, load 335 → 148 | ✅ |
| Larger downward move: unload, then sudden drop | yes | the next 3°: load 148 → −35, then the arm drops 2.3° within 0.3 s | ✅ |
| Interpolating from present position | load +320 → −168, arm drops | from a low-load hold: load 100 → 17, arm drops 1.5° | ✅ |
| Free-air lag at 4°/s | ≤ 12 raw (1°) descending, P=32 + integral nudges | descending: 0.8–1.2° at P=16, ≤ 0.4° at P=32; rising against gravity and friction: 6–7° at P=16, ≈3° at P=32; no nudges | ✅ |
| P=32 halves the holding offset | yes | yes (stiffness ∝ P) | ✅ |
| Overheat → torque off → arm drops | after ≈1 h of holding at 300–360 | model: 58 min at load 330; others ≤ 35 °C at load ≤ 150. Run: pressing into the table at 1000 trips in 140 s, torque off, arm drops 1.8° onto the table | ✅ |
| Heating while paused between turns | (operator thinking time) | a 32 s pause holding at load 1000: 31 → 41 °C | ✅ |
| Gripper squeeze, contact + 3 units | load 184 | load 188 | ✅ |

## §6 Failure modes the solver must be able to hit

| Mode | Status | How |
| --- | --- | --- |
| 1. Weak grip, piece slides out | ◐ | Low friction (0.35), gripper torque cap 50 %, squeeze load matches. In `grip.py` (standing queen at the real spot, `cmp-grip.jpg`) the lift came up empty rather than lifting and then slipping. |
| 2. Tool-down unreachable far out (wrist ±86) | ✅ | Goals clamped to the calibrated ranges; the grip test's best pose at 32 cm radius tilts the tool 10–20° |
| 3. Open fixed jaw lands on the piece | ✅ | Real jaw geometry and the moving jaw's 130° travel; in the grip test the jaw landed on the queen (audit: queen contact with load −1000) |
| 4. Near-table kinematic error | ✅ | §5 rows B, C, D |
| 5. Sag + friction dead band, jerky descents | ✅ | §2 rows |
| 6. Overheating, sudden drop | ✅ | §2 last rows |
| 7. Camera model valid only near calibration poses | ◐ | See §3 |

## Not modelled

Colour cast; load noise and the exact hysteresis shape; the real v47 gripper and camera mount
geometry (CAD gripper used, camera board as a 25×24×12 mm box); queen mass beyond "tens of grams".
