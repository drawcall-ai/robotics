# Fidelity: real SO-101 queen scene vs. simulation

Real data:
- REAL_OBSERVATIONS.md (RO, 2026-10-07/08). Its readings predate a calibration change:
  old lift reading = new − 5.36°, old elbow = new + 3.52°.
- The real arm's careful re-calibration on the evening of 2026-10-08 (the solver's
  `policy/real/DIFFERENCES.md`; views copied to `evidence/real2/`), in today's readings. Where the
  two conflict, the re-calibration wins.

Evidence is in `evidence/`, made by builder-only scripts that read the true joint angles
(`SO101_PUBLIC_TRUTH=1`, `truth.py`). Seed 0. ✅ reproduced, ◐ partly, ✗ not.

## The model

- **Readings ≠ joints** (`calibration.ts`): true angle = reading + offset. The offsets are the
  re-calibration's fit from 12 board views: pan 0, lift −0.09°, elbow +1.55°, wrist_flex −0.71°,
  plus a +100° wrist_roll zero error. LeRobot's roll zero is the middle of a full turn, so it is
  arbitrary, and a roll-0 calibration cannot see it. With 100° the fitted camera mount coincides
  with the printed v47 gripper's designed camera (`v47_poses.json`): 5.4 mm, 0.5°. The earlier 65°
  was an eyeball fit to the jaw tips. RO §1's earlier fit was 9° off on the elbow.
- **No hidden bending.** A load-dependent bend between encoder and link has been removed. In the
  real touch test the load fell steadily at the P rate after contact (no unbending). The new
  offsets meet RO's table contacts without it. And unclamped, it let attempt 2 fling the queen.
- **Calibrated ranges** (goals clamped): pan ±101.6°, lift ±95.4°, elbow ±90.3°, wrist_flex ±85.9°,
  roll ±180°, gripper 0..100 over 130°. The arm starts in the real rest pose: readings
  (−5.7, −94.5, 89.5, 16.9, −5.6, 5.2) real, (−5.7, −94.5, 89.6, 17.0, −5.6, 5.2) sim.
- **Servos** (`servo.ts`): P-only on the reading, K = 7.9 N·m/rad at P=16, capped at stall
  (2.94 N·m; gripper 50 %). MuJoCo Coulomb friction is 0.35 N·m on the arm joints.
  Present_Load = K(present − goal) in ‰ of stall. Heat: dT/dt = 0.3125·load² − (T − 25)/1440 s,
  torque off at 70 °C until below 55 °C and a new goal. A pause between turns counts its
  wall-clock time, up to 120 s, as holding time.
- **Camera:** 1280×960, fx = fy = 1061, centred, at the re-calibration's mount. That mount is
  within 0.8° and 3 mm of RO §3's. No colour cast: the real images are strongly red/pink.
- **Jaws:** stock SO-101. The printed v47 gripper is the stock fixed-jaw body (tip at z
  −104.4 mm, gripping face at x −7.9 mm) fused with a Pi 4 frame near the wrist and the camera
  plate; the moving jaw is stock. The Pi frame is not modelled. Relative to the calibrated camera,
  the gripper body and moving jaw (visuals and colliders) sit 4 mm further up gripper_link's z
  (`JAW_SHIFT`). The real board touches say 4.6 mm; the closed tips' pixels in the touch photos
  say about 3 mm (`cmp-touch-tips.jpg`); 4 mm fits both within their precision.
- **Scene:** white table at the base plane, base_link 3 mm above it.
  - Board as re-calibrated: 31.0 mm squares, a8 corner at (18.8, −9.8) cm, top 2.0 cm in
    base_link (2.28 cm above the table), yaw 3.5°, so d8 sits at (19.7, 1.1) cm.
  - Queen lying with its middle at about (24.5, 23.5) cm, crown away from the robot at 64°.
    It is the CC0 Staunton queen scaled to the real one as measured with the re-calibrated camera
    from above (rr_V3): base 2.26 cm, crown collar ≈1.5 cm, so ×0.56 in width, and 7.3 cm long.
    RO §4's 2.9 cm base came through the old calibration. The old 2.9 cm, 7.55 cm sim queen was
    bigger, so a real queen that looks the same in the camera stands farther away.
  - Friction (MuJoCo takes the larger of the two): queen 0.2 (felt base), board top 0.2, table
    0.5, arm 0.8. So only the queen on the board slides easily. A queen pushed near its head
    slides when friction is below base radius / push height, ≈0.2–0.26, and tips otherwise. The
    real one slid upright. Before this the queen ran at the default 0.5, because each of its
    colliders carried its own material, which replaces the body's whole one. The 0.35 it named
    never applied.
  - Seeds other than 0 vary widely, for robustness rather than fidelity. Board: ±5 cm, ±15°,
    28–35 mm squares, 10–22 mm frame, ±4 mm height, wood colours. Queen: standing or lying,
    anywhere within reach off d8, any heading, 0.85–1.15× size, dark to medium tint. Light:
    a red/pink cast from 0 to the real camera's and 0.6–1.3× brightness, as a light colour
    rather than a camera filter. Contact sheet: `evidence/seeds-sheet.jpg`.
- **Arm masses:** unchanged (URDF; about 0.42 kg beyond the shoulder). See Loads.

## Checks

| Item | Real | Sim | |
| --- | --- | --- | --- |
| Views at the new real readings (`cmp-r0*`, `cmp-rc*`) | — | same board layout, orientation, squares and queen; board within ≈50–120 px | ◐ |
| Touch test (`touch.py`): jaw meets the table at model tool-point z | −0.006 (table −0.003) | +0.009: the real jaw sits ≈1.5 cm higher in this pitched-down pose; ≈3 mm of it is the fitted vs designed camera mount | ✗ |
| Touch test: load after contact | +30 → 0 → −232 over 24 mm, at the P rate | 79 → 59 → 34 → 7 → −21 over 9 mm | ✅ |
| Touch test: load while descending | +30 | 68–79 (≈1° more sag) | ◐ |
| Survey (pan 0, lift 0.4, elbow −13, wrist 80): shoulder / elbow load (`loads.py`) | 36–76 / 144–160 (after integral) | 30 / 232 approaching with gravity; 265 / 226 nudged from below | ✅ / ◐ |
| Survey, goal = present for 2 s: drift and load (shoulder, elbow) | 0.5°, 36; 0.7°, 44 | 0.6°, 30; 0.1°, 5 | ✅ |
| RO §2 extended pose, from below: sag, load (`actuators.py`) | 6.8°, 320 | 6.8°, 321 | ✅ |
| RO §2: 3° step down at rest | no motion, load 336 → 212 | no motion, 321 → 180 | ✅ |
| RO §2: approach-direction loads | 104…344 | 78 / 321 | ✅ |
| RO §2: overheat trip | after ≈1 h at load 300–360 | model: 58 min at 330; pressing the table at 1000: 140 s (`thermal.py`) | ✅ |
| RO §4: gentle squeeze (contact + 3) | gripper load 184 | 188 (`grip.py`). Re-run after the jaw shift: its planned neck grasp now closes on nothing (queen on the table, untouched), so its targets need refitting | ◐ |
| RO §5 B, reached (`contacts.py`, readings converted) | jaw on table at 58.7 / −52.5 | on table at 55.7 / −53.3 (new offsets put B 1.4 cm below the table) | ◐ |
| RO §5 C, reached | on table at 54.2 / −48.2 | on table at 53.0 / −48.6 | ✅ |
| RO §5 C → "up 4 cm", plain P=16 | jaw ≈0.5–1 cm up | 0.45 cm (readings stop at 47.4 / −39.8 under sag and friction) | ✅ |
| RO §5 D's exact readings, reached | ≈0.5 cm (by eye) | 3.1 cm | ✗ |
| RO §5 Q, reached | resting on the lying queen's base (≈2.9 cm) | 1.3 cm, the queen elsewhere | ◐ |
| RO §5 early (folded down in front) | pressed into the table | 2.8 cm up, elbow short of its goal | ✗ |

**Loads.** The sim's gravity torques are shoulder 0.45 N·m and elbow 0.37 N·m in the survey pose,
and shoulder 0.65 N·m in RO §2's extended pose. The solver's "sim 262 vs real 36–76" compared loads
taken in opposite friction states. The sim's 262 was nudged up from below (gravity + friction); the
real arm arrived with gravity. Approached the same way, the sim gives 30. The real shoulder held
with goal = present, so its gravity torque is at most friction plus 36 units, about 0.45 N·m, which
the sim just meets. The touch descent hints the sim shoulder carries ≈0.1 N·m more than the real
one. That is within friction's uncertainty, so the masses stay.

| Real lying grasp + stand-up, replayed (`replay.py`, plans L1–L4, `lying.jsonl`) | clamps at 7.1, load 252; carried; stands on the board | queen placed at the noted base point: clamps at 12.7, load 227, then slips out while lifting. Placed by the noted head point instead: closes on nothing (`lyingB.jsonl`). The real queen pose is only known to ≈1 cm. With the earlier, top-heavier queen shape it clamped and stood the queen up at (33.9, 1.6) cm. Since the jaw shift and the folded-pose term, the L1 descent lands the gripper on the base rim, shoves the queen ≈1 cm and closes on nothing (gripper 3, load 0), the same before and after the friction fix, so stand-up and set-down are not replayed | ✗ |
| Real push (`push.py`, `real7/push_joints.csv`, `push.jsonl`): real readings from rest to the end of the push, at real timing, with an integral like the real harness; queen standing at (18.3, 0.9) cm | slid ≈4 cm outward, stayed upright | slides 6.8 cm outward, max tilt 3.7°. Before the friction fix it tipped over at the first jaw touch (0.33 N) | ✅ |
| Real neck regrasp of the standing queen, replayed (plans S1 + f, `neck.jsonl`, `cmp-neck.jpg`) | closes past the queen to 5.3, load ≤ 32; black jaw sweeps in front of it | queen where rr_S1 shows it, (34.7, −0.5) cm, back-projected; the solver's own estimate was 2 cm off because it assumed the bigger sim queen. Same view; the jaw pushes the crown aside, gripper 7.7, load 167, no clamp (its stop is 250) | ◐ |

## Failure modes the solver must be able to hit (RO §6)

1. Weak grip, piece slides out: ◐. The squeeze load matches and the gripper is capped at 50 %.
2. Tool-down poses unreachable far out (wrist ±86°): ✅.
3. The open jaw lands on the piece: ✅.
4. Near-table kinematic error: ◐. The sim now follows the careful calibration, so the remaining
   near-table error is the real calibration's residual. D says the real arm is ≈2.6 cm lower there
   than the sim.
5. Sag and the friction dead band, jerky descents: ✅.
6. Overheating, sudden drop: ✅.
7. Camera model valid only near the calibration poses: ◐ (roll offset).

## Evaluation

`TASK=scatter` (the antagonist task): the queen starts upright on d8. Success means its middle is
at rest a square or more from d8's centre, untouched, with the arm withdrawn. Board moved or queen
fallen off fail as in the main task. Checked with a temporary edit: a queen on a8 succeeds at 1.0 s.

Success needs the queen upright on d8 (centred within 9 mm, tilted under 10°, on the board top),
at rest and untouched, with the arm withdrawn, all held for one second. The episode runs until that
holds. Withdrawn means no gripper, jaw or camera box collider within 2 cm of the queen (exact
box-to-axis distance; the queen counts as a capsule as wide as its base) and the queen not
between the jaws, however wide open. The jaw rule came from the scatter solver: success fired
during a slow retreat with open jaws still around the queen, 2.7 cm from it. Regression check
(`withdraw_check.py`, `withdraw_check.txt`): jaws closed on a queen off d8 for 5 s, then opened
wide around it for 5 s, give no success; it comes 0.4 s after the arm has lifted away.

## Open

- Board touches D1/D2 (`touch_fit.py`, `touch_fit.txt`, readings in `real5/`). At the real
  contact readings the sim's lowest jaw point is now −1.7 to +0.2 mm from the board (mean
  −0.7) at all ten touches. The D1 x 0.33 touch is a false start. Before the folded-pose term and
  the jaw shift it was −9.4 to −10.7 mm on the d8 row and −3.6 to −5.5 mm elsewhere.

- Near the table, pitched down, the real jaws sit ≈1–1.5 cm higher than the careful calibration
  says (touch test; the neck regrasp agrees). The camera agrees within 2 mm there. So the
  residual lies between camera and jaws: ≈3 mm is the fitted vs designed mount; the rest is
  unexplained.

- D: at D's exact (converted) readings the sim's jaw is 3.1 cm up; the real arm's was ≈0.5 cm by
  eye. B lands 3° of lift early. Both suggest the real arm hangs a little lower far out than the
  careful calibration says. Not chased further.
- The images are cleaner than the real ones: no red/pink cast, no blur, crisp shadows.
- Not modelled: load noise, the real v47 gripper and camera mount geometry beyond the pad length,
  and Feetech overload protection.
