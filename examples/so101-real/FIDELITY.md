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
  plus a +65° wrist_roll zero error. LeRobot's roll zero is the middle of a full turn, so it is
  arbitrary, and a roll-0 calibration cannot see it; it puts the jaws where the real photos show
  them. RO §1's earlier fit was 9° off on the elbow.
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
- **Jaws:** the collision pads end at z = −0.095 in gripper_link, 3 mm above the URDF
  gripper_frame point, as the real touch test found. The CAD pads reached −0.104; the visual
  meshes still show the CAD jaws.
- **Scene:** white table at the base plane, base_link 3 mm above it.
  - Board as re-calibrated: 31.0 mm squares, a8 corner at (18.8, −9.8) cm, top 2.0 cm in
    base_link (2.28 cm above the table), yaw 3.5°, so d8 sits at (19.7, 1.1) cm.
  - Queen lying with its middle at about (24.5, 23.5) cm, crown away from the robot at 64°.
  - Seeds other than 0 move the board ±5 mm/±1° and the queen ±1 cm/±8°.
- **Arm masses:** unchanged (URDF; about 0.42 kg beyond the shoulder). See Loads.

## Checks

| Item | Real | Sim | |
| --- | --- | --- | --- |
| Views at the new real readings (`cmp-r0*`, `cmp-rc*`) | — | same board layout, orientation, squares and queen; board within ≈50–120 px | ◐ |
| Touch test (`touch.py`): jaw meets the table at model tool-point z | −0.006 (table −0.003) | −0.003 | ✅ |
| Touch test: load after contact | +30 → 0 → −232 over 24 mm, at the P rate | 79 → 59 → 34 → 7 → −21 over 9 mm | ✅ |
| Touch test: load while descending | +30 | 68–79 (≈1° more sag) | ◐ |
| Survey (pan 0, lift 0.4, elbow −13, wrist 80): shoulder / elbow load (`loads.py`) | 36–76 / 144–160 (after integral) | 30 / 232 approaching with gravity; 265 / 226 nudged from below | ✅ / ◐ |
| Survey, goal = present for 2 s: drift and load (shoulder, elbow) | 0.5°, 36; 0.7°, 44 | 0.6°, 30; 0.1°, 5 | ✅ |
| RO §2 extended pose, from below: sag, load (`actuators.py`) | 6.8°, 320 | 6.8°, 321 | ✅ |
| RO §2: 3° step down at rest | no motion, load 336 → 212 | no motion, 321 → 180 | ✅ |
| RO §2: approach-direction loads | 104…344 | 78 / 321 | ✅ |
| RO §2: overheat trip | after ≈1 h at load 300–360 | model: 58 min at 330; pressing the table at 1000: 140 s (`thermal.py`) | ✅ |
| RO §4: gentle squeeze (contact + 3) | gripper load 184 | 188 (`grip.py`) | ✅ |
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

Success needs the queen upright on d8 (centred within 9 mm, tilted under 10°, on the board top),
at rest, untouched, and no gripper, jaw or camera collider point within 2 cm of it, all held for
one second. The episode runs until that holds.

## Open

- D: at D's exact (converted) readings the sim's jaw is 3.1 cm up; the real arm's was ≈0.5 cm by
  eye. B lands 3° of lift early. Both suggest the real arm hangs a little lower far out than the
  careful calibration says. Not chased further.
- The images are cleaner than the real ones: no red/pink cast, no blur, crisp shadows.
- Not modelled: load noise, the real v47 gripper and camera mount geometry beyond the pad length,
  and Feetech overload protection.
