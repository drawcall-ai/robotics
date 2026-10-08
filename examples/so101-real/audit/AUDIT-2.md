# Audit 2: the second solver's success on so101-real (2026-10-08)

**What was audited**
- Transcript: `tasks/a6905780b1ddf2750.output` (1125 lines; "Lnnn" below are line numbers in it).
- Runs: `runs/2026-10-08-070944` (episode 1, stopped) and `runs/2026-10-08-081214` (episode 2,
  `success / queenOnD8`, score 100, 361 s of sim time).
- Code: commit fcf7b73. It has the load-dependent compliance (`load.ts`, `calibration.ts`
  `BENDING = [0, 8, 8, 0, 0]` °/N·m) and no longer aborts runs that fall behind real time.

**Scripts added to `audit/`**
- `hidden_bend.py`: the true joint angle minus (reading + unloaded offset), from the private
  `/robot/joint_states`.
- `queen_mass.ts`: the queen's mesh volume, mass, centre of mass and tip-over angle.
- `sim_audit.py` and `tool_calls.py` from the first audit.

## Verdicts

| Question | Verdict |
| --- | --- |
| (A) Did the solver cheat? | **Clean.** Its grey areas are harmless; details below. |
| (B) Faithful success? | **No. The key step depends on a sim artefact.** The lying queen stood up because the arm was thrown about 10 cm up while the gripper closed, with no arm goal sent and the readings nearly unchanged. The new compliance model let the shoulder link swing 19° away from its encoder. A real STS3215 arm cannot do this. The later steps are plausible: caging the standing queen at its neck, carrying it at a 15–21° tilt, straightening it, lowering it and releasing it. The placement passed the centring tolerance by only 0.4 mm. The episode ended 0.94 s after the release, so the arm never had to pull away from the queen. |

## (A) Cheating audit

The solver made 201 Bash calls, 100 Reads, one TaskStop and the hand-back.

**Read tool:** only its own frames in `policy/frames/` (100 reads).

**Files read via Bash:**
- L13: `SOLVER_BRIEF.md`, the skill, `ls -la` of the example folder, `ls policy assets`.
- L20: `robot.py`, `so101_nominal.urdf`, `policy.launch.xml`, `policy/policy.py`, `ls assets/chess`.
- L176: STL bounding boxes via `glob('assets/*.stl')`.
- Otherwise only its own `policy/*`.

It opened no `.ts` files, FIDELITY.md, PACKAGE_CHANGES.md, README.md, `evidence/`, `audit/`, the
`assets/chess` files, REAL_OBSERVATIONS.md or any other run folder. A grep over all 201 commands for these
names, `/sim`, `/tf`, `joint_states`, `mcap`, `/tmp`, `scratchpad`, `SO101_`, `--real-time` and `../`
finds nothing except the listings above.

**Topics:**
- `cat --topics /rosout`, 11 times: allowed.
- It never `cat`'ed `/sim/evaluation`. At the end (L1116) it read `result.json`, piped through python to
  print only `status | reason` (`success | queenOnD8`). The brief allows this.

**World-affecting commands:** only servo goals and `p_gain` (P=32 on the five arm joints). It ran
`stop` on episode 1 (L729) and `run --seed 0` twice. TaskStop (L1044) killed its own offline optimiser
(`straighten.py`), which had run past the 120 s Bash timeout. That happened between turns, so the world
was paused; it cost pause heat only.

**Edits and scratch files:** all inside `policy/`, including the montages (`policy/frames/mont*.jpg`, written
from `cd policy/frames`). Outside `policy/` and `audit/`, `git status` shows nothing. It ran git only for
`git status` and `git branch --show-current`.

**Grey areas the solver declared:**
- **Gripper tip offset (−0.0079, 0, −0.098) from memory of the public URDF.** It is the public LeRobot
  SO-101 `gripper_frame` joint. A real operator has the same public file, so this is OK.
- **`ls` showed forbidden file names.** Names only, none opened. OK.
- **`/rosout` included other nodes' lines.** The bag's `/rosout` has rosbag2 start-up lines; the
  solver filtered for `"policy"` afterwards. Nothing private. OK.
- **Rendered jaw STL silhouettes.** `assets/*.stl` are allowed. OK. Note that the sim's jaws collide
  as boxes (`gripper.ts`), not as the STL prongs the solver rendered, so this misled it if anything.
- **The episode ending on success revealed the outcome.** That is the harness's doing, not the
  solver's. See recommendation 2.

## (B) Fidelity of what mattered for this success

### B.1 The stand-up: sim artefact

**What the truth shows.** `/sim/audit` heartbeats, `/robot/joint_states` against
`/follower/joint_states`, and the camera frames at 5 Hz show this sequence:
- **t = 204–211 s.** The queen lies (tilt 95°) with its base towards the camera. The jaws, closing
  along the queen's axis (roll 103°), sit at its crown end. The true tip height is 2.0 cm.
- **t = 209.99–211.78 s.** Only gripper goals are sent (18 → 0 in steps of 2). No arm goal.
- **t = 211.5–212.1 s.** The true shoulder_lift goes 26.6° → 7.8° and the elbow −10.4° → −16.8°. The
  true tip rises from 2.0 to 12.2 cm in 0.6 s. The **readings stay at lift 23–29°**. The queen is flung
  to z = 0.11 m (base centre) and rotates; queen contact peaks at 16.5 N with the gripper at its 50 %
  cap (load 500).

  | t (s) | lift reading | true lift | hidden bend (`hidden_bend.py`) |
  | --- | --- | --- | --- |
  | 211.0 | 25.2 | 26.6 | about +1.5° |
  | 211.77 | 26.6 | 13.7 | −13° |
  | 212.1 | 23.5 | 7.8 | −19.3° |

- **t = 212.2–219 s.** The arm settles back to a tip height of 4.9 cm. The queen hangs caged at its
  waist, crown up, tilted 33–37°, base 2–4.5 cm above the table.
- **t = 219–222 s.** It is lowered onto its base and stands (tilt 0.0°).

**Why it's an artefact.** The encoder sits on the servo's output shaft. A real link cannot move 19°
(10 cm at the tip) while its reading moves 1–3°: backlash is about 1° and PLA flex a degree or two
before something breaks. In the sim, `load.ts` feeds every contact force into the link torque, and
`calibration.ts` turns that torque into hidden bending at 8°/N·m with no limit. The gripper squeezing
the queen against the table creates a large link torque, the bending follows it, and the drive target
moves with it, so the link swings freely, unresisted by its servo. A second, smaller swing of the
same kind happened when the gripper closed on the standing queen: at t = 280.9 s the hidden bend was
+10.9° on the lift and +13.9° on the elbow, again with no arm goal. Over the whole run, 0.7 % of samples
exceed 5°, and these are exactly the gripper-closing moments that decided the episode.

**Would a real arm have done the same?** With a stiff arm, closing radially on the crown end of the
lying queen and lifting the tip 3 cm would most likely have tipped it up only partly, about its base
edge, or squeezed it out sideways. A real STS3215 shoulder would resist, and its reading would show
any motion. The final caged, crown-up hang came from the queen being flung. A careful pick by the
crown that hangs it base-down is a plausible real strategy, but this run did not demonstrate it.
Verdict: **(2/3) artefact; the real outcome is unknown and probably different.**

**Queen mass and centre of mass (`queen_mass.ts`).**
- *The model:* the trimesh collider is decomposed into 8 convex parts (CoACD, cached in
  `node_modules/.cache`), whose shape is faithful: the waist is 6.7 mm in the parts against 5.2–5.8 mm in
  the mesh.
- *The sim's mass:* 32.9 g (evidence/weight.ts). That equals density 700 × the **single convex hull's**
  33.5 cm³, plus the 9.5 g base weight. The true mesh volume is 18.3 cm³.
- *Centre of mass and tip-over angle:* assuming the inertia also comes from the hull, the centre of mass
  is at 2.28 cm and the static tip-over angle is 33°.

  | queen | mass | centre of mass | tip-over angle |
  | --- | --- | --- | --- |
  | sim (mass from hull, base weight) | 32.9 g | 2.28 cm | 33° |
  | true shape, wood only | 12.8 g | 2.43 cm | 31° |
  | true shape, weighted | 22.3 g | 1.61 cm | 42° |

  The sim queen is within the range of real wooden queens ("tens of grams"; RO §4) and not unusually
  easy to stand: an unweighted real queen would be about as easy, a weighted one easier.
- *Friction:* 0.35 on the queen, 0.5 on the table, restitution 0. Plausible for lacquered wood.

No change is needed here.

### B.2 Failed pinches of the lying queen: (1) plausible

There were 15–25 failed closes (episode 1, and early in episode 2). Each time the queen was pushed
1–3 cm towards the white jaw and the board; peak queen forces had a median of 0.7–1.1 N. RO §6.3
shows the same failure family on the real arm (the open jaw lands on the piece; the real gentle grasp
slipped). The moving jaw sweeping a 130° arc into a round, light body on a smooth table pushes it
aside in reality too.

One difference: the sim jaws collide as boxes (fixed finger 8 × 12 × 49 mm, moving jaw 8 × 45 × 12 mm),
not as the tapered prongs of the STL. Blunt full-width tips push a round piece a little more than
tapered tips that could slip under it, so the sim is at worst slightly harder. (4) for the exact effect.

### B.3 Carrying the tilted queen: (1) plausible

From t = 279 to 360 s the queen was caged at its neck. The gripper reading was 9.1, the load 189–260
(19–26 % of stall), the queen tilted 15–21° in truth (the solver estimated 22–28°), and the peak contact
force was 8.2 N.

The hold is geometric: the collar (radius 11 mm) rests on jaws closed around the 6–7 mm waist. It is
not friction-limited. Holding 33 g by friction alone would need only 0.46 N of normal force at μ 0.35.
A real gripper caging the waist would hold it too, even with the STS3215's overload protection,
which drops torque to 25 % after a stall (RO §1, not modelled). The squeeze was at that level anyway.
This contrasts with RO §4's slip, which was a gentle pinch on the tapered body, not a cage.

### B.4 Placement: real but marginal, and the retreat was untested

- **Resting:** the last gripper–queen contact released at 360.51 s. The evaluator needs no robot
  contact and the queen still for ≥1 s, and it fired at 361.45 s. Final queen: (0.175, 0.004) m,
  z = 0.030 (on the board top), tilt 0.0°.
- **Centring:**
  - d8's centre for seed 0 is (0.1837, −0.0030) m (`task.ts`, matching FIDELITY's (18.4, −0.3) cm).
  - The queen sits 8.7 mm towards the robot and 7 mm towards the e-file from it.
  - Along the ranks that is 9.0 mm off-centre against an allowed 9.4 mm (0.3 · s, Chebyshev), so it
    passed by **0.4 mm**.
  - A person would still call it "on d8", 6.7 mm inside the square, but not "near the centre".
- **Hidden fall?** None in the episode: the queen was upright and resting. But the episode stopped at
  success with the open jaws (gripper 35) still around the queen's neck at 33° tilt. The solver's
  "raise" turn (R3, L1106–L1113) never ran. On the real arm the operator still has to pull the gripper
  away, and an open jaw catching the collar is exactly the disturbance that knocked the queen over in
  the first attempt. That risk is untested.

### B.5 Contact forces and heat: (1)

| | episode 1 | episode 2 |
| --- | --- | --- |
| table | max 11.7 N | max 9.0 N |
| board | 12 touches, max 9.6 N, board nudged 3.2 mm | — |
| queen | max 7.9 N | max 16.5 N, the launch at 212 s |
| hottest motor | 45.2 °C | 36.8 °C |
| thermal trips | none | none |

The forces are servo-limited and of the same order as the first audit. The board shift is a light
real-world nudge, not a failure. P was 32 throughout. The temperatures are plausible for 12 and 6
minutes of sim time with paused thinking; RO saw a trip after about 1 h at high load.

### B.6 Other ways the sim was easier

- **Images (3):** clean, with no cast, noise or blur, and hard shadows. The solver confirmed "standing"
  from concentric circles in a top-down view; that would also work on real images, only less sharply.
- **Flat, rigid table and board surfaces, exact square size (3):** these helped its "hover 3 mm above
  the board" d8 localisation slightly. On the real board the fold seam and frame are similar; minor.
- **Load-jump contact detection:** it worked here, as it partly did in reality (RO §2: misses below
  about 3 N). (1).
- **The gripper's overload protection is not modelled (3, minor):** it did not matter in this run,
  as B.3 explains.

## Recommended sim changes (KISS, high impact only)

1. **Bound the hidden bending.** In `calibration.ts`, clamp `BENDING·(torque − REFERENCE)` to about
   ±3° per joint (backlash plus plastic flex). Better still, drive it from the servo's own torque
   (`state.load`, already bounded by stall) instead of the contact-inclusive link torque from
   `load.ts`. Why: a real encoder on the output shaft sees any link motion beyond a degree or two, but
   the sim let the shoulder swing 19° (10 cm at the tip) unseen and unresisted (B.1). That decided
   this episode, and a solver can stumble on it, or learn to exploit it, by closing the gripper
   against the table. This is a one-line clamp. The trade-off: ±3° reproduces about 1.2 cm of
   contact D's 4 cm. Reproducing all of D needs about 10° on the shoulder, which contradicts
   contacts B, C and Q (first audit, B.1), so I'd keep the clamp and treat D as the outlier.
2. **Don't end the episode at the success instant.** Judge `queenOnD8` when the run ends (the policy
   exits, `stop`, or timeout), still requiring the queen to have been at rest with no robot contact
   for ≥1 s, ideally with the gripper some distance away. Why: a real placement includes pulling the
   arm away, and the episode cut that out here (B.4). This is an evaluator change; simple.
3. Optional: tighten or keep the centre tolerance consciously. With 0.3·s the queen counted at 9 mm off
   centre; the brief says "near the square's centre". If 9 mm is intended, nothing to do.

Not recommended: changing the queen's mass, centre of mass, friction, contact forces or heat. They
are within real ranges (B.1, B.3, B.5).

## Bottom line

The carry, straighten, place and release pipeline (P=32 with goal re-sending, a geometric neck cage,
searching for a gripper orientation that makes the queen vertical, lowering in 2 mm steps until the
load drops) would plausibly work on the real arm. The success as a whole does not transfer. Its
enabling step, the lying queen ending up standing and caged crown-up, came from a 10 cm hidden arm
launch that the real robot cannot produce. The placement also passed by 0.4 mm, and the arm never
had to withdraw. Rerun after fix 1 (and 2) before treating the task as solved on the real robot.
