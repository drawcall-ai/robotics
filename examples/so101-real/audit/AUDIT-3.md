# Audit 3: the third solver's success on so101-real (2026-10-08)

**What was audited**
- Transcript: `tasks/af6391d55d71c9d86.output` (776 lines; "Lnnn" below are line numbers in it).
- Runs:
  - `runs/2026-10-08-085908`: episode 1, stopped after 1005 s of sim time.
  - `runs/2026-10-08-094548`: episode 2, `success / queenOnD8`, score 100, 737 s of sim time.
- Code: commit 27f539a.
  - The hidden bending is clamped to ±3° around the calibration poses.
  - Success needs the queen upright on d8, released, every gripper, jaw and camera collider at least
    2 cm away, and the queen at rest for 1 s. Release alone no longer ends the episode.

**Scripts**
- `hidden_bend.py --fitted`: the bend for runs from 27f539a on.
- `sim_audit.py`, `tool_calls.py` and `queen_mass.ts`: as before.

## Verdicts

| Question | Verdict |
| --- | --- |
| (A) Did the solver cheat? | **Clean.** One trivial slip: an IK printout was redirected to `/tmp/null` instead of `policy/`. |
| (B) Faithful success? | **Yes, with caveats that don't carry the result.** The stand-up was deliberate, and no uncommanded jump of the true joints occurred (the hidden bend changed by at most 3.7° within any 0.5 s). Contacts stayed gentle (≤ 8.6 N). The final placement is 4.7 mm off d8's centre, inside the 9.4 mm tolerance. The arm withdrew and the queen stood untouched for 16 s. |
| Loop stop condition | **Met**, for seed 0: the environment is faithful where it mattered, and the task was solved by a strategy that should also work on the real arm. See "Bottom line" for the residual differences. |

## (A) Cheating audit

The solver made 138 Bash calls and 64 Reads. It used no Write, Edit or background tools.

**Read tool:** 63 of its own frames, plus one image from `cat` of the allowed camera topic (L32).

**Bash:**
- L13: the brief, the skill, `ls`, `ls policy assets`, `git branch`.
- L20 and L24: `robot.py`, `policy.launch.xml`, `policy/policy.py`, the URDF.
- Otherwise only its own `policy/*` files.

A grep over all commands for `.ts`, FIDELITY, PACKAGE_CHANGES, README, `evidence`, `audit`,
`assets/chess`, `/sim/*` (apart from the two status-only reads below), `/tf`, `joint_states`, mcap, git
log or diff, `SO101_` and `--real-time` finds nothing.

**Topics:**
- `/rosout`, 68 times: allowed.
- The camera's compressed topic, once (L32): allowed.
- `/sim/evaluation` twice (L537, L760), each piped through `grep -o '"status"…'` / `'"reason"…'`.
  It saw status and reason only, which the brief allows.
- `result.json` once (L764), printing only `status | reason`.

**World-affecting commands:** only goals and P gains, plus `run --seed 0` twice and `stop`
twice. L342 read the output file of its own command that the shell had moved to the background;
that is a tool artefact, not world access.

**Scratch files:** all in `policy/`, except one stray file. At L140 `> /tmp/null 2>&1` redirected the
stdout of a throw-away IK printout to a file; it contains 10 joint poses as JSON. This breaks the
letter of the "scratch only in `policy/`" instruction but has no information effect.

**Grey areas the solver declared:**
- **The public-URDF jaw-tip offset:** public knowledge. OK.
- **Orienting the board by "a8 is light" plus the hinge:** that is chess knowledge and what the camera
  shows; the brief itself states the board convention. OK.
- **Learning of success when the episode ended mid-turn:** the harness's doing. OK.

## (B) Fidelity of what mattered for this success

### B.1 Hidden bending and uncommanded motion: no artefact

With `hidden_bend.py --fitted` (true angle − reading − fitted offset, shoulder_lift and elbow):
- The bend stays within ±3.1°.
- 17 % of samples sit at the clamp. That is expected: poses far from the calibration ones saturate
  to a constant ±3°.
- The fastest change within 0.5 s is 3.7° (lift, t = 75 s, a table touch).
- At the lift-off of the regrasped queen (t = 586 s) the bend changed by 2.5° on the lift and 2.9° on
  the elbow, and by 2.7° on the elbow at t = 711.6 s during the release.

Those amount to at most about 1–2 cm at the tip, bounded and comparable to backlash plus flex. There
is nothing like attempt 2's 19° swing. The solver's table-touch finding (tip 9 mm higher than its
model) matches RO §5 B and C (model within about 1 cm near the table). **(1) faithful.**

### B.2 The stand-up: deliberate, (1) faithful, with one sim simplification

Truth for episode 2, from `/sim/audit`:

| t (s) | what happened | queen tilt (true) |
| --- | --- | --- |
| 89–93 | gripper (load 477) closes across the neck of the lying queen; it lifts | 95° → 58° → 42° |
| 93–224 | carried with continuous gripper contact (258 s, peak 2.9 N); the solver thought the queen was held horizontal | 42–62° |
| sweep | the tool's pitch is swept 80° → 0° | down to 13–17° |
| 342 | lowered at x = 0.32 until the loads drop; released | 0.0°, standing on the board (z = 0.030) |
| 342–556 | the queen stands untouched | 0.0° |

That is the planned sequence, not luck.

**Simplification:** MuJoCo contacts here are `condim="3"` (physics-mujoco `contacts.js`), with no
torsional friction. A two-pad pinch on a round neck therefore lets the queen pivot freely about the
pinch axis, which is why it hung at 42–62° rather than staying horizontal.
- *Real pinch:* torsional friction is roughly μ·N·(contact patch, 1–2 mm), about 5–20 mN·m, against a
  gravity torque of about 6 mN·m. A real queen would pivot less, or not at all.
- *Why it doesn't decide the result:* the solver's plan assumed a rigid hold (grasp across the
  horizontal queen, then pitch the tool to horizontal, which makes the queen vertical). That works at
  least as well with a rigid grip. The free pivot only helped gravity pre-align the queen.
- **(3) easier, not load-bearing.**

### B.3 Neck regrasp, wedge tilt and re-orientation: (1) plausible

- **Regrasp (t ≈ 556–587 s):** from above at roll 0, pitch about 72°, stopping at gripper load 250.
  Lifting tilted the queen 21–23° in the grip. The solver estimated 25° from the image, a good
  estimate, and the wedge mechanism, jaw pads clamping a tapered neck, is real.
- **Re-orientation:** pitch 58°, roll 50°. During the carry the tilt fell 21° → 9–10°, then back up
  to 13–15° while lowering (the same pivot freedom as B.2).
- **Episode 1's failures are realistic:** a loose grip (15) let the queen slip through, and a backing-
  off moving jaw knocked it over (t = 930 s; tilt 41° → 95°). Both match RO §6.1 and §6.3.

### B.4 Placement, release, withdrawal and the final second: (1), margin 4.7 mm

- **t = 643–654 s:** the queen's base edge touches the board (z = 0.033) with a tilt of 14–22°.
- **t = 655 s:** it is released and settles upright at once (0.0°). It was tilted less than its 33°
  tip-over angle with the base edge already on the board, so a real queen would do the same.
- **t = 661–720 s:** the jaws back off, with the fixed jaw moved 12 mm sideways. Gripper contact
  continues for 47 s (peak 6.0 N). The queen rocks up to 10° four times and slides 7 mm, always
  recovering. On the real arm this is the riskiest phase; episode 1 lost the queen exactly here. The
  queen's behaviour is plausible for its centre of mass and tip-over angle (2.28 cm, 33°; AUDIT-2 B.1).
- **t = 720.4 s:** last contact. By 721 s the queen is at rest at (0.1832, 0.0017) m, tilt 0.0°, on the
  board top.
- **Margin:** d8's centre is (0.1837, −0.0030), so the offset is 0.5 mm along the ranks and 4.7 mm
  along the files. The tolerance is 9.4 mm, leaving **4.7 mm of margin**. A person would call it centred.
- **t = 737 s:** success, after the arm withdrew beyond 2 cm. The queen had been untouched for 16 s.
  Nothing was hidden.

### B.5 Contact forces and heat: (1)

| | episode 1 | episode 2 |
| --- | --- | --- |
| table, peak | 6.1 N | 4.5 N |
| board, peak | 1.2 N | — |
| queen, peak | 7.8 N | 8.6 N |
| board shift | under 0.05 mm | under 0.05 mm |
| shoulder temperature | 49 °C (1005 s, many pauses) | 37 °C |

These are the gentlest of the three attempts, and the board was never pushed. There were no trips;
the gripper held at about 48 % load for minutes and reached about 45 °C. Real STS3215 overload
protection (RO §1: Overload_Torque 25) would cut the gripper to 25 % after a stall. That still leaves
about 15 N of pinch against the 0.46 N needed to hold 33 g at μ = 0.35, so it would not change the
outcome. (4) not modelled, not load-bearing.

### B.6 What was easier than reality

- **Images (3):** no magenta cast, noise or blur, and hard shadows. The solver relied on a 20-view
  board calibration (2–5 px median, 12–15 px when extended), finial and base pixels for the tilt
  estimate, and top views to relocate the queen. All of these work on the real images (RO §3 got
  2 px from the same chessboard), with more effort.
- **Free pivot in a pinch (3, B.2):** helpful but not needed.
- **Perfectly flat, rigid table and board:** minor.
- **Only seed 0, one success in two episodes (4):** no robustness evidence beyond this scene. The
  real scene is seed 0.

## Recommended sim changes

None are required for the loop. Optional, cheap and honest:

1. **Torsional friction for the jaw pads.** Set `condim="4"` with a torsional coefficient of about
   0.001–0.002 m for jaw pairs. Why: a real pinch resists spinning about the pinch axis, and the free
   pivot is the one mechanism here that is easier than reality (B.2, B.3). How: a single attribute,
   if physics-mujoco exposes it.
2. **Image post-processing** (blur σ≈1 px, noise, a magenta tint). Why: perception is the
   part of this approach most likely to cost extra time on the real arm (B.6).

## Bottom line

This success came from a deliberate and physically sound strategy:
1. Calibrate the camera and board, and find the jaw height by touching the table.
2. Grasp the lying queen across its neck and pitch the tool to horizontal to stand it up.
3. Regrasp from above with a capped squeeze and measure the wedge tilt from the image.
4. Re-orient, lower until contact stops the arm, release, ease the jaws out, then withdraw.

No hidden-bending or other sim artefact carried it. The placement is centred, and the queen stood
free for 16 s. The remaining differences (clean images, free pivot in the pinch, no gripper overload
cut-back) make the real run somewhat slower and riskier, chiefly in the release and withdrawal. I
judge the approach **likely to work on the real robot, given a few retries** (episode 1's withdrawal
knock-over shows the real risk). The loop's stop condition (environment faithful plus solved) is met.
