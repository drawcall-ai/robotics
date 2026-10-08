# SO-101 real: the queen

A simulation of one real scene, built so that a policy developed here meets the same problems it
would meet on the real arm: an SO-101 follower with a wrist camera, a folding wooden chessboard
and a dark wooden queen lying on the table beside it. The task is to stand the queen upright on
d8.

```sh
pnpm install && pixi install
pixi run drawcall-robotics run --seed 0 --timeout 30 -- python3 policy/policy.py
```

- **For the solver:** read [SOLVER_BRIEF.md](SOLVER_BRIEF.md) only. It states the task, the
  interface (LeRobot units, `robot.py`) and the rules.
- **For the auditor and the scenario builder:** [FIDELITY.md](FIDELITY.md) maps the measured
  behaviour of the real robot to how the simulation reproduces it, with evidence in `evidence/`;
  [PACKAGE_CHANGES.md](PACKAGE_CHANGES.md) lists what the robotics package still lacks.

Seed 0 is the scene as measured; other seeds move the board (±5 mm, ±1°) and the queen (±1 cm,
±8°) within the measurements' uncertainty.

| File | What |
| --- | --- |
| `scene.ts` | Evaluator (queen upright on d8) and the audit trail on `/sim/audit` |
| `environment.ts` | Table, board, queen, lighting (ground truth) |
| `robot.ts`, `model.ts`, `gripper.ts`, `hulls.ts` | The SO-101 and its real wrist camera |
| `servo.ts` | The Feetech servo bus: LeRobot units, P-only control, stiction, load, heat |
| `calibration.ts` | The hidden zero errors and the camera mount (ground truth) |
| `robot.py` | The solver's Python harness |
| `so101_nominal.urdf` | The public SO-101 kinematics a real operator has |
| `evidence/` | Builder-only checks against the real photos and measurements |

The CAD assets and derived model data are Apache-2.0 ([assets/LICENSE](assets/LICENSE)); the chess
set is CC0 ([assets/chess/LICENSE.txt](assets/chess/LICENSE.txt)).
