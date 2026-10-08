# Changes the robotics package needs for so101-real

Everything else of the real robot's behaviour lives in the example (`servo.ts`, `calibration.ts`,
`scene.ts`). Two changes are needed for an honest evaluation. Two more are optional.

## 1. Keep simulator truth off the policy's ROS graph (required)

**Why.** The example hides the true joint angles behind LeRobot readings with a zero error
(REAL_OBSERVATIONS §1, §5), but the package still publishes the truth where any policy can
subscribe: `Robot` sends `/robot/joint_states` (true radians), `/tf`, `/tf_static` and
`/robot/robot_description` (with the true camera mount); `Evaluator` sends `/sim/evaluation` with
metrics; the scene's audit trail `/sim/audit` is on the same domain. Today only the rules in
SOLVER_BRIEF.md keep the solver from reading them.

**API.**
- `ros.publish(name, type, message, { private: true })`: recorded to the bag, never sent to other
  ROS nodes.
- `new Robot({ private: true })`: its joint states, TF and description are published privately.
- `Evaluator({ private: true })` likewise for `/sim/evaluation` and `/sim/score` (the run's
  `result.json` still gets the final result).

**Acceptance.** In a `run` of so101-real, `ros2 topic list` from the policy's process shows only
`/follower/*`, `/robot/wrist_camera/*`, `/clock`, `/rosout` and `/parameter_events`; the bag still
contains `/robot/joint_states`, `/tf`, `/sim/audit` and `/sim/evaluation`, and `cat` shows them.

## 2. Contact forces (required for the audit)

**Why.** The auditor must judge how hard the arm hit the table, the board and the queen (§5:
"forces were low"; §6.4). `RigidBody` contact events carry no force, so the audit trail now
records only who touched what, for how long, and the servo loads meanwhile, which are a poor
proxy (§2: load misses contacts below about 3 N).

**API.** `world.contacts(body)`: the body's current contacts as `{ otherBody, point, normal,
force }`, with the normal force in newtons (MuJoCo `mj_contactForce`).

**Acceptance.**
- A queen standing at rest on the table reports a total normal force of m·g ± 5 %.
- Rerunning `evidence/contacts.py` (§5 poses B and C), every table touch in `/sim/audit` reports
  a peak force, and the touch at C with goals pushed into the table (`reached`) reports more than
  the plain one.

## 3. Joint friction (optional)

**Why.** The example emulates stiction by holding a joint's drive target while goal changes stay
within ±τs/K. That reproduces §2's "3° down: no motion" and the sudden drop, but it has no
kinetic friction. So a slow move lags by the full gravity sag (3–4° at P=16, 1.4–1.9° at P=32,
measured in sim) where the real arm lagged at most 1° (§2), and the settled load does not depend
on the approach direction as §2's 104…344 does.

**API.** `JointDriveOptions.frictionLoss` (Coulomb friction, N·m, MuJoCo joint `frictionloss`) and
`armature` (reflected rotor inertia, kg·m²).

**Acceptance.** With `frictionLoss: 0.42` and the dead band removed from `servo.ts`, in the §2
pose at P=16: a 3° downward goal step does not move the shoulder; a 4°/s descent lags at most
1.5°; settled shoulder loads after approaching from above and from below differ by at least
150 units.

## 4. A pause hook for devices (optional)

**Why.** Heat must accrue while the arm holds its pose between turns (a real operator's thinking
time). `servo.ts` now detects a pause from the wall-clock gap between two physics steps.

**API.** `Device.resumed?(wallSeconds: number)`, called once when a paused simulation resumes.

**Acceptance.** `servo.ts` charges the pause through the hook, with no `performance.now()`.
