---
name: robot-task
description: Solve a simulated robot task in this repo (e.g. examples/so101) with the drawcall-robotics CLI, from exploring turn by turn to an autonomous policy.launch.xml evaluated on many seeds. Use when building, debugging or evaluating a policy or controller for a scene; not for changing the simulator or the robotics package itself.
---

# Solving a robot task

A previous attempt ran 9 days, 51 active hours and about 300 full episodes without finishing, mostly waiting on episodes. Everything below exists to keep each iteration short and each decision honest.

Run commands from the example folder inside its ROS env (`pixi run drawcall-robotics …`). `run` prints its run folder on stdout. `continue`, `stop` and `cat` default to `runs/latest`, which is whoever's run started last in that folder; when other runs may start alongside yours, pass yours: `continue --run <folder>`, `stop --run <folder>`, `cat <folder>`. When the user wants to watch, add `--viewer` to the single `run`: it opens Lichtblick, which stays live across every `continue` of that run.

## Workflow

**1. Explore turn by turn.** Start one run and drive the robot with small Python turns through the example's `robot.py` harness (`Robot`: `move`, `image`, `param`, and `joints`, the measured positions; read it for exact signatures):

```sh
pixi run drawcall-robotics run --seed 3 --timeout 30 -- python3 - <<'EOF'
from robot import Robot
try:
    robot = Robot()  # calls rclpy.init() itself
    robot.move()  # waits for joint states and holds the pose; robot.joints is empty until then
    robot.move(shoulder_lift=robot.joints["shoulder_lift"] - 0.1)
    robot.get_logger().info(f"joints: {robot.joints}")
except KeyboardInterrupt:  # the turn's end ejects the script
    pass
EOF
pixi run drawcall-robotics continue --run <folder> --timeout 30 -- python3 - <<'EOF'
...  # the next step; the world kept its state while paused
EOF
```

A turn ends when the script exits or after `--timeout` sim seconds; the sim then pauses with the arm holding its pose. Each turn costs about 4 s of wall time on top of its sim time (`run --timeout 5` ≈ 10 s, `continue --timeout 5` ≈ 6.5 s, `stop` 2–5 s), so do several small actions per turn rather than one. After each turn, inspect:

```sh
pixi run drawcall-robotics cat <folder>        # /sim/evaluation and /rosout as JSON lines
pixi run drawcall-robotics cat <folder> --topics /robot/wrist_camera/image_raw/compressed --start -1 | tail -1
```

`--start -1` means the last sim second. Image lines carry `image`, a file path: Read it and look. Understand the situation yourself before writing logic. A single run stays paused until you `continue` or `stop` it, also when its policy finished before the episode did; an idle paused run quits on its own after 30 min.

**2. Grow skills.** A turn script that works becomes a function with a contract: what it needs, what it guarantees, how it reports failure.

**3. Move decisions into code.** Each time you decide by looking ("a knight, lying"), copy that frame to `labels/<name>.jpg` with your answer in `labels/<name>.txt`. Then replace one decision type at a time with code or an off-the-shelf model, chosen by an explicit criterion, and score it offline against those labels. That is evaluation, not training: self-trained models are not allowed (sim-to-real gap).

**4. Assemble the policy** in `policy.launch.xml` until it solves the task with no help: `pixi run drawcall-robotics run`.

**5. Evaluate on many seeds**: fixed dev seeds plus fresh random ones.

```sh
pixi run drawcall-robotics run --seed 3,17,42,88 | pixi run drawcall-robotics cat
pixi run drawcall-robotics run --count 8
pixi run drawcall-robotics run --seed 17 --timeout 60   # rerun a failure alone, then step with continue --timeout
```

Each batch episode gets `runs/<stamp>/<seed>/` with `policy.log` (the policy's stdout and stderr) and `result.json`; stdout lists those folders. Group failures into classes and fix a class rather than a single seed: a fix shaped by one seed tends to break others.

## Rules

- **Decide only from what the robot senses: camera and joints.** Ground truth and evaluator metrics are for checking; a policy that decides from them learns what it can never see on the real robot.
- **Prefer the rclpy logger to `print()`.** Only /rosout reaches the bag and `cat`; `print()` lands only in the terminal (single runs) or `policy.log` (batches).
- **Use `run`, `continue`, `cat` and `stop`; no hand-rolled batch scripts, watchdogs or sleep-polling.** A hand-rolled watchdog once cost 5 h. `stop` ends the run and closes its bag. Only if `stop` fails, end the sim yourself: its pid is in `<folder>/state.json`; kill only that process group (`kill -- -<pid>`), never other runs.
- **If the same problem needs a third batch, build a faster reproduction first:** a shorter scene, `--timeout` turns from a paused state, or saved frames.
- **Report iteration latency in status updates** (wall time per turn or batch), so slow loops get noticed.
- **Keep `--real-time-factor` at 1 for evaluations.** Other factors change how much sim time the policy's computation takes, so scores stop meaning what they measure.
- **Commands expire after 0.5 s; never block the publishing thread.** Run slow work (vision, planning) in another thread, or the arm goes limp mid-motion.
- Harness scripts that speed up iteration may stay in the example.
