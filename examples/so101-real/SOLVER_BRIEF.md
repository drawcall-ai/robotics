# Solver brief: stand the queen on d8

## The task

An SO-101 follower arm (LeRobot `so101_follower`, Feetech STS3215 servos, no leader arm) is
screwed to a table. A Raspberry Pi camera is mounted on its gripper, looking down along the jaws.
In front of the arm is a folding wooden chessboard; **rank 8 is the rank nearest the robot and the
a-file is on the robot's right**. A dark wooden chess queen lies on its side somewhere on the table
near the board.

**Goal:** the queen stands upright on d8, near the square's centre, released, the arm withdrawn
(a couple of centimetres clear, the jaws not around it), and it stays there. The episode succeeds once all of that has held for a second.

**The scene varies with the seed**, as it would between real sessions. Seed 0 is the real scene.
Other seeds move and turn the board (still in front of the robot, rank 8 nearest, a-file on its
right) and change its square size, frame, height and wood colours. The queen may stand or lie in
any direction, anywhere within reach on the table or the board (never already on d8). It comes in
somewhat different sizes and dark-to-medium wood tones. The light runs from neutral to the real
camera's strong red/pink cast. Develop a policy that works on any seed, not on seed 0.

**The opposite task**, for testing a policy repeatedly: with `TASK=scatter` (e.g.
`TASK=scatter pixi run drawcall-robotics run --seed 5 …`) the queen starts upright on d8. The
goal is the queen at rest a square or more away from d8's centre, on the board or the table, the
arm withdrawn, the board not moved. Running scatter and place in turn resets the scene, as an
operator would between real attempts.
Knocking the board off its place or the queen off the table fails the episode. The episode ends
after 30 simulated minutes.

This simulates one real scene and that robot as it behaves on hardware. Expect what the real arm
does: its readings are not exact joint angles, its servos sag under load and stick, the motors
heat up, and the camera is all you see. Develop as you would on the real arm.

## What you know (the operator's knowledge)

- The arm's kinematics: `so101_nominal.urdf`, the public SO-101 model, kinematics only. The
  robot's base_link is about 3 mm above the table. The CAD meshes in `assets/*.stl` are the public
  SO-101 parts.
- LeRobot's calibration: every arm reading is in degrees, 0 in the middle of the joint's calibrated
  range. The zeros were set by eye, so a reading of 0 is near, not at, the model's zero. The
  calibrated ranges, which the servos enforce on goals, are:

  | joint | range |
  | --- | --- |
  | shoulder_pan | ±101.6° |
  | shoulder_lift | ±95.4° |
  | elbow_flex | ±90.3° |
  | wrist_flex | ±85.9° |
  | wrist_roll | ±180° (a full turn; its zero is arbitrary) |
  | gripper | 0 (jaws touching) to 100 (fully open), linear in motor angle |

  Readings have the same signs as the URDF joint angles. The arm starts in its rest pose, folded so
  that it holds itself with the torque off: readings about (−6, −94, 89, 17, −6, 5).
- The gripper has a white fixed jaw and a black moving jaw.
- The camera: Raspberry Pi Camera V2.1, 1280×960, datasheet intrinsics fx = fy = 1061 px,
  cx = 640, cy = 480, no distortion. Its mounting on the gripper has not been calibrated.
- Servo facts: LeRobot sets P=16 (the factory default is 32), I=0. The gripper's torque is
  limited to half of the stall torque. STS3215 servos cut their torque when they overheat.

## The interface

Use the harness `robot.py` (read it; it is short). Everything is in LeRobot units:

```python
from robot import Robot
robot = Robot()                       # calls rclpy.init() itself
robot.joints                          # Present_Position: degrees, gripper 0..100
robot.velocities                      # Present_Velocity, per second
robot.loads                           # Present_Load, ±1000 = ±100 % of stall torque
robot.status                          # {"temperature": {...} °C, "torque": {...} on/off, "p": {...}}
robot.goal(shoulder_lift=10.0, gripper=60)   # Goal_Position; the servo holds it until the next goal
robot.p_gain(shoulder_lift=32)        # P_Coefficient, 1..64
robot.wait(1.0)                       # simulated seconds
robot.settle(timeout=5.0)             # wait until the joints stop moving, returns the readings
rgb = robot.image()                   # next wrist camera frame, 960x1280x3 RGB
robot.get_logger().info("...")        # logs go to /rosout, which `cat` shows
```

Goals persist: a servo holds its last goal, also between turns, and keeps heating while it holds
against a load. A servo that overheats cuts its torque (the arm drops) and stays off until it has
cooled down and receives a new goal.

The underlying topics, the only ones you may use: `/follower/joint_states` (readings, velocities,
loads), `/follower/status`, `/follower/goal`, `/follower/p_gain` (all `sensor_msgs/JointState` but
the status, a JSON `std_msgs/String`), `/robot/wrist_camera/image_raw` (and its `compressed` and
`camera_info`), `/clock` and your own `/rosout`.

## How to run

From this folder (see the `robot-task` skill for the full workflow):

```sh
pixi run drawcall-robotics run --seed 3 --timeout 30 -- python3 - <<'EOF'
from robot import Robot
robot = Robot()
robot.get_logger().info(f"{robot.joints}")
EOF
pixi run drawcall-robotics continue --run <folder> --timeout 30 -- python3 my_step.py
pixi run drawcall-robotics cat <folder> --topics /rosout
pixi run drawcall-robotics cat <folder> --topics /robot/wrist_camera/image_raw/compressed --start -1 | tail -1
pixi run drawcall-robotics stop --run <folder>
```

A turn ends when your script exits or after `--timeout` simulated seconds; the world then pauses
with the servos holding their goals. **A pause is not free:** a real arm keeps heating while its
operator thinks, so each pause charges its wall-clock duration, up to 120 s, as holding time to
the motors' temperatures. Time spent inside a turn counts in simulated seconds as usual.

Use `--seed 0` and other seeds: the scene varies a little between seeds, so do not hard-code
positions. Your final policy goes in `policy/` and is started by `policy.launch.xml`
(`pixi run drawcall-robotics run --seed N` without a script runs it).

## Rules

1. Decide only from what the real robot gives you: joint readings, velocities, loads, status and
   camera images.
2. Do not read the scene, task, environment, model, servo or calibration sources (`*.ts`,
   `pixi.toml` and `package.json` excepted), `FIDELITY.md`, `PACKAGE_CHANGES.md`, `evidence/`, or
   any `REAL_OBSERVATIONS.md`. They hold ground truth.
3. Do not subscribe to or `cat` any other topic. The simulator's truth (`/robot/joint_states`,
   `/tf`, `/tf_static`, `/robot/robot_description`, `/sim/*`) is not on your ROS graph, but it is
   recorded in the run's bag: do not `cat` it. You may read the final `status` and `reason` of
   `/sim/evaluation` (or the run's `result.json`) to learn whether an episode succeeded, nothing
   else in it.
4. No teleporting or other changes to the simulated world except through the servo goals and gains.
5. Edit only your own `policy/` folder and scratch files. Do not change the example, the robotics
   package, the skill or the configuration.
6. No models trained on this simulation; off-the-shelf models and classic vision are fine.

**Exception for diagnosis.** To root-cause a failure or a sim/real difference you may read the
simulator's recorded truth in the run's bag (`/robot/joint_states`, `/tf`, `/sim/audit`,
`/sim/evaluation` metrics) and `evidence/`. Use it only to understand what happened. It must never
become an input to your policy: no constants tuned from it and no reading it at run time. The
sources in rule 2 and `REAL_OBSERVATIONS.md` stay off limits.
