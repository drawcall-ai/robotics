# SO101 desk cleanup

An SO101 follower arm with a wrist RGB camera picks three colored cubes off a desk and places them in a movable blue bowl. Each process start samples fresh bowl/object positions and yaw angles. Rejection sampling keeps their footprints separated by at least 18 mm and centers within a 30 cm reach radius; all objects start outside the bowl.

```sh
cd examples/so101
pixi run pnpm install
pixi run pnpm --filter @drawcall/robotics rebuild rclnodejs
pnpm dev                 # simulator, ROS controller and viewer
pnpm launch              # without viewer
pnpm record              # record one evaluated episode
pnpm results rosbag2_<date>
pnpm replay rosbag2_<date>
```

The scene lives in [scene.ts](scene.ts), robot construction in [robot.ts](robot.ts), contact pads in [gripper.ts](gripper.ts), placement/containment geometry in [task.ts](task.ts), and the controller in [ros/controller.py](ros/controller.py). The ROS environment is pinned by `pixi.lock`. No hardware is required. `Robot` automatically publishes its visual model and link transforms for the live 3D panel and new recordings; no `robot_state_publisher` launch entry is needed.

The in-scene evaluator owns the task outcome. All eight corners of every cube must lie inside the bowl, below its rim and above its floor. All three cubes and the bowl must settle for 1.5 seconds, with no cube touching the robot. Moving the bowl more than **2 cm**, rotating it more than **10°** from its initial orientation, or dropping a cube off the desk fails the episode. Peak displacement and rotation are retained, so moving the bowl back does not erase a violation. Timeout is 240 simulation seconds. Success scores 100; partial credit reflects current containment and bowl displacement. Results publish on `/sim/evaluation` and `/sim/score` for the viewer and recording.

## Robot and camera

Joint transforms, limits and visual meshes come from TheRobotStudio's [SO101 camera URDF](https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib_camera.urdf), included in `assets/so101.urdf`. The optional [32×32 UVC wrist-camera mount](https://github.com/TheRobotStudio/SO-ARM100/tree/main/Optional/Wrist_Cam_Mount_32x32_UVC_Module) is attached to the rotating wrist. The camera is RGB only, 640×480, 70° vertical FOV, 15 Hz; these are simulation calibration settings, not a claim that every real SO101 uses the same lens. Its optical center is at the housing's lens tip, local `(0, 0, 0.027)` metres, looking along housing +Z.

This is a contact simulation, with simplified convex collision shapes, separate jaw-pad boxes to keep the grasp cavity open, and a bowl built from box walls over an octagonal box floor. Friction constraints are ten times stiffer than penetration ones (MuJoCo's `impratio` with elliptic cones), the setting MuJoCo Menagerie grippers use; the grasp also holds at the default, with a few millimetres more creep. Fixed camera parts share the wrist rigid body; the optional mount collides with the environment but not overlapping printed arm parts. Joint servos apply capped torques (3 Nm, gripper current-limited to 0.2 Nm) and cannot exceed their rated speed. Objects and the bowl remain dynamic throughout: there is no grasp attachment, teleportation, scripted object motion, or fixed bowl. The CAD assets and derived model/collision data are Apache-2.0; see [assets/LICENSE](assets/LICENSE).

## Controller boundary

The controller was delegated only after the environment and ROS template were built. Its permitted observations are joint **positions** from `/robot/joint_states` and JPEG images from `/robot/wrist_camera/image/compressed`. Static robot geometry, camera calibration and task dimensions are known. It publishes only joint-position targets on `/robot/joint_commands`; `/sim/start` is used once to start evaluation. It does not read simulator state, object poses, transforms, scores, evaluator output, or randomization state. The same inputs and outputs are available from a real SO101 with a wrist camera, so the policy is written to transfer: nothing in it depends on the simulator.

The policy is one script, [ros/controller.py](ros/controller.py): from the home pose it turns the wrist camera across the desk and locates the bowl and the three cubes by colour, intersecting pixel rays with the known cube-top and bowl heights. It looks at the bowl once more from straight above it, then for each cube hovers 7 cm over it, measures its position and yaw again at close range, and chooses among the four wrist rolls that square the jaws to a cube face the one whose low gripper corners stay clearest of the bowl wall and the other cubes. It descends with open jaws, closes, lifts, and reads the jaw angle: a jaw that stops short of its limit is holding the cube. It carries the cube 12 cm up, over the rim, and lets it go 7.5 cm above the desk on one of three spots around the bowl centre, so the cubes land beside each other rather than in a tower. The node only plays that script one action at a time, streaming eased joint trajectories at 50 Hz along straight tool lines and waiting for the joints to settle before each look. The geometry lives in [ros/motion.py](ros/motion.py), colour segmentation in [ros/vision.py](ros/vision.py) and the SO101 transforms and IK in [ros/kinematics.py](ros/kinematics.py).

The controller also publishes what it saw, as an output only: every look goes out again on `/robot/perception/image/compressed` with the detected blobs outlined and labelled with their estimated desk positions and yaws. It appears as its own image panel in the live viewer and in recordings, next to the raw wrist camera, and deliberately not in the 3D panel, where an estimate could be mistaken for ground truth the robot is not given.

The source separation is an auditable development constraint, not a DDS security sandbox: simulator diagnostic topics remain available to human observers. Controller changes belong in `ros/`; scene/evaluator changes must be reviewed separately. Set `SO101_DEBUG_CAMERAS=1` to save the wrist image and joint positions of every look to `ros/stage_*` files. Diagnostics are off by default.

```sh
pnpm test
pnpm typecheck
pixi run python3 -m unittest discover -s ros -p 'test_*.py'
```

## Recorded validation

On 2026-09-20, on physics 0.8.0 with the friction impedance ratio at 10 and the perception image published, the controller completed a fresh randomized episode in **74.56 simulation seconds**, scoring **100**, with all three cubes settled inside the bowl. Peak bowl translation was **0.0002 mm** and rotation **0.00004°**. Four earlier fresh layouts that day also scored 100 (`rosbag2_2026-09-19T23-58-15.899Z`, `rosbag2_2026-09-20T00-00-05.683Z`, `rosbag2_2026-09-20T00-01-44.746Z` at ratio 50, `rosbag2_2026-09-20T10-02-02.660Z` at ratio 10). This validates five complete episodes, not every possible random layout. The local recording is `rosbag2_2026-09-20T10-10-32.074Z` (recordings are ignored by Git); inspect it with `pnpm results rosbag2_2026-09-20T10-10-32.074Z` or replay it with `pnpm replay` and that path, which shows the wrist camera, the controller's annotated looks and the score.

The same script was also exercised offline against ground-truth layouts in an in-process world (no camera) for five random seeds, placing all three cubes without moving the bowl in each. Before the bowl floor became an octagon of boxes ([scene.ts](scene.ts)), a cube landing on an edge kicked the bowl by centimetres in both the offline and the recorded episodes: MuJoCo resolves a box against a cylinder through one contact point. [bowl.test.ts](bowl.test.ts) drops tilted cubes onto the floor to keep that from returning.
