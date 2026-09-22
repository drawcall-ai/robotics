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

The scene lives in [scene.ts](scene.ts), robot construction in [robot.ts](robot.ts) from the link and joint data in [model.ts](model.ts) and the collision hulls in [hulls.ts](hulls.ts), contact pads in [gripper.ts](gripper.ts), placement/containment geometry in [task.ts](task.ts), and the controller in [controller.py](controller.py). The ROS environment is pinned by `pixi.lock`. No hardware is required. `Robot` automatically publishes its visual model and link transforms for the live 3D panel and new recordings; no `robot_state_publisher` launch entry is needed.

The in-scene evaluator owns the task outcome. All eight corners of every cube must lie inside the bowl, below its rim and above its floor. All three cubes and the bowl must settle for 1.5 seconds, with no cube touching the robot. Moving the bowl more than **2 cm**, rotating it more than **10°** from its initial orientation, or dropping a cube off the desk fails the episode. Peak displacement and rotation are retained, so moving the bowl back does not erase a violation. Timeout is 240 simulation seconds. Success scores 100; partial credit reflects current containment and bowl displacement. Results publish on `/sim/evaluation` and `/sim/score` for the viewer and recording.

## Robot and camera

Joint transforms, limits and visual meshes come from TheRobotStudio's [SO101 camera URDF](https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib_camera.urdf), transcribed once into [model.ts](model.ts). That is the only copy: the URDF on `/robot/robot_description` is generated from the scene built on it (`drawcall-robotics urdf scene.ts` prints it), and the controller reads its kinematics from that topic. The optional [32×32 UVC wrist-camera mount](https://github.com/TheRobotStudio/SO-ARM100/tree/main/Optional/Wrist_Cam_Mount_32x32_UVC_Module) is attached to the rotating wrist. The camera is RGB only, 640×480, 70° vertical FOV, 15 Hz; these are simulation calibration settings, not a claim that every real SO101 uses the same lens. Its optical center is at the housing's lens tip, local `(0, 0, 0.027)` metres, looking along housing +Z.

This is a contact simulation, with simplified convex collision shapes, separate jaw-pad boxes to keep the grasp cavity open, and a bowl built from box walls over an octagonal box floor. Friction constraints are ten times stiffer than penetration ones (MuJoCo's `impratio` with elliptic cones), the setting MuJoCo Menagerie grippers use; the grasp also holds at the default, with a few millimetres more creep. Fixed camera parts share the wrist rigid body; the optional mount collides with the environment but not overlapping printed arm parts. Joint servos apply capped torques (3 Nm, gripper current-limited to 0.2 Nm) and cannot exceed their rated speed. Objects and the bowl remain dynamic throughout: there is no grasp attachment, teleportation, scripted object motion, or fixed bowl. The CAD assets and derived model/collision data are Apache-2.0; see [assets/LICENSE](assets/LICENSE).

## Controller boundary

The controller was delegated only after the environment and ROS template were built. Its permitted observations are joint **positions** from `/robot/joint_states`, JPEG images from `/robot/wrist_camera/image_raw/compressed` and the robot's URDF from `/robot/robot_description`, as a real robot's description would be. Camera calibration and task dimensions are known. It publishes only joint-position targets on `/robot/joint_commands`; `/sim/start_evaluation` is used once to start evaluation. It does not read simulator state, object poses, transforms, scores, evaluator output, or randomization state. The same inputs and outputs are available from a real SO101 with a wrist camera, so the policy is written to transfer: nothing in it depends on the simulator.

The policy is one script, [controller.py](controller.py): from the home pose it turns the wrist camera across the desk and locates the bowl and the three cubes by colour, intersecting pixel rays with the known cube-top and bowl heights. It looks at the bowl once more from straight above it, then for each cube hovers 7 cm over it, measures its position and yaw again at close range, and chooses among the four wrist rolls that square the jaws to a cube face the one whose low gripper corners stay clearest of the bowl wall and the other cubes. It descends with open jaws, closes, lifts, and reads the jaw angle: a jaw that stops short of its limit is holding the cube. It carries the cube 12 cm up, over the rim, and lets it go 7.5 cm above the desk on one of three spots around the bowl centre, so the cubes land beside each other rather than in a tower. The node only plays that script one action at a time, streaming eased joint trajectories at 50 Hz along straight tool lines and waiting for the joints to settle before each look. The geometry lives in [motion.py](motion.py), colour segmentation in [vision.py](vision.py) and the arm chain, read from the URDF, and its IK in [kinematics.py](kinematics.py).

The controller also publishes what it saw, as an output only: every look goes out again on `/robot/perception/image/compressed` with the detected blobs outlined and labelled with their estimated desk positions and yaws. The launch file passes it to the viewer with `--show`, so it appears as its own image panel live and in recordings, next to the raw wrist camera, and deliberately not in the 3D panel, where an estimate could be mistaken for ground truth the robot is not given.

The source separation is an auditable development constraint, not a DDS security sandbox: simulator diagnostic topics remain available to human observers. Controller changes belong in the Python files; scene/evaluator changes in the TypeScript files must be reviewed separately. Set `SO101_DEBUG_CAMERAS=1` to save the wrist image and joint positions of every look to `stage_*` files. Diagnostics are off by default.

## Recorded validation

On 2026-09-22, on physics 0.8.0, the controller completed a fresh randomized episode in **46.56 simulation seconds**, scoring **100** with all three cubes settled inside the bowl; peak bowl translation was **0.0002 mm** and rotation **0.00004°**. Five earlier layouts also scored 100. This validates complete episodes, not every possible random layout. The local recording is `rosbag2_2026-09-22T19-43-21.506Z` (recordings are ignored by Git); inspect it with `pnpm results` or replay it with `pnpm replay` and that path, which shows the wrist camera, the controller's annotated looks and the score.

The bowl floor collides as an octagon of boxes ([scene.ts](scene.ts)) because MuJoCo resolves a box against a cylinder through one contact point, which let a cube landing on an edge kick the bowl by centimetres.
