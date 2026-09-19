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

This is a contact simulation, with simplified convex collision shapes and separate jaw-pad boxes to keep the grasp cavity open. Fixed camera parts share the wrist rigid body; the optional mount collides with the environment but not overlapping printed arm parts. Joint servos apply capped torques (3 Nm, gripper limited to 0.05 Nm). Objects and the bowl remain dynamic throughout: there is no grasp attachment, teleportation, scripted object motion, or fixed bowl. The CAD assets and derived model/collision data are Apache-2.0; see [assets/LICENSE](assets/LICENSE).

## Controller boundary

The controller was delegated only after the environment and ROS template were built. Its permitted observations are joint **positions** from `/robot/joint_states` and JPEG images from `/robot/wrist_camera/image/compressed`. Static robot geometry, camera calibration and task dimensions are known. It publishes only joint-position targets on `/robot/joint_commands`; `/sim/start` is used once to start evaluation. It does not read simulator state, object poses, transforms, scores, evaluator output, or randomization state. The ROS policy uses color segmentation, camera-ray projection, forward/inverse kinematics and physical close/lift/release actions. It checks finger clearance along transfer paths against the image-estimated bowl and uses wrist images to confirm a grasp before transferring. It is a handwritten vision baseline: live trials have included both successful completion and failed grasps/bowl movement, so the evaluator result is authoritative.

Before closing, the policy fits the known 22 mm top face in the wrist image to correct the cube's position relative to the gripper without assuming tabletop height. A horizontal face also establishes camera-relative desk clearance for the entire jaw sweep. It tries equivalent cube-face wrist orientations when the preferred grasp would approach the bowl too closely. Initial surveying assumes the known colors and tabletop height; collision planning approximates the fingers and their supports, rather than checking every arm link.

The source separation is an auditable development constraint, not a DDS security sandbox: simulator diagnostic topics remain available to human observers. Controller changes belong in `ros/`; scene/evaluator changes must be reviewed separately. Set `SO101_DEBUG_CAMERAS=1` to write sensor-only debug captures to `ros/wrist_latest.jpg`, `ros/joints_latest.txt`, and `ros/stage_*` files. Diagnostics are off by default.

```sh
pnpm test
pnpm typecheck
pixi run python3 -m unittest discover -s ros -p 'test_*.py'
```

## Recorded validation

On 2026-09-18, the final controller completed a fresh randomized episode in **176.72 simulation seconds**, scoring **100**, with all three cubes settled inside the bowl. Peak bowl translation was **0.386 mm** and rotation **0.0153°**. This validates one complete episode, not every possible random layout. The local recording is `rosbag2_2026-09-18T17-22-21.623Z` (recordings are ignored by Git); inspect it with `pnpm results rosbag2_2026-09-18T17-22-21.623Z` or replay it with `pnpm replay` and that path.

The earlier successful recording above predates automatic model/TF publication and contains only sensor/evaluation playback. A recording with 3D robot playback is `rosbag2_2026-09-18T18-28-27.632Z`; this visualization-validation episode timed out rather than completing the task. The 3D panel shows the robot; the desk, bowl and cubes remain visible through the wrist camera.
