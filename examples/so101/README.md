# SO101 chess

An SO101 follower arm with a wrist RGB camera sets up a chessboard's black pieces. The task is built up in steps, each a scene:

- [scene.ts](scene.ts), the default: **one piece lies on its side within reach, and the robot must stand it on its home square**, a knight facing the far side of the board.
- [upright.ts](upright.ts): one lying piece to stand up anywhere, a test of righting alone.
- [setup.ts](setup.ts): **all sixteen black pieces scattered within reach, some lying, to be set up on their squares.**

```sh
cd examples/so101
nvm use                  # Node 24 or newer, as .nvmrc says
pixi run pnpm install
npx drawcall market install      # textures and sky, into public/
pixi run pnpm --filter @drawcall/robotics rebuild rclnodejs
pnpm dev                 # simulator, ROS controller and viewer
pnpm launch              # without viewer
pnpm launch scene:=setup.ts       # another scene
pnpm launch real_time_factor:=0   # as fast as the host allows
pnpm episodes --count 12          # evaluate twelve seeds in parallel
pnpm record              # record one evaluated episode
pnpm results rosbag2_<date>
pnpm replay rosbag2_<date>
```

The controller needs a chess-piece detector the example does not ship and a segmentation model; see [Piece detector](#piece-detector) and [Segmenter](#segmenter). `SEED=<n>` repeats a scene and `PIECE=<kind>` chooses the piece.

The world every step shares lives in [environment.ts](environment.ts), the pieces in [pieces.ts](pieces.ts), texture and model loading in [assets.ts](assets.ts), board geometry in [task.ts](task.ts), robot construction in [robot.ts](robot.ts) from the link and joint data in [model.ts](model.ts) and the collision hulls in [hulls.ts](hulls.ts), contact pads in [gripper.ts](gripper.ts), and the controller in [controller.py](controller.py). The ROS environment is pinned by `pixi.lock`. No hardware is required. `Robot` publishes its visual model and link transforms for the live 3D panel and recordings.

## World

Every process start samples a new world, so the controller cannot be tuned to one:

- **Board:** 22–25 mm squares, a 4–16 mm frame, 6–18 mm thick, 0.3–0.8 kg, placed and turned (±6°) so that every square is in reach. Squares are inlaid wood, each with its own grain, or printed.
- **Desk:** market wood or plain laminate in several colours.
- **Pieces:** Poly Haven's CC0 Staunton set scaled to the board, with ±6% height and width, and tinted, glossy or matte wood finishes. Each collides as its own mesh, which the physics backend splits into convex parts, and a hidden weight in its base puts its centre of mass low, as in real weighted pieces.
- **Light:** image-based sky lighting plus a shadow-casting sun, with random direction and strength.

The textures come from `npx drawcall market` (`assetDependencies` in [package.json](package.json)). The chess set's license is in [assets/chess/LICENSE.txt](assets/chess/LICENSE.txt).

The in-scene evaluator owns the outcome. A piece is placed when it stands upright on a home square of its kind, centred, a knight facing rank 1, let go of and still for a second. In [scene.ts](scene.ts) that ends the episode with 100; [setup.ts](setup.ts) scores 100 × placed pieces / 16 and succeeds when all are placed. Moving the board or dropping a piece off the desk fails the episode; the time limits are 600 and 3600 simulation seconds. Results publish on `/sim/evaluation` and `/sim/score`.

## Robot and camera

Joint transforms, limits and visual meshes come from TheRobotStudio's [SO101 camera URDF](https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib_camera.urdf), transcribed once into [model.ts](model.ts). The URDF on `/robot/robot_description` is generated from the scene built on it (`drawcall-robotics urdf scene.ts` prints it), and the controller reads its kinematics from that topic. The [32×32 UVC wrist-camera mount](https://github.com/TheRobotStudio/SO-ARM100/tree/main/Optional/Wrist_Cam_Mount_32x32_UVC_Module) is attached to the rotating wrist. The camera is RGB only, 640×480, 70° vertical FOV, 15 Hz.

This is a contact simulation at MuJoCo's default 2 ms step. Friction constraints are ten times stiffer than penetration ones (MuJoCo's `impratio` with elliptic cones), the setting MuJoCo Menagerie grippers use. Joint servos apply capped torques (3 Nm, gripper current-limited to 0.2 Nm) and cannot exceed their rated speed. The pieces and the board stay dynamic throughout: there is no grasp attachment, teleportation or scripted object motion. The CAD assets and derived model and collision data are Apache-2.0; see [assets/LICENSE](assets/LICENSE).

## Controller boundary

The controller's permitted observations are:

- joint **positions** from `/robot/joint_states`;
- images from `/robot/wrist_camera/image_raw`;
- the robot's URDF from `/robot/robot_description`.

Camera calibration is known. It publishes only joint-position targets on `/robot/joint_commands`, and calls `/sim/start_evaluation` once. It does not read simulator state, object poses, scores or randomization, so the same inputs and outputs are available from a real SO101 with a wrist camera.

The policy is one script, [controller.py](controller.py). It looks at the scene, handles one piece, and looks again, until every piece is in place or none is left it can handle; then it exits, which ends the episode.

1. **Survey.** From five views over the board, the board is measured in [vision.py](vision.py), once.
   - Its top is the plane on which the views agree.
   - The median of the views, resampled onto that plane, is the empty board.
   - The 8×8 checker pattern in that image gives the square size, centre and heading.
   - Eight more views cover the desk beside the board.
   - Standing pieces come from the piece detector; lying ones from segmenting the views, as outlines longer than a piece is wide.
2. **Candidates.** Sightings from different views are grouped by where their rays point. Each group is placed where its rays meet, which is least-squares triangulation, and gives the piece's middle. Candidates the arm cannot reach, on filled squares, or where handling failed twice, are skipped.
3. **From above.** The camera looks straight down at a candidate, and the segmenter, prompted densely around where it is expected, outlines it: lying or standing, where its base is, and which way a knight's head leans.
4. **Standing it up.** A lying piece is picked up by its middle and laid out pointing away from the robot, base inwards, clear of the board; then gripped near its head and turned up about its base's edge as far as the wrist pitches, from where the weighted piece falls upright.
5. **Close looks.** A standing piece is looked at from closer, around the robot's side of it. Every close sighting refines the estimate, the side views vote on its kind among those with a free home square, and a candidate no close look confirms is passed over.
6. **Grasp and place.**
   - The whole path is planned before the arm moves, clear of the board and the pieces on it.
   - The pads close centred on the piece's middle height, so no height or shape model is needed.
   - A jaw that stops short of its closed limit is holding the piece.
   - It is set on the nearest free home square of its kind, a knight turned to face rank 1.

Motion geometry lives in [motion.py](motion.py), and the arm chain and its IK in [kinematics.py](kinematics.py). The node streams eased joint trajectories at 50 Hz along straight tool lines and waits for the joints to settle before each look.

The controller also publishes what it saw, as an output only: close looks go out on `/robot/perception/image/compressed` with the piece estimate marked. The launch file shows it in the viewer with `--show`, but not in the 3D panel, where an estimate could be mistaken for ground truth.

Set `SO101_DEBUG_CAMERAS=1` to save the wrist image and joint positions of every look to `stage_*` files, and the robot description to `stage_robot.urdf`.

## Piece detector

Pieces are found by a pretrained, off-the-shelf chess-piece detector rather than one trained on this simulation, so that nothing is fitted to rendered pieces. [detector.py](detector.py) runs any YOLOv8-style ONNX export with OpenCV's DNN module; set `SO101_DETECTOR` to its file (default `models/pieces.onnx`, which Git ignores). It takes a 736×960 letterboxed RGB image and scores twelve classes, a colour and a kind each, in the order bishop, king, knight, pawn, queen, rook, black then white. The colour is ignored.

In an evaluation of the public chess detectors on this scene's renders, only [KanisornPutta/chess-model-yolov8m](https://huggingface.co/KanisornPutta/chess-model-yolov8m) found standing pieces reliably:

- it found 97 of 103 standing pieces and named 94% of those correctly;
- it could not reliably name lying pieces;
- it takes about 0.3 s per image on an M2 Pro, and an estimated 3–5 s on a Raspberry Pi 4;
- export it with `yolo export model=<weights>.pt format=onnx imgsz=736,960 opset=12 simplify=True`.

Its upload is tagged MIT, but the weights come from an unlicensed repository and were trained with Ultralytics, which claims AGPL for YOLO weights. That is why the example does not ship the model; check the license before using it.

## Segmenter

Lying pieces, and outlines from above, come from [MobileSAM](https://github.com/ChaoningZhang/MobileSAM) (Apache-2.0), a pretrained promptable segmentation model: [segment.py](segment.py) prompts it on a grid of points and keeps each object once. It runs on ONNX Runtime from the image encoder and single-mask decoder that [Acly/MobileSAM](https://huggingface.co/Acly/MobileSAM) publishes (MIT); put `mobile_sam_image_encoder.onnx` and `sam_mask_decoder_single.onnx` in `models/` or set `SO101_SEGMENTER` to their folder.

Inference threads follow `OMP_NUM_THREADS`, which `episodes` sets to each episode's share of the cores. The launch file runs the controller at lower priority than the simulator, as the world never waits for a robot's computation: an episode whose simulation falls behind real time is run again rather than scored.
