# Warehouse example

A warehouse and a differential-drive robot in [scene.ts](scene.ts), driven by a standard ROS 2 stack: [bringup.launch.xml](bringup.launch.xml) starts the simulator node, ros2_control, SLAM Toolbox, Nav2 and a mission that drives four goals and a ground-truth evaluator that ends the launch. The simulator supplies `/robot/robot_description`, `/tf` and `/tf_static`; the controller manager consumes the description directly. [params.yaml](params.yaml) holds every node's parameters, [pixi.toml](pixi.toml) the ROS installation.

```sh
curl -fsSL https://pixi.sh/install.sh | sh   # once: pixi installs ROS from RoboStack
pixi run pnpm install                         # once: the ROS environment and rclnodejs built against it
pnpm dev                                      # builds @drawcall/robotics, then pixi run launch viewer:=true
```

`pnpm record` runs an episode while `ros2 bag record` writes the simulation topics, excluding the clock and noisy Nav2 internals to a `rosbag2_*` directory here; `pnpm replay rosbag2_<date>` opens it in Lichtblick with the same layout, including score versus recording time and terminal status. Lichtblick opens connected to the running stack. While it runs, `ros2 service call /sim/pause std_srvs/srv/Trigger` pauses the whole stack, `/sim/step` advances one physics substep, and `ros2 topic echo /sim/state` shows ground truth.

`pnpm results rosbag2_<date>` prints the final score, metrics, status and reason as JSON without ROS running. Approaching each of four ordered trigger checkpoints earns up to 25 points, preserving the best progress on each leg; completing all four earns 100. Only the onboard camera is recorded. Chassis contact with obstacles or falling fails the task, and 180 simulation seconds times out. The mission calls `/sim/start` after Nav2 is active. The simulator stops stepping on completion, closes and verifies the bag, then exits to end the launch. `/sim/reset` is rejected for evaluated scenes: launch again for a new episode.

`pixi run launch` runs ROS directly, without a custom message build. The viewer plots `/sim/score.data`; `/sim/evaluation` contains the complete JSON snapshot in its `data` field.

The stack runs on CycloneDDS; Fast DDS misbehaves on macOS with a full Nav2 stack. rclnodejs 2.2.0 needs the one-line compile fix in `../../patches` on macOS.
