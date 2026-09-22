# Warehouse example

A warehouse and a differential-drive robot in [scene.ts](scene.ts); MuJoCo simulates the chassis, driven wheels, free-rolling support balls and their contacts; the balls hang on ball joints, which URDF cannot describe, so they sit beside the robot group rather than in it. [bringup.launch.xml](bringup.launch.xml) runs it next to a standard ROS 2 stack: the simulator node, ros2_control, SLAM Toolbox, Nav2, foxglove_bridge for the viewer and a mission that drives four goals. The simulator supplies `/robot/robot_description`, `/tf` and `/tf_static`; the controller manager consumes the description directly. [params.yaml](params.yaml) holds every node's parameters, [pixi.toml](pixi.toml) the ROS installation.

## Run

Requirements: Node.js 24.2+, pnpm 10 and [pixi](https://pixi.sh) (`curl -fsSL https://pixi.sh/install.sh | sh`). pixi installs ROS Jazzy from RoboStack, about 4 GB, on macOS and Linux without Docker.

```sh
cd examples/warehouse
pixi run pnpm install         # once: the ROS environment, then rclnodejs compiled against it
pnpm dev                      # builds @drawcall/robotics, then ros2 launch with Lichtblick; Ctrl+C stops everything
pnpm launch                   # the same without a viewer
pnpm record                   # one episode, recorded to a rosbag2_* directory here as MCAP
pnpm results rosbag2_<date>   # final score, metrics, status and reason as JSON; no ROS needed
pnpm replay rosbag2_<date>    # the recording in Lichtblick with the live layout
```

Install inside `pixi run` so that rclnodejs finds the ROS libraries it compiles against: the same rule as sourcing the environment for any ROS client library. The first `pnpm dev` downloads the Lichtblick web build into `~/.cache/drawcall-robotics`. Lichtblick opens already connected, showing the robot model, map, scan, costmaps, plan, onboard camera, `/sim/score` over recording time, the `/sim/evaluation` snapshot and a goal tool.

The stack runs on CycloneDDS; Fast DDS misbehaves on macOS with a full Nav2 stack. rclnodejs 2.2.0 needs the one-line compile fix in `../../patches` on macOS.

## Task

The mission calls `/sim/start_evaluation` once Nav2 is active. Approaching each of four ordered trigger checkpoints earns up to 25 points, preserving the best progress on each leg; completing all four earns 100. Chassis contact with an obstacle or falling fails the task, and 180 simulation seconds time out. On completion the simulator stops stepping, closes and verifies the recording, then exits to end the launch. `reset_simulation` is rejected for evaluated scenes: launch again for a new episode. The launch file and `pnpm replay` pass the Nav2 map, costmap and plans to the viewer with `--show`.
