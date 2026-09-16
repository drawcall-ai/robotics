# Warehouse example

A warehouse and a differential-drive robot in [scene.ts](scene.ts), driven by a standard ROS 2 stack: [bringup.launch.xml](bringup.launch.xml) starts the simulator node, robot_state_publisher with the URDF exported from the scene, ros2_control, SLAM Toolbox, Nav2 and a mission that drives four goals and ends the launch. [params.yaml](params.yaml) holds every node's parameters, [pixi.toml](pixi.toml) the ROS installation.

```sh
curl -fsSL https://pixi.sh/install.sh | sh   # once: pixi installs ROS from RoboStack
pixi run pnpm install                         # once: the ROS environment and rclnodejs built against it
pnpm dev                                      # builds @drawcall/robotics, then pixi run launch viewer:=true
```

`pnpm record` runs an episode while `ros2 bag record` writes every topic but Nav2's internals to a `rosbag2_*` directory here; `pnpm replay rosbag2_<date>` opens it in Lichtblick with the same layout. Lichtblick opens connected to the running stack. While it runs, `ros2 service call /sim/pause std_srvs/srv/Trigger` pauses the whole stack, `/sim/step` advances one physics substep, and `ros2 topic echo /sim/state` shows ground truth.

The stack runs on CycloneDDS; Fast DDS misbehaves on macOS with a full Nav2 stack. rclnodejs 2.2.0 needs the one-line compile fix in `../../patches` on macOS.
