# Drawcall Robotics

Connect a Three.js physics scene to ROS 2. Declare a robot with cameras, lidar and motors as ordinary objects, export it as URDF from the same model, and run it as a native ROS 2 node next to a standard ros2_control, SLAM Toolbox and Nav2 stack. The simulator is a process in your launch file, the way Gazebo or Webots would be.

| Directory                                          | Purpose                                                                                 |
| -------------------------------------------------- | --------------------------------------------------------------------------------------- |
| [packages/robotics](packages/robotics/README.md)   | `@drawcall/robotics`: devices, URDF export, the `drawcall-robotics` simulator CLI       |
| [examples/warehouse](examples/warehouse/README.md) | A warehouse scene and differential-drive robot with its ROS Jazzy stack through pixi    |
| [examples/so101](examples/so101/README.md)         | Randomized desk cleanup with an SO101 wrist camera, movable bowl and in-scene evaluator |

The workspace uses `@drawcall/physics` and its Rapier backend **0.6.0**. `JointDrive` belongs to physics; robotics extends it as `JointActuator`, the drive ros2_control commands.

## Run the warehouse

Requirements: Node.js 24.2+, pnpm 10 and [pixi](https://pixi.sh) (`curl -fsSL https://pixi.sh/install.sh | sh`). pixi installs ROS Jazzy from RoboStack, about 4 GB, on macOS and Linux without Docker.

```sh
cd examples/warehouse
pixi run pnpm install   # once: the ROS environment, then rclnodejs compiled against it
pnpm dev                # ros2 launch with the simulator and Lichtblick; Ctrl+C stops everything
```

Install inside `pixi run` so that rclnodejs finds the ROS libraries it compiles against. That is the same rule as for any ROS client library: source the environment first.

`pnpm dev` builds the package and runs `pixi run launch viewer:=true`; the `dev`, `launch`, `record` and `replay` scripts live in `examples/warehouse`. The XML launch file starts the simulator with `--viewer`, ros2_control, SLAM Toolbox, Nav2, foxglove_bridge and the mission. The simulator automatically publishes the robot description and internal link transforms; ros2_control subscribes to `/robot/robot_description`. The first run downloads the Lichtblick web build into `~/.cache/drawcall-robotics`. Lichtblick opens already connected, showing the robot model, map, scan, costmap, plan, the onboard camera, score versus recording time, evaluation status and a goal tool. The mission starts evaluation when Nav2 is ready and drives four goals. The scene evaluator scores ground-truth checkpoints and ends the episode. The runner closes and verifies the recording before exiting and ending the launch.

`pnpm launch` runs the same without a viewer. `pnpm record` asks the simulator to manage `ros2 bag record`, writing every topic to a `rosbag2_*` directory in `examples/warehouse` as MCAP, and `pnpm replay rosbag2_<date>` opens that recording in Lichtblick with a layout built from the sensors it holds.

While it runs, the simulator answers `/sim/pause`, `/sim/resume`, `/sim/step` and `/sim/reset` (`std_srvs/Trigger`) and publishes ground truth on `/sim/state`. Because every node runs on `use_sim_time`, pausing the simulator pauses the whole stack and one step advances it by exactly one physics substep. `drawcall-robotics snapshot /robot/camera/image/compressed --out frame.jpg` saves what a camera sees.

## API shape

A scene is a plain script. It builds a Three.js scene in the default physics world and puts a robot in it. A `Robot` is a group: the bodies, joints and sensors under it are the robot, and its URDF, joint states and joint commands follow from that structure.

```ts
import { CameraSensor, LaserSensor, Robot } from "@drawcall/robotics";

const scene = new Scene();
const robot = new Robot();
robot.name = "warehouse";
scene.add(robot);
const chassis = new RigidBody({ mass: 12 }); // "base_link"; wheels, named joints, actuators follow
robot.add(chassis, leftWheel, leftAxle, rightWheel, rightAxle);

const laser = new LaserSensor({
  name: "scan",
  frameId: "laser",
  samples: 361,
  range: 12,
});
const camera = new CameraSensor({
  name: "camera",
  frameId: "camera_optical",
  rate: 5,
});
laser.position.set(0, 0.24, -0.32);
camera.position.set(0, 0.28, -0.38);
chassis.add(laser, camera);
```

The base is the body no joint hangs from, every named axis joint becomes a URDF joint, a joint driven by a `JointActuator` is commanded through ros2_control with the interface the actuator declares (position, velocity or effort), a plain `JointDrive` is a simulated spring, and each sensor mounted on the robot is a frame. `drawcall-robotics urdf scene.ts` runs the script against an offline runtime and prints that URDF; `drawcall-robotics sim scene.ts` runs it as a ROS 2 node through rclnodejs, rendering the cameras with Dawn. Scenes with several robots pick one with `--robot <name>`. In a browser the same devices attach to a rosbridge connection; see the [package guide](packages/robotics/README.md).

## ROS topics

Defaults under namespace `/robot`:

| Topic                                                                | Type                                          | Direction          |
| -------------------------------------------------------------------- | --------------------------------------------- | ------------------ |
| `/robot/joint_commands`                                              | `sensor_msgs/msg/JointState`                  | ros2_control → sim |
| `/robot/joint_states`                                                | `sensor_msgs/msg/JointState`                  | sim → ros2_control |
| `/robot/scan`                                                        | `sensor_msgs/msg/LaserScan`                   | sim → ROS          |
| `/robot/camera/image/compressed`, `/robot/camera/camera_info`        | `CompressedImage`, `CameraInfo`               | sim → ROS          |
| `/clock`                                                             | `rosgraph_msgs/msg/Clock`                     | sim → ROS          |
| `/sim/state`, `/sim/pause`, `/sim/resume`, `/sim/step`, `/sim/reset` | `std_msgs/msg/String`, `std_srvs/srv/Trigger` | sim → agents       |

ROS-internal topics follow upstream conventions: `/diff_drive_controller/cmd_vel` and `/odom`, `/joint_states` from the broadcaster, `/tf`, `/map`, `/plan`. Absolute names such as `/clock` bypass the namespace. Use one clock publisher per ROS graph.

Three.js forward `-Z`, left `-X`, up `+Y` map to ROS `+X`, `+Y`, `+Z`. Camera optical frames use `+Z` forward, `+X` right and `+Y` down. The simulator publishes `base_footprint → base_link → wheels, sensors` automatically on `/tf` and `/tf_static`. SLAM owns `map → odom`; the drive controller owns `odom → base_footprint`. Visual meshes and colors are embedded in the retained `/robot/robot_description` message, so new recordings replay the model without access to the original asset files.

## Timing and limits

Every ROS node runs on `use_sim_time`. Simulation time counts completed fixed substeps at 120 Hz and is published on `/clock`; there is no lockstep, and pausing the simulator pauses the clock and with it every node.

Joint commands are unstamped `JointState` messages republished at the controller manager's rate. Each actuator keeps the latest value for its joint and expires it after 0.5 seconds of simulation or wall time. **Expiry removes actuation; the robot can coast. It does not brake or hold position.**

Current scope:

- Topic-based ros2_control hardware, not a compiled hardware plugin or trajectory action server.
- Encoder odometry, SLAM Toolbox mapping and localization, Nav2 navigation; ground truth only on `/sim/state`.
- A configurable planar lidar, 360° by default, that sees everything including its own robot; no-hit rays and returns inside `minRange` use `range_max + 1`.
- Cameras render into targets and read back asynchronously; a sample is skipped while a readback is in flight. Defaults are 320×180 at 5 Hz.
- `/sim/reset` restarts the clock only in scenes without an evaluator. Evaluated scenes require a new launch, keeping one episode per recording and resetting ROS state together.

## Validation

`pnpm check` builds, typechecks, runs the package's unit tests, including the URDF export of the example, and checks formatting. The example is verified by running it: `pnpm launch` ends after the robot has completed its circuit.

## Evaluation and results

The scene declares one `Evaluator` using named success/failure predicates, numeric metrics and an absolute score callback. It uses the default ROS runtime and evaluates after each completed physics step and synchronous device update. Failure wins over simultaneous success; timeout is a separate status. Evaluation does not publish ground truth to the robot controller.

```ts
const evaluation = new Evaluator({
  timeout: 60,
  metrics: { checkpoints: () => reached },
  success: { reachedGoal: () => goal.overlaps(robot.base) },
  failure: { restrictedArea: () => restricted.overlaps(robot.base) },
  score: ({ status }) => (status === "success" ? 100 : 0),
});
```

The simulator starts evaluation automatically, or waits for `/sim/start` with `--wait-for-start`. The warehouse mission uses that service after Nav2 is active. Trigger overlap means intersection, not full containment or a precise docking pose. The warehouse counts base-link overlap with four ordered checkpoints, awarding up to 25 points per leg as the robot approaches each checkpoint; earned progress never decreases and entering the final trigger completes the 100-point score, and fails on chassis contact with an obstacle or falling. Its time limit is 180 simulation seconds.

`/sim/evaluation` carries a JSON snapshot in standard `std_msgs/msg/String`: a simulation timestamp, elapsed seconds, status, reason and named numeric metrics (including `score`). Complete snapshots are published initially, when metrics change at up to 10 Hz, and immediately on termination. Intermediate samples can be coalesced; compute cumulative counts or maxima in the task if they matter. Final values are frozen. The native publisher uses reliable, transient-local QoS; browser rosbridge uses a latched topic.

No custom ROS package or message build is needed. `/sim/score` mirrors the snapshot’s score as `std_msgs/msg/Float64` for standard plotting tools. `/sim/evaluation` remains the authoritative result; consumers parse its `data` field as JSON.

```sh
cd examples/warehouse
pnpm record
pnpm results rosbag2_<date>  # JSON; no ROS environment, viewer or simulation needed
pnpm replay rosbag2_<date>
```

One bag recording contains one episode, possibly split across MCAP files. `results` reads the complete bag directory or a single MCAP file, decodes its embedded ROS schemas, and returns the terminal status, reason and metrics. Missing results, unfinished recordings, backwards evaluation time and invalid metrics are errors, not zero scores. A task failure is a valid result; a crash before terminal publication leaves an incomplete recording.

The simulator owns its recorder: it waits for discovery, stops stepping at the terminal substep, waits for DDS acknowledgements, gracefully closes rosbag, and reads the resulting MCAP to verify the terminal snapshot before exiting. Startup and shutdown records may sit outside the evaluation interval. Interrupted active evaluations publish `aborted` when ROS remains available; simulation errors publish `error` when possible. Process death or ROS loss cannot guarantee final publication.

Live and replay layouts include a score plot on the shared recording timeline and the evaluation message. Replay currently opens one MCAP file; result extraction supports split recordings.
