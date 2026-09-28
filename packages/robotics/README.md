# @drawcall/robotics

ROS 2 simulation for Three.js and Drawcall physics: sensors and ros2_control bindings as scene objects, URDF export from the same model, and a headless simulator that runs as a native ROS 2 node.

## A scene

A scene is a plain script: it builds a Three.js scene in the default physics world and puts a `Robot` in it. The robot is a group, and everything under it is the robot.

```ts
import { CameraSensor, JointActuator, Robot } from "@drawcall/robotics";
import { BoxCollider, RevoluteJoint, RigidBody } from "@drawcall/physics";
import { Scene } from "three";

const scene = new Scene();
const robot = new Robot();
robot.name = "cart";
scene.add(robot);

const base = new RigidBody({ mass: 12 });
base.name = "base_link";
base.add(new BoxCollider({ size: [0.6, 0.2, 0.8] }));
const wheel = new RigidBody({ mass: 1 });
wheel.name = "wheel_link";
const axle = new RevoluteJoint({ body0: base, body1: wheel, axis: "X" });
axle.name = "wheel";
axle.setDrive(
  new JointActuator({ interface: "velocity", damping: 30, maxForce: 10 }),
);
robot.add(base, wheel, axle);

const camera = new CameraSensor({
  name: "camera",
  frameId: "camera_optical",
  rate: 5,
});
camera.position.set(0, 0.3, -0.4);
base.add(camera);
```

From that structure the robot derives everything ROS needs: the base link is the body no joint hangs from, every named axis joint is a URDF joint, a joint driven by a `JointActuator` is commanded through ros2_control with the actuator's interface (position, velocity or effort) while a plain `JointDrive` is a simulated spring, each sensor on the robot is a fixed frame, and a `base_footprint` link sits under the lowest collider. Before its first physics step, the robot publishes retained `robot_description` and `/tf_static` messages. It then publishes `joint_states` and its moving link transforms on `/tf` at its `rate`, 100 Hz by default, and hands each actuator its joint's value from `joint_commands`. `exportURDF()` serializes the runtime's robot, and `exportURDF({ name })` picks one of several. A body under the robot that hangs from a joint URDF cannot describe, such as a ball joint, is an error: keep it beside the robot group.

Several robots in one scene each take their own namespace, `new Robot({ namespace: "/arm1" })`: their topics and their sensors' topics go under it, and their TF frames carry it as a prefix (`arm1/base_link`), as `robot_state_publisher`'s `frame_prefix` and ros2_control's controllers do. The URDF keeps plain link names. A robot without a namespace uses the runtime's and plain frame names; two robots in one namespace are an error.

```sh
drawcall-robotics urdf scene.ts > robot.urdf      # links, colliders, joints, mounts, ros2_control block; meshes embedded
drawcall-robotics urdf scene.ts -o robot.urdf -s  # meshes as STL files beside it, for RViz and MoveIt
drawcall-robotics sim scene.ts --viewer               # serves and opens Lichtblick with a layout for the scene's sensors
drawcall-robotics sim scene.ts --viewer --show /map  # also display another topic, see below
drawcall-robotics view rosbag2_2026_09_16-14_50_26    # a rosbag2 directory or .mcap file, layout from its sensors
drawcall-robotics snapshot /robot/camera/image_raw/compressed --out frame.jpg
drawcall-robotics episodes bringup.launch.xml --count 12 --arg scene:=task.ts  # evaluation episodes in parallel
```

`sim` is a ROS 2 node through [rclnodejs](https://github.com/RobotWebTools/rclnodejs). It publishes `/clock`, retained `<namespace>/robot_description`, `/tf`, `/tf_static`, `<namespace>/joint_states`, one `LaserScan` per lidar and a raw `Image`, a `CompressedImage` and a `CameraInfo` per camera under image_transport's names, and actuators apply `<namespace>/joint_commands` as published by the [topic-based ros2_control hardware interface](https://github.com/PickNikRobotics/topic_based_ros2_control) the URDF declares. It offers the standard [simulation_interfaces](https://github.com/ros-simulation/simulation_interfaces) services `/sim/get_simulator_features`, `/sim/get_simulation_state`, `/sim/set_simulation_state` (play, pause), `/sim/step_simulation` and `/sim/reset_simulation`, plus `/sim/start_evaluation`. Physics steps on while up to three frames per camera read back and waits only when all three are in flight, so frames are never dropped and a slow GPU slows the simulation instead; each frame carries the simulation time it was rendered at. `--real-time-factor <x>` sets simulated seconds per wall-clock second, 1 by default; 0 runs as fast as the host allows. Nodes that follow simulation time (`use_sim_time`) keep pace at any factor. The viewer shows the robots, lidars and cameras; `--show <topic>` adds any other topic, a compressed image in its own panel and anything else in 3D, and the view follows `map` when `/map` is shown. `view` takes the same flag and gives every recorded compressed image a panel. Start it from a launch file like any simulator:

```python
ExecuteProcess(cmd=["drawcall-robotics", "sim", "scene.ts"])  # node_modules/.bin on PATH, e.g. through pixi.toml
```

Install the package with the ROS environment sourced, for example `pixi run pnpm install`, because rclnodejs compiles against it like any ROS client library. rclnodejs is an optional dependency, so browser projects skip the build; it has prebuilt binaries only for Ubuntu, and on macOS its 2.2.0 release needs the one-line compile fix in this repository's `patches/`. The CLI needs Node.js 24.2 or newer.

## In the browser

The same devices work against rosbridge, for a page that simulates while ROS runs elsewhere. The page authors the scene, builds its MuJoCo world, and steps it in its own loop:

```ts
import { ROS2, RosbridgeConnection } from "@drawcall/robotics";
import { buildWorld } from "@drawcall/physics-mujoco";

const connection = new RosbridgeConnection();
await connection.connect("ws://localhost:9090", 10_000);
const ros = new ROS2(connection, { renderer, namespace: "/robot" });
await import("./scene.ts"); // the same script the CLI runs
const world = await buildWorld({ gravity: [0, -9.81, 0] });
ros.attachWorld(world);
// In the render loop: world.update(deltaSeconds);
```

`renderer` is a `CaptureRenderer`: a Three.js `WebGPURenderer`, or anything with `render`, `getRenderTarget`, `setRenderTarget` and `readRenderTargetPixelsAsync`. Byte arrays are `Uint8Array` everywhere; the rosbridge transport encodes them as base64 on the wire.

The page owns renderer cleanup. Call `await ros.disposeAsync()` before releasing the renderer: it disconnects and detaches every device immediately, then waits for pending device work, including GPU readbacks. Put renderer and world disposal in an outer cleanup scope so both still run if device cleanup fails. Disposing one camera works the same way: `await camera.dispose()` detaches it immediately and waits for its GPU cleanup; a readback failure rejects that promise without faulting the remaining runtime.

## Devices

| Device          | Configuration and output                                                                                                                                                                                                                |
| --------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `Robot`         | Group with a `name`, optional `namespace` and `rate`; publishes its description, link TF and joint states; `exportURDF()` also exports it offline                                                                                       |
| `JointActuator` | `JointDrive` with a ros2_control `interface` and URDF `maxVelocity`; attached with `joint.setDrive`, its robot hands it `joint_commands` for its joint's name; it rejects targets from the scene                                        |
| `CameraSensor`  | PerspectiveCamera with `name`, dimensions, `fov`, `rate`, `quality`, `frameId`; publishes `<name>/image_raw` (`rgb8`), `<name>/image_raw/compressed` and `<name>/camera_info`                                                           |
| `LaserSensor`   | Object3D with `name`, `samples`, `range`, `minRange`, `angleMin`, `angleMax`, `rate`, `frameId`; publishes a scan of everything, its own robot included; per REP 117 no return is `+Inf` and one inside the `minRange` dead zone `-Inf` |

Devices register with the runtime they are given, by default the current runtime, and unregister on `dispose()`. A camera captures the scene it is part of. `ros.find(CameraSensor)` lists the registered devices of a class. The owning `Robot` publishes mounting and internal joint transforms; localization owns transforms above `base_footprint`. Do not also run `robot_state_publisher` for the same robot. Joint commands expire after 0.5 s and expiry makes the actuator passive without braking. Disposing an actuator detaches it from its joint.

## URDF export

`exportURDF()` serializes the robot in ROS coordinates: bodies become links with actual Three.js visual meshes and colors, separate primitive or mesh collision geometry, fixed joints and revolute/prismatic joints with their axes and limits, sensors become fixed frames, `base_footprint` sits at the lowest collider, joint limits carry each actuator's `maxForce` as effort and `maxVelocity` as velocity, and actuators produce the topic-based `ros2_control` block. Mesh geometry is embedded as binary STL data URLs in the XML, so the retained description and its recorded copy need no asset server or sidecar files. Visual meshes currently support a single untextured material per mesh; unsupported textures or material arrays fail explicitly. The viewer uses ROS Z-up mesh coordinates and model framing saved in an XML comment. Recordings made before description/TF publication do not gain 3D models retroactively. To export from your own script without ROS, create the runtime over an `OfflineConnection`, which only builds; `findRobot(ros, name?)` picks a scene's robot, and `rosPosition`, `rosRotation`, `rosOpticalRotation` and `rollPitchYaw` are the coordinate conversions.

## Custom devices

Implement the `Device` interface: `beforeStep(time, delta)` for commands, `afterStep(time, delta)` for measurements, `stop()` for faults and `dispose()` as needed, then call `ros.register(this)`, which returns the unregister function; `dispose()` calls it. `ros.publish` and `ros.subscribe` are the building blocks. Runtime disposal synchronously calls every device's disposal, releases hooks and the transport, and reports an `AggregateError` if any synchronous release throws. Devices with asynchronous work return a promise from `dispose()`. Runtime `disposeAsync()` waits for every such promise and reports synchronous and asynchronous release failures together. It also works after `dispose()`, and repeated calls report the original failures; repeated synchronous `dispose()` calls remain no-ops.

## Evaluation

`Evaluator` is exported from `@drawcall/robotics`. It registers with the default runtime; it is not a scene object or sensor. See the [evaluation guide](../../README.md#evaluation-and-results) for the API, standard ROS topics, recording lifecycle and offline results command. The runner starts one evaluation per recording, checks named conditions after physics/device updates, and freezes the final numeric metrics.

`episodes` runs a launch file's evaluation once per seed, several at a time, and tabulates the results. Each episode is a `ros2 launch` in a ROS domain of its own (`--domain` sets the first, so runs side by side need distinct ranges), kept to this machine, with `SEED` set for the scene, `OMP_NUM_THREADS` set to its share of the cores unless already set, and `DRAWCALL_RESULT` naming the file `sim` writes the final result to. Logs and results go to `--out`, one folder per seed; `--timeout` stops an episode by wall clock, as Ctrl-C would.
