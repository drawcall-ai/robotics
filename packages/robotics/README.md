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

From that structure the robot derives everything ROS needs: the base link is the body no joint hangs from, every named axis joint is a URDF joint, a joint driven by a `JointActuator` is commanded through ros2_control with the actuator's interface (position, velocity or effort) while a plain `JointDrive` is a simulated spring, each sensor on the robot is a fixed frame, and a `base_footprint` link sits under the lowest collider. Before its first physics step, the robot publishes retained `robot_description` and `/tf_static` messages. It then publishes `joint_states` and its moving link transforms on `/tf` at 20 Hz; each actuator applies its own `joint_commands`; `exportURDF()` serializes the runtime's robot, and `exportURDF({ name })` picks one of several.

```sh
drawcall-robotics urdf scene.ts > robot.urdf      # links, colliders, joints, mounts, ros2_control block
drawcall-robotics sim scene.ts --viewer               # serves and opens Lichtblick with a layout for the scene's sensors
drawcall-robotics sim scene.ts --viewer --perception  # also the controller's own view of the world, see below
drawcall-robotics view rosbag2_2026_09_16-14_50_26    # a rosbag2 directory or .mcap file, layout from its sensors
drawcall-robotics snapshot /robot/camera/image/compressed --out frame.jpg
```

`sim` is a ROS 2 node through [rclnodejs](https://github.com/RobotWebTools/rclnodejs). It publishes `/clock`, retained `<namespace>/robot_description`, `/tf`, `/tf_static`, `<namespace>/joint_states`, one `LaserScan` per lidar and a `CompressedImage` plus `CameraInfo` per camera, and actuators apply `<namespace>/joint_commands` as published by the [topic-based ros2_control hardware interface](https://github.com/PickNikRobotics/topic_based_ros2_control) the URDF declares. It offers `/sim/pause`, `/sim/resume`, `/sim/step` and `/sim/reset` as `std_srvs/Trigger` and publishes ground truth on `/sim/state`. A controller can publish what it saw, as perception nodes usually do for RViz and Foxglove: with `--perception`, the viewer layout adds an image panel for `<namespace>/perception/image/compressed`, a `CompressedImage` with the controller's detections drawn on it. It stays out of the 3D panel, where it could be mistaken for ground truth. Recordings capture it, and `view` gives every recorded image topic a panel. Start it from a launch file like any simulator:

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

| Device          | Configuration and output                                                                                                                                                                 |
| --------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `Robot`         | Group with a `name`; automatically publishes its description, link TF and joint states; `exportURDF()` also exports it offline                                                           |
| `JointActuator` | `JointDrive` with a ros2_control `interface` and URDF `maxVelocity`; attached with `joint.setDrive`, it applies `joint_commands` for its joint's name and rejects targets from the scene |
| `CameraSensor`  | PerspectiveCamera with `name`, dimensions, `fov`, `rate`, `quality`, `frameId`; publishes `<name>/image/compressed` and `<name>/camera_info`                                             |
| `LaserSensor`   | Object3D with `name`, `samples`, `range`, `minRange`, `angleMin`, `angleMax`, `rate`, `frameId`; publishes a scan of everything, its own robot included; `minRange` is the dead zone     |

Devices register with the runtime they are given, by default the current runtime, and unregister on `dispose()`. A camera captures the scene it is part of. `ros.find(CameraSensor)` lists the registered devices of a class. The owning `Robot` publishes mounting and internal joint transforms; localization owns transforms above `base_footprint`. Do not also run `robot_state_publisher` for the same robot. Joint commands expire after 0.5 s and expiry makes the actuator passive without braking. Disposing an actuator detaches it from its joint.

## URDF export

`exportURDF()` serializes the robot in ROS coordinates: bodies become links with actual Three.js visual meshes and colors, separate primitive or mesh collision geometry, fixed joints and revolute/prismatic joints with their axes and limits, sensors become fixed frames, `base_footprint` sits at the lowest collider, joint limits carry each actuator's `maxForce` as effort and `maxVelocity` as velocity, and actuators produce the topic-based `ros2_control` block. Mesh geometry is embedded as binary STL data URLs in the XML, so the retained description and its recorded copy need no asset server or sidecar files. Visual meshes currently support a single untextured material per mesh; unsupported textures or material arrays fail explicitly. The viewer uses ROS Z-up mesh coordinates and model framing saved in an XML comment. Recordings made before description/TF publication do not gain 3D models retroactively. To export from your own script without ROS, create the runtime over an `OfflineConnection`, which only builds; `findRobot(ros, name?)` picks a scene's robot, and `rosPosition`, `rosRotation`, `rosOpticalRotation` and `rollPitchYaw` are the coordinate conversions.

## Custom devices

Implement the `Device` interface: `beforeStep(time, delta)` for commands, `afterStep(time, delta)` for measurements, `stop()` for faults and `dispose()` as needed, then call `ros.register(this)`, which returns the unregister function; `dispose()` calls it. `ros.publish` and `ros.subscribe` are the building blocks. Runtime disposal synchronously calls every device's disposal, releases hooks and the transport, and reports an `AggregateError` if any synchronous release throws. Devices with asynchronous work return a promise from `dispose()`. Runtime `disposeAsync()` waits for every such promise and reports synchronous and asynchronous release failures together. It also works after `dispose()`, and repeated calls report the original failures; repeated synchronous `dispose()` calls remain no-ops.

## Evaluation

`Evaluator` is exported from `@drawcall/robotics`. It registers with the default runtime; it is not a scene object or sensor. See the [evaluation guide](../../README.md#evaluation-and-results) for the API, standard ROS topics, recording lifecycle and offline results command. The runner starts one evaluation per recording, checks named conditions after physics/device updates, and freezes the final numeric metrics.
