# @drawcall/robotics

ROS 2 simulation for Three.js and Drawcall physics: sensors and ros2_control bindings as scene objects, URDF export from the same model, and a headless simulator that runs as a native ROS 2 node.

## A scene

A scene is a plain script: it builds a Three.js scene in the default physics world and puts a `Robot` in it. The robot is a group, and everything under it is the robot.

```ts
import { CameraSensor, Robot } from "@drawcall/robotics";
import {
  BoxCollider,
  JointMotor,
  RevoluteJoint,
  RigidBody,
} from "@drawcall/physics";
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
new JointMotor({ joint: axle, damping: 30, maxForce: 10 });
robot.add(base, wheel, axle);

const camera = new CameraSensor({
  name: "camera",
  frameId: "camera_optical",
  rate: 5,
});
camera.position.set(0, 0.3, -0.4);
base.add(camera);
```

From that structure the robot derives everything ROS needs: the base link is the body no joint hangs from, every named axis joint is a URDF joint, a joint with a motor is commanded through ros2_control (velocity, or position when the motor has stiffness), each sensor on the robot is a fixed frame, and a `base_footprint` link sits under the lowest collider. The robot publishes `joint_states` and applies `joint_commands` itself; `robot.urdf()` serializes it.

```sh
drawcall-robotics urdf scene.ts > robot.urdf      # links, colliders, joints, mounts, ros2_control block
drawcall-robotics sim scene.ts --viewer               # serves and opens Lichtblick with a layout for the scene's cameras
drawcall-robotics view rosbag2_2026_09_16-14_50_26    # a rosbag2 directory or .mcap file, layout from its sensors
drawcall-robotics snapshot /robot/camera/image/compressed --out frame.jpg
```

`sim` is a ROS 2 node through [rclnodejs](https://github.com/RobotWebTools/rclnodejs). It publishes `/clock`, `<namespace>/joint_states`, one `LaserScan` per lidar and a `CompressedImage` plus `CameraInfo` per camera, and drives motors from `<namespace>/joint_commands` as published by the [topic-based ros2_control hardware interface](https://github.com/PickNikRobotics/topic_based_ros2_control) the URDF declares. It offers `/sim/pause`, `/sim/resume`, `/sim/step` and `/sim/reset` as `std_srvs/Trigger` and publishes ground truth on `/sim/state`. Start it from a launch file like any simulator:

```python
ExecuteProcess(cmd=["drawcall-robotics", "sim", "scene.ts"])  # node_modules/.bin on PATH, e.g. through pixi.toml
```

Install the package with the ROS environment sourced, for example `pixi run pnpm install`, because rclnodejs compiles against it like any ROS client library. rclnodejs is an optional dependency, so browser projects skip the build; it has prebuilt binaries only for Ubuntu, and on macOS its 2.2.0 release needs the one-line compile fix in this repository's `patches/`. The CLI needs Node.js 24.2 or newer.

## In the browser

The same devices work against rosbridge, for a page that simulates while ROS runs elsewhere. The page owns the physics world (`setupWorld` makes it the default) and steps it in its own loop:

```ts
import { ROS2, RosbridgeConnection } from "@drawcall/robotics";

const connection = new RosbridgeConnection();
await connection.connect("ws://localhost:9090", 10_000);
new ROS2(connection, { renderer, namespace: "/robot" }); // in the default physics world
await import("./scene.ts"); // the same script the CLI runs
```

`renderer` is a Three.js `WebGPURenderer`; the runtime needs `render`, `getRenderTarget`, `setRenderTarget` and `readRenderTargetPixelsAsync`. Byte arrays are `Uint8Array` everywhere; the rosbridge transport encodes them as base64 on the wire.

## Devices

| Device         | Configuration and output                                                                                                                                                             |
| -------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `Robot`        | Group with a `name`; publishes `joint_states` for its joints, applies `joint_commands` to its motors, `urdf()` exports it                                                            |
| `CameraSensor` | PerspectiveCamera with `name`, dimensions, `fov`, `rate`, `quality`, `frameId`; publishes `<name>/image/compressed` and `<name>/camera_info`                                         |
| `LaserSensor`  | Object3D with `name`, `samples`, `range`, `minRange`, `angleMin`, `angleMax`, `rate`, `frameId`; publishes a scan of everything, its own robot included; `minRange` is the dead zone |

Devices register with the runtime they are given, by default the current one from `getROS2()`, and unregister on `dispose()`. A camera captures the scene it is part of. `ros.find(CameraSensor)` lists the registered devices of a class. Sensors never publish TF or world poses: the URDF's `robot_state_publisher` owns mounting transforms and localization owns the rest. Joint commands expire after 0.5 s and expiry disables actuation without braking.

## URDF export

`robot.urdf()` serializes the robot in ROS coordinates: bodies become links with box, sphere and cylinder colliders as geometry, revolute and prismatic joints keep their axes and limits, sensors become fixed frames, `base_footprint` sits at the lowest collider, and motors produce the topic-based `ros2_control` block. Output is deterministic and independent of where the robot stands. `findRobot(ros, name?)` picks a scene's robot, and an `OfflineConnection` gives a runtime that only builds. Helpers `rosPosition`, `rosRotation`, `rosOpticalRotation` and `rollPitchYaw` implement the conversions.

## Custom devices

Extend `Device`, or `withDevice(Base)` for Three.js subclasses, to inherit registration and disposal; implement `beforeStep(time, delta)` for commands, `afterStep(time, delta)` for measurements and `stop()` for faults. `ros.publish`, `ros.subscribe`, `Schedule`, `stamp` and `header` are the building blocks. Runtime disposal releases every device, subscription, hook and the transport, and reports an `AggregateError` if any release throws.
