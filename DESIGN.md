# Implemented design

This document describes the current implementation. The public entry point is `@drawcall/robotics`; the ROS application is an example rather than a required custom companion protocol.

## Responsibilities

| Layer                     | Owns                                                                                        |
| ------------------------- | ------------------------------------------------------------------------------------------- |
| Three.js application      | Scene hierarchy, renderer, physical robot assembly and display loop                         |
| `@drawcall/physics` 0.4.0 | Bodies, joints, native `JointMotor`, measured state and fixed substeps                      |
| `@drawcall/robotics`      | ROS transport, device registration, command freshness, sensor sampling and frame conversion |
| ROS application           | Navigation/control policy and command publication                                           |

Physics has no dependency on robotics. Robotics uses the `PhysicsWorld` contract; the warehouse selects Rapier. A motor binding does not implement a JavaScript servo: it sets a native `JointMotor` target.

## Setup and ownership

`await setupROS2(url, { world, scene, renderer, namespace })` opens rosbridge and installs the default runtime. The scene and renderer support sensor capture, while the world supplies time and substep hooks. Setup has no robot inventory or device-discovery phase. It resolves on transport connection, not on ROS application readiness.

`getROS2()` throws unless the default runtime is connected. Built-in devices capture that runtime in their constructors, or accept an explicit `ros` option. `new ROS2(connection, options)` permits an alternative implementation of the exported `Connection` interface; it does not install the default.

`CameraSensor` extends `withDevice(PerspectiveCamera)`; `LaserSensor` extends `withDevice(Object3D)`. Parenting determines spatial placement. Construction registers them independently of the scene graph, so removing a node from its parent does not stop publication. Call `dispose()` to unregister it and release its resources.

`MotorBinding` binds an existing physics motor, with one binding per motor. Construction disables that motor until a fresh command arrives. Binding disposal disables actuation and unsubscribes without disposing the motor or joint. Runtime disposal disposes its registered devices and closes transport; the application retains the physics world and rendering resources.

## Update sequence

The application calls `world.update(frameDelta)` and renders at display rate. Every executed fixed substep has this sequence:

1. The world invokes its before-step observers at the starting simulation time.
2. Robotics applies the latest fresh command to each bound motor, or disables stale actuation.
3. The physics backend evaluates native drives and constraints and advances the world.
4. The world exposes synchronized transforms, measured joint state and completed simulation time.
5. Robotics publishes `/clock`, then calls registered after-step devices to sample at their individual rates.

The synchronous device hooks do not wait for ROS. `/clock` and sensor stamps share simulation time, but a ROS timer may run before newly published sensors have arrived. Consumers use the latest available measurements. Dropped catch-up time is not simulated time.

`MotorBinding` receives unstamped `Float64` commands, replacing its previous value. It checks age against simulation time and monotonic wall time before a substep. Position commands use radians or meters; velocity commands use radians/second or meters/second. Velocity bindings require zero motor stiffness. Expiry disables the drive rather than applying brakes or holding a pose.

The physics backend's angular position restrictions still apply. ROS position controllers must respect its supported native-motor target domain; robotics does not supply trajectory interpolation or bypass backend errors.

## Extension contract

A device extends `Device`, or uses `withDevice(Base)` when it already needs a superclass. Both provide registration and disposal. The structural contract has optional `beforeStep(time, delta)`, `afterStep(time, delta)` and `stop()`, plus required `dispose()`. The shared lifecycle registers through `ros.register(device)` and owns the unregister function. Manual structural implementations may use the same registration API. Built-in and external devices use the same mechanism; runtime dispatch has no device-type branches.

Extensions use `ros.publish` and `ros.subscribe`, `Schedule`, `stamp`, and frame-conversion helpers. They own command validation and their sampling/actuation behavior. Actuator extensions should implement `stop()` to remove their actuation when the runtime fails or disconnects. See the [package guide](packages/robotics/README.md#custom-devices) for an example.

## Transport and failure

`RosbridgeConnection` uses roslib over a websocket transport. Topic types are checked for conflicts. Subscriptions and publishers use queue length/size one. If the socket has more than 4 MiB buffered, new publications are dropped and counted. This bounds ordinary sustained publishing pressure; it is not a reliable delivery or acknowledgement protocol.

Transport/protocol errors become runtime errors. A malformed motor message faults the runtime and stops actuators. Disconnection stops actuators when physics next invokes its hook. Device update failures are surfaced rather than silently converted into valid measurements. A failed runtime must be disposed and recreated; there is no automatic reconnect or session negotiation.

## Data and coordinates

Relative device topics use `/robot` by default. Camera `name` selects a topic prefix; laser `name` selects its scan topic. Callers must choose unique topic and frame IDs where needed. `/clock` and `/tf` are global. The implementation does not negotiate device IDs or exclusive ROS graph ownership.

World positions map as `(x_ros, y_ros, z_ros) = (-z_three, -x_three, y_three)`. Body/laser rotations convert between corresponding forward-left-up bases. Camera TF uses an optical basis: right, down, forward. Pose, laser and camera TF are direct children of `map`; this is ground-truth simulation TF rather than a URDF-derived articulated frame tree.

Joint feedback reports measured position/velocity; the effort array is empty because measured reaction effort is unavailable. Camera output is JPEG `CompressedImage` plus ideal pinhole `CameraInfo`, sharing the capture timestamp. Lidar uses backend raycasts, configurable body exclusion, a front 180° sweep and `range_max` for a miss.

## Deliberate boundaries

There is no `ros2_control` adapter, command timestamp scheduler, service/action layer, robot-description loader, registration handshake, lockstep, electrical motor model or coordinated reset. These are separate integrations to add when a concrete application needs them. Reset the demo by reloading the browser. Its controller detects backwards sensor timestamps and resets the patrol and sensor cache. Other ROS nodes remain responsible for handling clock jumps or restarting.

The warehouse demonstrates a ROS-owned controller with camera and lidar inputs. Its lightweight obstacle avoidance is not a general navigation planner. Synchronous camera readback/JPEG and per-ray physics queries suit a small example; many high-resolution sensors need separate performance work.
