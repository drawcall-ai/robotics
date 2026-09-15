# Drawcall Robotics

Connect a Three.js physics scene to ROS 2 through rosbridge. Declare cameras, lidar and motor bindings as ordinary objects; ROS runs in Docker and sends commands to native physics motors.

| Directory                                                  | Purpose                                                                     |
| ---------------------------------------------------------- | --------------------------------------------------------------------------- |
| [packages/robotics](packages/robotics/README.md)           | `@drawcall/robotics`: runtime, devices, coordinate conversion and transport |
| [examples/warehouse](examples/warehouse)                   | Three.js warehouse and differential-drive robot                             |
| [examples/warehouse/ros](examples/warehouse/ros/README.md) | ROS Jazzy controller, rosbridge image and live smoke check                  |
| [DESIGN.md](DESIGN.md)                                     | Implemented architecture and scope                                          |

The workspace uses `@drawcall/physics` and its Rapier backend **0.4.0**. `JointMotor` belongs to physics; robotics binds ROS commands to it.

## Run the warehouse

Requirements: Node.js 22.12+ (CI uses 24), pnpm 10 and a running Docker engine with Compose.

```sh
pnpm install
pnpm ros
```

In another terminal:

```sh
pnpm dev
```

Open the URL printed by Vite; if its preferred port is occupied, Vite chooses another. Click **Connect to ROS 2** with `ws://localhost:9090`. Compose exposes the websocket only on host `127.0.0.1`.

The browser simulates the robot and publishes sensor data. The Python ROS node owns waypoint navigation and wheel commands. It patrols four goals, uses lidar for obstacle avoidance, and slows near the origin docking bay when its camera sees the orange sign. The dashboard displays ROS status and the captured camera image.

Stop the Docker service when finished:

```sh
docker compose -f examples/warehouse/compose.yaml down
```

## API shape

```ts
const ros = await setupROS2("ws://localhost:9090", {
  world,
  scene,
  renderer,
  namespace: "/robot",
});

const motor = new JointMotor({ joint: wheelJoint, damping: 10, maxForce: 20 });
new MotorBinding({ motor, name: "left_wheel", mode: "velocity" });

const camera = new CameraSensor({ name: "camera", rate: 5 });
chassis.add(camera);

// Keep the application's existing display loop.
world.update(frameDelta);
renderer.render(scene, viewer);
```

See the [package guide](packages/robotics/README.md) for imports, complete setup, multiple devices and extensions.

- `setupROS2` connects and installs physics hooks in one awaited call.
- `getROS2()` returns the connected default runtime or throws. Devices capture it at construction; use `{ ros }` for an explicit runtime.
- Devices register when constructed. Scene parenting positions spatial sensors; there is no scene discovery or upfront device list.
- Before each fixed physics substep, motor bindings apply the latest fresh command. After each completed substep, the runtime publishes `/clock` and samples due sensors.
- Native physics motors execute the drive. Sensor joint states report physical motion, not target values.
- Dispose the runtime to release its devices, subscriptions and hooks. Physics bodies, joints, motors, the scene and renderer remain application-owned.

## ROS topics

Defaults under namespace `/robot`:

| Topic                                                     | Type                                  | Direction                 |
| --------------------------------------------------------- | ------------------------------------- | ------------------------- |
| `/robot/left_wheel/command`, `/robot/right_wheel/command` | `std_msgs/msg/Float64`                | ROS → browser             |
| `/robot/joint_states`                                     | `sensor_msgs/msg/JointState`          | Browser → ROS             |
| `/robot/pose`                                             | `geometry_msgs/msg/PoseStamped`       | Browser → ROS             |
| `/robot/scan`                                             | `sensor_msgs/msg/LaserScan`           | Browser → ROS             |
| `/robot/camera/image/compressed`                          | `sensor_msgs/msg/CompressedImage`     | Browser → ROS             |
| `/robot/camera/camera_info`                               | `sensor_msgs/msg/CameraInfo`          | Browser → ROS             |
| `/clock`                                                  | `rosgraph_msgs/msg/Clock`             | Browser → ROS             |
| `/tf`                                                     | `tf2_msgs/msg/TFMessage`              | Browser → ROS             |
| `/robot/status`                                           | `std_msgs/msg/String` containing JSON | Demo controller → browser |

Relative topic names use the runtime namespace. Absolute names such as `/clock` and `/tf` bypass it. Use one clock publisher per ROS graph. Frame IDs are independent of topic namespaces; choose distinct frame IDs for multiple robots or sensors.

Three.js forward `-Z`, left `-X`, up `+Y` map to ROS map axes `+X`, `+Y`, `+Z`. Camera optical frames use `+Z` forward, `+X` right and `+Y` down. Published TF locates body, laser and camera optical frames directly beneath `map`.

## Timing and limits

Enable `use_sim_time` on ROS consumers; the example controller does this automatically. Simulation time counts completed fixed substeps, including catch-up work. This is asynchronous messaging: publishing `/clock` does not wait for ROS callbacks or acknowledge that ROS processed a step. There is no lockstep runner.

Motor commands are unstamped `Float64` values. Each binding keeps the latest received value and checks both elapsed simulation time and monotonic wall time; the default timeout is 0.5 seconds. Expiry or disconnection disables the drive before further physics advancement. **Disabling removes actuation; the robot can coast. It does not brake or hold position.** A new command can reactivate a timed-out binding. There is no automatic transport reconnect; dispose and set up a new runtime.

Current scope:

- Direct per-motor topics, not a `ros2_control` hardware plugin or trajectory action server.
- Ground-truth map pose, not estimated odometry or localization.
- A 180° planar lidar; no-hit rays use `range_max`, not infinity. All rays in a scan represent one simulation instant.
- Synchronous WebGL readback and JPEG encoding, with 320×180 at 5 Hz as the camera default. Large images or many cameras can stall the browser. Outgoing publications are dropped when websocket buffering exceeds 4 MiB; `ros.dropped` counts them.
- Ordinary perspective capture without lens distortion. Render-target capture bypasses application postprocessing.
- Reset means reloading the application. The demo controller detects backwards sensor timestamps and restarts its patrol; other ROS nodes must handle clock jumps or be restarted. `world.reset()` alone is not a coordinated ROS reset.

## Validation

```sh
pnpm check
python3 -m unittest discover -s examples/warehouse/ros -p 'test_*.py'
```

With Docker running and the browser connected, run the live smoke check:

```sh
docker compose -f examples/warehouse/compose.yaml exec ros bash -lc \
  'source /opt/ros/jazzy/setup.bash && python3 /app/smoke.py --timeout 60'
```

The smoke check requires live pose, scan, JPEG, joint-state and status messages, an advancing clock, nonzero commands on both wheels and resulting pose movement. A timeout fails visibly and reports which observations are missing.

For a complete circuit, including camera-assisted docking:

```sh
docker compose -f examples/warehouse/compose.yaml exec ros bash -lc \
  'source /opt/ros/jazzy/setup.bash && python3 /app/smoke.py --patrol --timeout 240'
```
