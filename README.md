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

The browser simulates the robot and publishes sensor data. SLAM Toolbox estimates position and builds an occupancy map from lidar and wheel encoders. Nav2 plans and follows four goals using that map. The dashboard shows the ROS map, estimated position and camera image. See the [ROS guide](examples/warehouse/ros/README.md) for architecture, reset and validation.

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

| Topic                                                     | Type                                  | Direction                      |
| --------------------------------------------------------- | ------------------------------------- | ------------------------------ |
| `/robot/left_wheel/command`, `/robot/right_wheel/command` | `std_msgs/msg/Float64`                | ROS → browser                  |
| `/robot/joint_states`                                     | `sensor_msgs/msg/JointState`          | Browser → ROS                  |
| `/robot/scan`                                             | `sensor_msgs/msg/LaserScan`           | Browser → ROS                  |
| `/robot/camera/image/compressed`                          | `sensor_msgs/msg/CompressedImage`     | Browser → ROS                  |
| `/robot/camera/camera_info`                               | `sensor_msgs/msg/CameraInfo`          | Browser → ROS                  |
| `/odom`                                                   | `nav_msgs/msg/Odometry`               | Encoder adapter → ROS          |
| `/map`                                                    | `nav_msgs/msg/OccupancyGrid`          | SLAM Toolbox → ROS/browser     |
| `/cmd_vel`                                                | `geometry_msgs/msg/Twist`             | Nav2 → motor adapter           |
| `/tf`, `/tf_static`                                       | `tf2_msgs/msg/TFMessage`              | ROS estimation and robot model |
| `/clock`                                                  | `rosgraph_msgs/msg/Clock`             | Browser → ROS                  |
| `/robot/status`                                           | `std_msgs/msg/String` containing JSON | Mission client → browser       |

Relative topic names use the runtime namespace. Absolute names such as `/clock` and `/tf` bypass it. Use one clock publisher per ROS graph. Frame IDs are independent of topic namespaces; choose distinct frame IDs for multiple robots or sensors.

Three.js forward `-Z`, left `-X`, up `+Y` map to ROS map axes `+X`, `+Y`, `+Z`. Camera optical frames use `+Z` forward, `+X` right and `+Y` down. The browser publishes no TF. ROS provides `map → odom → base_footprint → base_link → sensors` from SLAM, encoder odometry and known mounting offsets.

## Timing and limits

Enable `use_sim_time` on ROS consumers; the example controller does this automatically. Simulation time counts completed fixed substeps, including catch-up work. This is asynchronous messaging: publishing `/clock` does not wait for ROS callbacks or acknowledge that ROS processed a step. There is no lockstep runner.

Motor commands are unstamped `Float64` values. Each binding keeps the latest received value and checks both elapsed simulation time and monotonic wall time; the default timeout is 0.5 seconds. Expiry or disconnection disables the drive before further physics advancement. **Disabling removes actuation; the robot can coast. It does not brake or hold position.** A new command can reactivate a timed-out binding. There is no automatic transport reconnect; dispose and set up a new runtime.

Current scope:

- Direct per-motor topics, not a `ros2_control` hardware plugin or trajectory action server.
- Wheel encoder odometry, SLAM Toolbox mapping/localization, and Nav2 navigation; no ground-truth pose API.
- A configurable planar lidar, 360° by default; no-hit rays use `range_max + 1`, marking them outside the valid measurement range. All rays in a scan represent one simulation instant.
- Synchronous WebGL readback and JPEG encoding, with 320×180 at 5 Hz as the camera default. Large images or many cameras can stall the browser. Outgoing publications are dropped when websocket buffering exceeds 4 MiB; `ros.dropped` counts them.
- Ordinary perspective capture without lens distortion. Render-target capture bypasses application postprocessing.
- Reset means reloading the application. Restart the ROS service before reloading the browser to reset SLAM and odometry together. Backwards encoder timestamps stop the service visibly. `world.reset()` alone is not a coordinated ROS reset.

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

The smoke check requires live odometry, scan, JPEG, joint-state and status messages, an occupancy map and the expected TF chain, an advancing clock, nonzero commands on both wheels and resulting encoder-estimated motion. A timeout fails visibly and reports which observations are missing.

For a complete circuit, through Nav2:

```sh
docker compose -f examples/warehouse/compose.yaml exec ros bash -lc \
  'source /opt/ros/jazzy/setup.bash && python3 /app/smoke.py --patrol --timeout 240'
```
