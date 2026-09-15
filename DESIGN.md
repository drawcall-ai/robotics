# Implemented design

The browser is the simulated hardware. ROS owns estimation, mapping and navigation.

## Responsibilities

| Owner                 | Responsibilities                                                                   |
| --------------------- | ---------------------------------------------------------------------------------- |
| Three.js application  | Scene, physical robot assembly, sensor placement and display loop                  |
| Drawcall physics      | Bodies, joints, native motors, measured joint motion and fixed substeps            |
| Robotics package      | Transport, sensor measurement publication, command expiry and lifecycle            |
| ROS bridge adapter    | Encoder-only wheel odometry and velocity-command conversion                        |
| robot_state_publisher | Fixed transforms from known mounting geometry                                      |
| SLAM Toolbox          | Laser scan matching, map construction, loop closure and map-to-odometry correction |
| Nav2                  | Costmaps, path planning, path following and recovery behaviors                     |
| Mission client        | Operator goal sequence and status display                                          |

## Sensor boundary

Camera and lidar use scene geometry internally to synthesize measurements. They publish image/calibration or ranges with timestamps and frame identifiers. They never publish their world positions or TF. JointStateSensor reports measured joint angles/velocities, analogous to ideal encoders. No ground-truth pose device is exported.

These sensors do not yet model noise, quantization, distortion or rolling acquisition. Ideal measurements are distinct from leaking the simulator's world pose. The example's odometry integrates encoder angle changes and cannot observe wheel slip. SLAM receives that imperfect dead reckoning and independent scans, not simulator poses.

## Frames and timing

Three forward -Z, left -X, up +Y map to ROS forward +X, left +Y, up +Z. Camera optical axes are right/down/forward. ROS owns the chain `map → odom → base_footprint → base_link → laser/camera_optical`. Fixed sensor mounting offsets come from the example URDF; the first encoder sample establishes an arbitrary odometry origin.

Before each fixed physics step, fresh ROS commands update native motors. After each step, `/clock` and due sensors use completed simulation time. This is asynchronous ROS transport, not a lockstep simulation. Outgoing publications drop above 4 MiB of websocket buffering.

`MotorBinding` checks simulation and wall-clock command age. Expiry/disconnection disables actuation and permits coasting. The ROS bridge separately checks fresh velocity commands and encoders. Nav2 consumes `/odom` and `/robot/scan`, publishing `/cmd_vel`; the adapter converts that to wheel topics.

## Ownership and failures

`setupROS2` establishes the default runtime; devices can receive an explicit `ros`. Device/withDevice handles registration and native Three.js inheritance. DeviceLifecycle defines optional step/stop hooks and required disposal. Removing a scene parent does not unregister a sensor.

Runtime disposal owns device, subscription, physics-hook and transport cleanup. Cleanup attempts every release before surfacing aggregated errors. Motors, joints, world, scene and renderer remain application-owned. The transport does not reconnect automatically.

A new browser simulation requires restarting the ROS stack: resetting only odometry while retaining an old SLAM graph is invalid. Backwards encoder timestamps terminate the service visibly. Disconnection without a clock reset may reconnect to the existing session.

## Example scope

The example uses upstream SLAM Toolbox and Nav2 with application-specific YAML configuration. It starts without a preloaded warehouse map. Four mission goals are coordinates in the initially established map frame. The camera is available for other perception work but is not used by the lidar navigation pipeline.

The Python hardware adapter is a minimal ROS interface, not a ros2_control hardware plugin. Wheel geometry and mounting transforms must agree with the physical assembly, as on a real robot. Odometry covariance is uncalibrated. The planar lidar sees only its scan plane. There is no claim of calibrated localization accuracy or a full hardware sensor model.

Validation includes physics behavior, no-pose-leak sensor tests, encoder integration tests, package/type checks and a live SLAM/Nav2 smoke check. The ROS guide documents full mission validation and reset procedures.
