# Warehouse SLAM and navigation

Run from the repository root:

```sh
pnpm ros
pnpm dev
```

Open the browser and click **Connect to ROS 2**. The ROS service starts:

- **SLAM Toolbox**: builds `/map` from `/robot/scan`, matches scans, closes loops and publishes `map → odom`.
- **Nav2**: plans on the growing occupancy map and controls motion with DWB, costmaps and recovery behaviors.
- **robot_state_publisher**: reads `robot.urdf` and publishes fixed chassis/camera/lidar mounting transforms.
- **bridge.py**: integrates measured wheel encoder angles into `/odom` and `odom → base_footprint`; translates Nav2 `/cmd_vel` into native wheel motor commands.
- **mission.py**: submits four operator-specified goals to Nav2. It does not compute navigation commands or read simulator state.
- **rosbridge**: transports sensor measurements and motor commands to/from the browser.

## Data boundary

The browser publishes `/clock`, `/robot/joint_states`, `/robot/scan`, `/robot/camera/image/compressed` and `/robot/camera/camera_info`. It publishes no world positions and no TF. Images, calibration, ranges and encoder angles are ideal simulated measurements; no noise, lens distortion or encoder quantization model is claimed.

```text
map                         SLAM Toolbox estimate
 └─ odom                    wheel encoder dead reckoning
     └─ base_footprint       planar robot pose
         └─ base_link       fixed chassis height
             ├─ laser       calibrated mounting offset
             └─ camera_optical
```

Odometry starts at zero on the first encoder sample; it is not seeded from the scene. The simulated support balls rotate freely on spherical joints instead of sliding as fixed chassis colliders. Wheel slip is not observable in encoders and therefore produces odometry error. SLAM corrects localization using laser observations. Odometry covariance is a conservative uncalibrated model, not a measured hardware accuracy claim. The bridge assumes less than π radians of wheel motion between samples; encoders publish at 50 Hz.

The 360° lidar has 361 rays, a 12 m maximum range and 10 Hz sampling. The mounted scan plane is approximately 0.6 m above the floor. A 2D scanner cannot detect obstacles entirely outside that plane. No-hit rays use `range_max + 1`, the finite no-return convention also supported by ROS pointcloud_to_laserscan. This preserves JSON compatibility without reporting a fake obstacle at maximum range.

The camera remains an independent image sensor and preview. Navigation uses lidar and encoders; camera-based navigation is not configured. Goals `(4,0) → (4,3) → (0,3) → (0,0)` are mission inputs relative to the initially established map frame, not a supplied map. Nav2 can plan into unknown space while SLAM builds the map. Failed goals are surfaced and stop the mission.

Physical perimeter walls provide laser returns around the starting area. SLAM discovers these walls and the shelving from scans; their geometry is never supplied to the planner. Without surrounding returns, the initial scan map can exclude the robot's starting position.

## Lifetime

All ROS nodes use simulation time. Motor bindings expire commands on simulation and wall time. The bridge also stops sending motion when encoder or command updates are stale.

For a new simulation, **restart the ROS service, then reload and reconnect the browser**:

```sh
docker compose -f examples/warehouse/compose.yaml restart ros
```

A backwards encoder timestamp terminates the service visibly. Keeping an old SLAM map/TF history while silently zeroing odometry would corrupt localization. Disconnect/reconnect without resetting browser simulation time retains the current mapping session. Run one browser publisher per ROS graph.

## Validation

```sh
python3 -m unittest discover -s examples/warehouse/ros -p 'test_*.py'
docker compose -f examples/warehouse/compose.yaml exec -T ros bash -lc \
  'source /opt/ros/jazzy/setup.bash && python3 /app/smoke.py --timeout 90'
```

The smoke test checks encoder odometry, lidar, decoded JPEG, clock, commands, known occupancy cells, the expected TF chain, estimated motion and absence of a ground-truth pose publisher. It is not a ground-truth localization-accuracy benchmark.

Use `--patrol --timeout 240` to require all four Nav2 goals to succeed. The dashboard displays the actual ROS occupancy grid and estimated map position.

Configuration: [slam.yaml](slam.yaml), [nav2.yaml](nav2.yaml), [robot.urdf](robot.urdf). Upstream references: [SLAM Toolbox](https://docs.ros.org/en/jazzy/p/slam_toolbox/), [Nav2](https://docs.nav2.org/).
