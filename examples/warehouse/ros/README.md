# Warehouse ROS controller

From `examples/warehouse`:

```sh
docker compose up --build
```

ROS Jazzy and rosbridge share one supervised container. The websocket binds only to host `127.0.0.1:9090`. Either child exiting stops the service with a visible failure. `use_sim_time` drives the 20 Hz controller from browser `/clock` updates.

The controller consumes `/robot/pose` (`PoseStamped`), `/robot/scan` (`LaserScan`), `/robot/camera/image/compressed` (JPEG `CompressedImage`), and `/robot/joint_states` (`JointState`). Coordinates are ROS map coordinates: X forward, Y left, Z up. It publishes wheel angular velocities on `/robot/left_wheel/command` and `/robot/right_wheel/command` (`Float64`), using radius 0.22 m and track 0.9 m.

The patrol visits `(4,0) → (4,3) → (0,3) → (0,0)` with two simulation seconds at each waypoint. Lidar blocks forward motion near obstacles and selects a turn toward greater clearance. An orange sign in the center of the camera slows the final docking approach within 1.5 m of the origin. Missing initial sensors or stale timestamped pose/lidar/camera readings stop wheel commands. The browser should also expire wheel commands when the connection fails. This is a small demonstration controller, not a navigation stack: it has no global obstacle planner.

`/robot/status` is JSON carried by `std_msgs/msg/String`: `state`, `waypoint`, `goal`, `orange_fraction`, `front_distance`, `images`, `scans`, `joint_frames`, and `pose_frames`.

With the browser simulation running, verify live ROS sensor reception, image decoding, advancing simulation clock, outgoing nonzero commands and resulting pose motion:

```sh
docker compose exec ros bash -lc 'source /opt/ros/jazzy/setup.bash && python3 /app/smoke.py --timeout 60'
```

Pure navigation tests require only Python:

```sh
python3 -m unittest discover -s ros -p 'test_*.py'
```

Add `--patrol --timeout 240` to the smoke command to require all four waypoints and camera-assisted docking. Browser reloads restart the clock; the controller detects backwards pose timestamps and clears its patrol and sensor state.
