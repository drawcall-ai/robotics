---
name: robotics
description: Author a robot simulation with @drawcall/robotics — a .glts scene whose robots, sensors and actuators are ROS 2 devices, plus the launch files and evaluator that make it an episode the CLI (npx @drawcall/robotics) runs and scores. Use when creating or changing a scene folder, a robot model, its sensors or its evaluator, or exporting its URDF.
---

# @drawcall/robotics

A scene is a GLTS asset, a `.glts` script that builds a Three.js scene (the glts skill covers the format), which the CLI runs as a ROS 2 node. The robot's own software runs beside it as ordinary ROS 2 nodes and only sees topics, so the same software can drive the real robot. Physics objects come from `@drawcall/physics`, which ships its own skill; the CLI simulates them with MuJoCo.

## A scene folder

```
scene.glts           the world, the robot and the evaluator
bringup.launch.xml   the robot's persistent stack (drivers, ros2_control, Nav2, ...)
policy.launch.xml    the robot's decision-making; its exit ends the turn
pixi.toml            the ROS 2 environment the CLI runs in
```

Run the CLI from that folder inside the ROS environment, for example `pixi run npx @drawcall/robotics run`. Both launch files take a `use_sim_time` argument and pass it to their nodes; a bringup with nothing to start is an empty `<launch>`. Every node the robot's software needs belongs in one of the two files, never in the scene.

## Writing scene.glts

```ts
import { scene } from "@drawcall/glts"; // the CLI simulates what is under it
import { BoxCollider, RevoluteJoint, RigidBody } from "@drawcall/physics";
import {
  CameraSensor,
  Evaluator,
  getROS2,
  JointActuator,
  Robot,
} from "@drawcall/robotics";

getROS2().show.push("/robot/camera/image_raw/compressed"); // extra viewer panels

const robot = new Robot();
robot.name = "arm";
scene.add(robot);
const base = new RigidBody({ bodyType: "static" });
base.name = "base_link";
const link = new RigidBody({ mass: 0.2 });
link.name = "link";
link.add(new BoxCollider({ size: [0.05, 0.2, 0.05] }));
const joint = new RevoluteJoint({ body0: base, body1: link, axis: "Z" });
joint.name = "shoulder";
joint.setDrive(
  new JointActuator({
    interface: "position",
    stiffness: 20,
    damping: 1,
    maxForce: 3,
  }),
);
link.add(new CameraSensor({ name: "camera", width: 640, height: 480 }));
robot.add(base, link, joint);

new Evaluator({
  timeout: 120,
  success: { raised: () => joint.getState().position > 1 },
  score: ({ status }) => (status === "success" ? 100 : 0),
});
```

- **The robot is its group.** Bodies, named joints and sensors under a `Robot` make up its URDF, joint states and TF; a joint driven by a `JointActuator` takes commands on the robot's `joint_commands` topic, as topic-based ros2_control publishes them. Name every body and joint as the URDF should.
- **Devices register themselves** with the runtime the CLI creates before it loads the scene, so constructing them in the script is enough. The scene shares the CLI's `three`, `@drawcall/physics` and `@drawcall/robotics`; other packages resolve from the scene's folder.
- **Don't build a physics world** or step anything: the CLI builds the world over `scene`, steps it, runs `onFrame` callbacks before each step and publishes `/clock`.
- **The evaluator is the episode.** `success` and `failure` conditions end it, `score` grades it, `metrics` are recorded every step. Judge from simulator truth there, never from what the robot reports.
- **The runtime holds viewer and recording settings.** `getROS2().show` lists extra topics the viewer shows; `recordExclude` is a regular expression of topics the bag leaves out, for subscribers that disturb the robot's stack.
- **Hide what the real robot could not know.** `private: true` on a `Robot` or `Evaluator` records its topics in the bag without publishing them to the robot's software; use it for ground truth the policy must not read.
- **Randomize from the episode's seed.** The CLI sets `SEED` for each episode; draw every random choice from it so a seed replays the same world.
- **ROS frames differ from Three.js.** Three.js is Y-up, ROS is Z-up with X forward; convert with `rosPosition`, `rosRotation` and `rosPose` wherever a value crosses to ROS.

## Behavior that is easy to get wrong

- An actuator holds the pose the scene authored until it is first commanded. Once commanded, a command older than 0.5 simulated seconds releases the joint, the way a real servo bus times out. Controllers must keep publishing.
- Everything outside `scene` is not simulated: an object added to another root, or created but never added, throws when commanded.
- The simulator runs at real time by default. Slow scene code (heavy per-step metrics, synchronous I/O) makes episodes fall behind; batches end an episode that falls 2 s behind.

## Other commands

`npx @drawcall/robotics urdf scene.glts` prints the robot's URDF for tools that need it outside a run. `cat` prints a run's recording as JSON lines and `view` opens it in Lichtblick.
