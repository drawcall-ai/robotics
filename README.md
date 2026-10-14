# @drawcall/robotics

Simulate robots in Three.js physics scenes as ordinary ROS 2 nodes, so the same robot code runs in simulation and on hardware, and evaluate it on many randomized episodes. Robots, sensors and actuators are scene objects, the URDF comes from the same model, and an in-scene evaluator scores each episode.

```sh
npm install three @drawcall/physics @drawcall/robotics
npx drawcall robotics run
```

A scene is a [GLTS](https://github.com/drawcall-ai/glts) asset, `scene.glts`; the CLI runs, steps, records and evaluates its episodes inside a ROS 2 environment. The package ships a skill for authoring a scene.

| Example                                                                                      | What it shows                                           |
| -------------------------------------------------------------------------------------------- | ------------------------------------------------------- |
| [`examples/so101`](https://github.com/drawcall-ai/robotics/tree/main/examples/so101)         | An SO101 arm sets up chess pieces from its wrist camera |
| [`examples/warehouse`](https://github.com/drawcall-ai/robotics/tree/main/examples/warehouse) | A mobile robot navigates a warehouse with Nav2          |

The SO101 controller needs models the example does not ship; its README lists them.
