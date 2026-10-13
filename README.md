# @drawcall/robotics

Simulate robots in Three.js physics scenes as ordinary ROS 2 nodes, so the same robot code runs in simulation and on hardware, and evaluate it on many randomized episodes. Robots, sensors and actuators are scene objects, the URDF comes from the same model, and an in-scene evaluator scores each episode.

```sh
npm install three @drawcall/physics @drawcall/robotics
drawcall-robotics run
```

`drawcall-robotics` runs, steps, records and evaluates episodes inside a ROS 2 environment. The package ships skills for authoring a scene and for solving a task in one.

| Example                                                                                      | What it shows                                           |
| -------------------------------------------------------------------------------------------- | ------------------------------------------------------- |
| [`examples/so101`](https://github.com/drawcall-ai/robotics/tree/main/examples/so101)         | An SO101 arm sets up chess pieces from its wrist camera |
| [`examples/warehouse`](https://github.com/drawcall-ai/robotics/tree/main/examples/warehouse) | A mobile robot navigates a warehouse with Nav2          |

The SO101 controller needs models the example does not ship; its README lists them.
