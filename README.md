# Drawcall Robotics

Simulate robots in Three.js physics scenes as ordinary ROS 2 nodes, so the same robot code runs in simulation and on hardware, and evaluate it on many randomized episodes.

- [packages/robotics](packages/robotics/README.md): the library and the `drawcall-robotics` CLI.
- [examples/so101](examples/so101/README.md): an SO101 arm sets up chess pieces from its wrist camera.
- [examples/warehouse](examples/warehouse/README.md): a mobile robot navigates a warehouse with Nav2.

```sh
cd examples/so101 && pnpm start
```

The SO101 controller needs the models listed in [examples/so101/README.md](examples/so101/README.md), which are not shipped.
