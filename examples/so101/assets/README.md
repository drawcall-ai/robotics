# SO101 model assets

RobotStudio SO101 camera model and STL meshes downloaded on 2026-09-18 from
https://github.com/TheRobotStudio/SO-ARM100/tree/main/Simulation/SO101
under Apache-2.0 (see LICENSE).

`../model.ts` holds link masses, visual transforms, joint origins and limits from the upstream
`so101_new_calib_camera.urdf`, without geometry modifications. The URDF the simulator publishes is
generated from the scene built on it.

`../hulls.ts` contains reduced convex support points sampled from the collided meshes;
it changes collision fidelity, not the visual meshes or articulated dimensions.
Mesh units and scales are in `../model.ts`, including the millimetre camera mount.
The jaw collision pads are documented in `../gripper.ts`; the camera optical frame is in `../robot.ts`.
