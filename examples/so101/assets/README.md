# SO101 model assets

RobotStudio SO101 camera model and STL meshes downloaded on 2026-09-18 from
https://github.com/TheRobotStudio/SO-ARM100/tree/main/Simulation/SO101
under Apache-2.0 (see LICENSE). `so101.urdf` is the upstream
`so101_new_calib_camera.urdf`, retained without geometry modifications.

`../model.ts` extracts link masses, visual transforms, joint origins and limits.
`../hulls.ts` contains reduced convex support points sampled from these meshes;
it changes collision fidelity, not the visual meshes or articulated dimensions.
Mesh units and scales come from the URDF, including the millimetre camera mount.
The jaw collision pads are documented in `../gripper.ts`; the camera optical frame is in `../robot.ts`.
