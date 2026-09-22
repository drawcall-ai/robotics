// The SO101 model, from TheRobotStudio's so101_new_calib_camera.urdf (Apache-2.0). CAD link
// coordinates, metres and radians. The simulator publishes the URDF generated from it.
export const links = [
  {
    name: "base_link",
    mass: 0.147,
    visuals: [
      {
        xyz: [-0.00636471, -9.94414e-5, -0.0024],
        rpy: [1.5708, -1.67685e-15, 1.5708],
        file: "base_motor_holder_so101_v1.stl",
        scale: [1.0, 1.0, 1.0],
        material: "3d_printed",
      },
      {
        xyz: [-0.00636471, -8.97657e-9, -0.0024],
        rpy: [1.5708, -2.78073e-29, 1.5708],
        file: "base_so101_v2.stl",
        scale: [1.0, 1.0, 1.0],
        material: "3d_printed",
      },
      {
        xyz: [0.0263353, -8.97657e-9, 0.0437],
        rpy: [-8.21148e-16, 7.84513e-18, 1.249e-15],
        file: "sts3215_03a_v1.stl",
        scale: [1.0, 1.0, 1.0],
        material: "sts3215",
      },
      {
        xyz: [-0.0309827, -0.000199441, 0.0474],
        rpy: [1.5708, -1.35493e-14, 1.5708],
        file: "waveshare_mounting_plate_so101_v2.stl",
        scale: [1.0, 1.0, 1.0],
        material: "3d_printed",
      },
    ],
  },
  {
    name: "shoulder_link",
    mass: 0.100006,
    visuals: [
      {
        xyz: [-0.0303992, 0.000422241, -0.0417],
        rpy: [1.5708, 1.5708, 0.0],
        file: "sts3215_03a_v1.stl",
        scale: [1.0, 1.0, 1.0],
        material: "sts3215",
      },
      {
        xyz: [-0.0675992, -0.000177759, 0.0158499],
        rpy: [1.5708, -1.5708, 0.0],
        file: "motor_holder_so101_base_v1.stl",
        scale: [1.0, 1.0, 1.0],
        material: "3d_printed",
      },
      {
        xyz: [0.0122008, 2.22413e-5, 0.0464],
        rpy: [-1.5708, 2.35221e-33, 0.0],
        file: "rotation_pitch_so101_v1.stl",
        scale: [1.0, 1.0, 1.0],
        material: "3d_printed",
      },
    ],
  },
  {
    name: "upper_arm_link",
    mass: 0.103,
    visuals: [
      {
        xyz: [-0.11257, -0.0155, 0.0187],
        rpy: [-3.14159, -5.27356e-16, -1.5708],
        file: "sts3215_03a_v1.stl",
        scale: [1.0, 1.0, 1.0],
        material: "sts3215",
      },
      {
        xyz: [-0.065085, 0.012, 0.0182],
        rpy: [3.14159, -0.0, -1.30911e-30],
        file: "upper_arm_so101_v1.stl",
        scale: [1.0, 1.0, 1.0],
        material: "3d_printed",
      },
    ],
  },
  {
    name: "lower_arm_link",
    mass: 0.104,
    visuals: [
      {
        xyz: [-0.0648499, -0.032, 0.0182],
        rpy: [3.14159, -0.0, 6.67202e-31],
        file: "under_arm_so101_v1.stl",
        scale: [1.0, 1.0, 1.0],
        material: "3d_printed",
      },
      {
        xyz: [-0.0648499, -0.032, 0.018],
        rpy: [-3.14159, -2.55351e-15, -1.83387e-30],
        file: "motor_holder_so101_wrist_v1.stl",
        scale: [1.0, 1.0, 1.0],
        material: "3d_printed",
      },
      {
        xyz: [-0.1224, 0.0052, 0.0187],
        rpy: [-3.14159, -7.88861e-31, -3.14159],
        file: "sts3215_03a_v1.stl",
        scale: [1.0, 1.0, 1.0],
        material: "sts3215",
      },
    ],
  },
  {
    name: "wrist_link",
    mass: 0.079,
    visuals: [
      {
        xyz: [8.32667e-17, -0.0424, 0.0306],
        rpy: [1.5708, 1.5708, 0.0],
        file: "sts3215_03a_no_horn_v1.stl",
        scale: [1.0, 1.0, 1.0],
        material: "sts3215",
      },
      {
        xyz: [0.0, -0.028, 0.0181],
        rpy: [-1.5708, -1.5708, 0.0],
        file: "wrist_roll_pitch_so101_v2.stl",
        scale: [1.0, 1.0, 1.0],
        material: "3d_printed",
      },
    ],
  },
  {
    name: "gripper_link",
    mass: 0.087,
    visuals: [
      {
        xyz: [0.0077, 0.0001, -0.0234],
        rpy: [-1.5708, -5.19179e-17, -1.66533e-16],
        file: "sts3215_03a_v1.stl",
        scale: [1.0, 1.0, 1.0],
        material: "sts3215",
      },
      {
        xyz: [8.32667e-17, -0.000218214, 0.000949706],
        rpy: [-3.14159, -5.55112e-17, 0.0],
        file: "wrist_roll_follower_so101_v1.stl",
        scale: [1.0, 1.0, 1.0],
        material: "3d_printed",
      },
    ],
  },
  {
    name: "moving_jaw_so101_v1_link",
    mass: 0.012,
    visuals: [
      {
        xyz: [-5.55112e-17, -5.55112e-17, 0.0189],
        rpy: [9.53145e-17, 6.93889e-18, 1.24077e-24],
        file: "moving_jaw_so101_v1.stl",
        scale: [1.0, 1.0, 1.0],
        material: "3d_printed",
      },
    ],
  },
  {
    name: "wrist_camera_mount_link",
    mass: 0.02,
    visuals: [
      {
        xyz: [0.0, 0.0, 0.0],
        rpy: [0.0, 0.0, 0.0],
        file: "wrist_camera_mount_so101_v1.stl",
        scale: [0.001, 0.001, 0.001],
        material: "3d_printed",
      },
    ],
  },
  {
    name: "wrist_camera_link",
    mass: 0.015,
    visuals: [
      {
        xyz: [0.0, 0.0, 0.0],
        rpy: [0.0, 0.0, 0.0],
        file: "wrist_camera_so101_v1.stl",
        scale: [1.0, 1.0, 1.0],
        material: "sts3215",
      },
    ],
  },
] as const;
export const joints = [
  {
    name: "gripper",
    parent: "gripper_link",
    child: "moving_jaw_so101_v1_link",
    xyz: [0.0202, 0.0188, -0.0234],
    rpy: [1.5708, -5.24284e-8, -1.41553e-15],
    limits: [-0.174533, 1.74533],
  },
  {
    name: "wrist_roll",
    parent: "wrist_link",
    child: "gripper_link",
    xyz: [5.55112e-17, -0.0611, 0.0181],
    rpy: [1.5708, 0.0486795, 3.14159],
    limits: [-2.74385, 2.84121],
  },
  {
    name: "wrist_flex",
    parent: "lower_arm_link",
    child: "wrist_link",
    xyz: [-0.1349, 0.0052, 3.62355e-17],
    rpy: [4.02456e-15, 8.67362e-16, -1.5708],
    limits: [-1.65806, 1.65806],
  },
  {
    name: "elbow_flex",
    parent: "upper_arm_link",
    child: "lower_arm_link",
    xyz: [-0.11257, -0.028, 1.73763e-16],
    rpy: [-3.63608e-16, 8.74301e-16, 1.5708],
    limits: [-1.69, 1.69],
  },
  {
    name: "shoulder_lift",
    parent: "shoulder_link",
    child: "upper_arm_link",
    xyz: [-0.0303992, -0.0182778, -0.0542],
    rpy: [-1.5708, -1.5708, 0.0],
    limits: [-1.74533, 1.74533],
  },
  {
    name: "shoulder_pan",
    parent: "base_link",
    child: "shoulder_link",
    xyz: [0.0388353, -8.97657e-9, 0.0624],
    rpy: [3.14159, 4.18253e-17, -3.14159],
    limits: [-1.91986, 1.91986],
  },
  {
    name: "wrist_camera_mount_joint",
    parent: "gripper_link",
    child: "wrist_camera_mount_link",
    xyz: [-0.015, 0.024, -0.0315],
    rpy: [-1.5708, -0.0008, -1.5708],
    limits: null,
  },
  {
    name: "wrist_camera_joint",
    parent: "wrist_camera_mount_link",
    child: "wrist_camera_link",
    xyz: [-0.04941, -0.03813, 0.0175],
    rpy: [0.0, -1.5708, -2.0076],
    limits: null,
  },
] as const;
