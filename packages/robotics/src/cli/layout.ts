const IMAGE = /\/image\/compressed$/;

/** A layout for the standard topics: map, scans, costmap, plan and robot in 3D, one image panel per camera. */
export function defaultLayout(scans: string[], images: string[]) {
  const scene = {
    cameraState: {
      perspective: true,
      distance: 16,
      phi: 55,
      thetaOffset: 40,
    },
    followMode: "follow-pose",
    followTf: "map",
    scene: { transforms: { enablePreloading: true } },
    topics: {
      "/map": { visible: true, colorMode: "costmap" },
      ...Object.fromEntries(
        scans.map((scan) => [
          scan,
          {
            visible: true,
            colorMode: "flat",
            flatColor: "#f7ab39",
            pointSize: 4,
          },
        ]),
      ),
      "/local_costmap/costmap": { visible: true, colorMode: "costmap" },
      "/plan": { visible: true, lineWidth: 0.05 },
      "/local_plan": { visible: true, lineWidth: 0.03 },
    },
    layers: {
      robot: {
        layerId: "foxglove.Urdf",
        instanceId: "robot",
        label: "Robot",
        visible: true,
        sourceType: "topic",
        topic: "/robot_description",
        displayMode: "visual",
        order: 1,
      },
    },
    publish: {
      type: "pose",
      poseTopic: "/goal_pose",
      pointTopic: "/clicked_point",
      poseEstimateTopic: "/initialpose",
    },
  };
  const imagePanels: Record<
    `Image!${string}`,
    {
      imageMode: { imageTopic: string; calibrationTopic?: string };
      scene: { transforms: { enablePreloading: boolean } };
    }
  > = {};
  for (const image of images) {
    const imageMode: { imageTopic: string; calibrationTopic?: string } = {
      imageTopic: image,
    };
    // The topic pair `CameraSensor` publishes; an image from elsewhere shows without calibration.
    if (IMAGE.test(image))
      imageMode.calibrationTopic = image.replace(IMAGE, "/camera_info");
    imagePanels[`Image!${image}`] = {
      imageMode,
      scene: { transforms: { enablePreloading: true } },
    };
  }
  let column: Panel | undefined;
  for (const image of [...images].reverse()) {
    const panel = `Image!${image}`;
    column =
      column === undefined
        ? panel
        : { direction: "column", first: panel, second: column };
  }
  return {
    configById: Object.assign({ "3D!scene": scene }, imagePanels),
    globalVariables: {},
    userNodes: {},
    playbackConfig: { speed: 1 },
    layout:
      column !== undefined
        ? {
            direction: "row",
            first: "3D!scene",
            second: column,
            splitPercentage: 65,
          }
        : "3D!scene",
  };
}

type Panel = string | { direction: "column"; first: Panel; second: Panel };
