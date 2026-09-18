const IMAGE = /\/image\/compressed$/;

/** A layout for the standard topics: map, scans, costmap, plan and robot in 3D, one image panel per camera. */
export function defaultLayout(
  scans: string[],
  images: string[],
  evaluation = false,
) {
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
  const evaluationPanels = {
    "Plot!score": {
      paths: [
        {
          value: "/sim/score.data",
          enabled: true,
          timestampMethod: "receiveTime",
          showLine: false,
          lineSize: 3,
          label: "Score",
          color: "#f7ab39",
        },
      ],
      xAxisVal: "timestamp",
      xAxisLabel: "Recording time (s)",
      yAxisLabel: "Score",
      minYValue: 0,
      showXAxisLabels: true,
      showYAxisLabels: true,
      showLegend: true,
      showPlotValuesInLegend: true,
      isSynced: true,
    },
    "RawMessages!evaluation": {
      topicPath: "/sim/evaluation",
      diffEnabled: false,
    },
  };
  let column: Panel | undefined;
  for (const image of [...images].reverse()) {
    const panel = `Image!${image}`;
    column =
      column === undefined
        ? panel
        : { direction: "column", first: panel, second: column };
  }
  if (evaluation) {
    const details: Panel = column
      ? {
          direction: "column",
          first: column,
          second: "RawMessages!evaluation",
          splitPercentage: 60,
        }
      : "RawMessages!evaluation";
    column = {
      direction: "column",
      first: "Plot!score",
      second: details,
      splitPercentage: 40,
    };
  }
  const evaluated: Partial<typeof evaluationPanels> = evaluation
    ? evaluationPanels
    : {};
  return {
    configById: Object.assign({ "3D!scene": scene }, imagePanels, evaluated),
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

type Panel =
  | string
  | {
      direction: "column";
      first: Panel;
      second: Panel;
      splitPercentage?: number;
    };
