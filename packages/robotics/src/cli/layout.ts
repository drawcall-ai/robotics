import { EVALUATION_TOPIC, SCORE_TOPIC } from "../evaluation.js";
import type { ModelView } from "../urdf/view.js";
const IMAGE = /\/image_raw\/compressed$/;
const SCENE = {
  meshUpAxis: "z_up",
  transforms: {
    enablePreloading: true,
    showLabel: false,
    axisScale: 0,
    lineWidth: 0,
  },
};

export interface LayoutTopics {
  scans?: string[];
  images?: string[];
  /** The `CameraInfo` topics published beside images, for calibrated image panels. */
  calibrations?: string[];
  /** Further topics to display: compressed images get an image panel, anything else the 3D panel. */
  show?: string[];
  /** Adds the score plot and the evaluation status. */
  evaluation?: boolean;
  /** One robot model each; the view frames the first. */
  models?: ModelView[];
}

/**
 * A layout with the robots, scans and shown topics in 3D, and one image panel per image topic,
 * calibrated when its camera publishes a `CameraInfo`. The view follows `map` when it is shown.
 */
export function defaultLayout({
  scans = [],
  images: cameras = [],
  calibrations = [],
  show = [],
  evaluation = false,
  models = [],
}: LayoutTopics) {
  const images = [
    ...new Set([
      ...cameras,
      ...show.filter((topic) => topic.endsWith("/compressed")),
    ]),
  ];
  const shown = show.filter((topic) => !topic.endsWith("/compressed"));
  const mapped = shown.includes("/map");
  const [model] = models;
  const scene = {
    cameraState: {
      perspective: true,
      distance: mapped ? 16 : (model?.distance ?? 2),
      targetOffset: mapped ? [0, 0, 0] : (model?.target ?? [0, 0, 0]),
      phi: 55,
      thetaOffset: 40,
    },
    followMode: "follow-pose",
    followTf: mapped ? "map" : (model?.frame ?? "base_footprint"),
    scene: SCENE,
    topics: {
      ...Object.fromEntries(shown.map((topic) => [topic, { visible: true }])),
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
    },
    layers: Object.fromEntries(
      models.map((robot, index) => [
        `robot${index}`,
        {
          layerId: "foxglove.Urdf",
          instanceId: `robot${index}`,
          label: robot.topic,
          visible: true,
          sourceType: "topic",
          topic: robot.topic,
          framePrefix: robot.framePrefix,
          displayMode: "visual",
          order: 1,
        },
      ]),
    ),
    publish: {
      type: "pose",
      poseTopic: "/goal_pose",
      pointTopic: "/clicked_point",
      poseEstimateTopic: "/initialpose",
    },
  };
  const imagePanels = Object.fromEntries(
    images.map((image) => {
      // The topic pair `CameraSensor` publishes; an image from elsewhere shows without calibration.
      const calibration = image.replace(IMAGE, "/camera_info");
      const calibrated =
        IMAGE.test(image) && calibrations.includes(calibration);
      return [
        `Image!${image}`,
        {
          imageMode: {
            imageTopic: image,
            ...(calibrated ? { calibrationTopic: calibration } : {}),
          },
          scene: SCENE,
        },
      ];
    }),
  );
  const evaluationPanels = {
    "Plot!score": {
      paths: [
        {
          value: `${SCORE_TOPIC}.data`,
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
      topicPath: EVALUATION_TOPIC,
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
  const configById: Record<string, unknown> = {
    "3D!scene": scene,
    ...imagePanels,
    ...(evaluation ? evaluationPanels : {}),
  };
  return {
    configById,
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
