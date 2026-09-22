import { EVALUATION_TOPIC, SCORE_TOPIC } from "../evaluation.js";
import type { ModelView } from "../urdf/view.js";
const IMAGE = /\/image\/compressed$/;
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
  /** Adds the score plot and the evaluation status. */
  evaluation?: boolean;
  model?: ModelView;
}

/**
 * A layout for the standard topics: map, scans, costmap, plan and robot in 3D, and one image panel
 * per image topic, calibrated when its camera publishes a `CameraInfo`.
 */
export function defaultLayout({
  scans = [],
  images = [],
  calibrations = [],
  evaluation = false,
  model,
}: LayoutTopics) {
  const scene = {
    cameraState: {
      perspective: true,
      distance: scans.length ? 16 : (model?.distance ?? 2),
      targetOffset: scans.length ? [0, 0, 0] : (model?.target ?? [0, 0, 0]),
      phi: 55,
      thetaOffset: 40,
    },
    followMode: "follow-pose",
    followTf: scans.length ? "map" : (model?.frame ?? "base_footprint"),
    scene: SCENE,
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
        topic: model?.topic ?? "/robot_description",
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
