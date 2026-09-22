import { expect, it } from "vitest";
import { defaultLayout } from "../src/cli/layout.js";

it("shows every scan in the 3D panel and gives each image its own panel", () => {
  const { configById, layout } = defaultLayout({
    scans: ["/robot/scan", "/rear/scan"],
    images: ["/robot/camera/image_raw/compressed", "/other/image"],
    calibrations: ["/robot/camera/camera_info"],
    show: ["/robot/perception/image/compressed", "/map", "/plan"],
  });
  expect(layout).toEqual({
    direction: "row",
    first: "3D!scene",
    second: {
      direction: "column",
      first: "Image!/robot/camera/image_raw/compressed",
      second: {
        direction: "column",
        first: "Image!/other/image",
        second: "Image!/robot/perception/image/compressed",
      },
    },
    splitPercentage: 65,
  });
  for (const topic of ["/robot/scan", "/rear/scan", "/map", "/plan"])
    expect(configById["3D!scene"]).toHaveProperty(["topics", topic]);
  expect(configById["Image!/robot/camera/image_raw/compressed"]).toHaveProperty(
    "imageMode",
    {
      imageTopic: "/robot/camera/image_raw/compressed",
      calibrationTopic: "/robot/camera/camera_info",
    },
  );
  expect(configById["Image!/other/image"]).toHaveProperty("imageMode", {
    imageTopic: "/other/image",
  });
  // An annotated image without its own CameraInfo must not wait for calibration.
  expect(configById["Image!/robot/perception/image/compressed"]).toHaveProperty(
    "imageMode",
    { imageTopic: "/robot/perception/image/compressed" },
  );
});

it("is the single 3D panel without images", () => {
  expect(defaultLayout({ scans: ["/robot/scan"] }).layout).toBe("3D!scene");
});

it("puts a single image beside the 3D panel without an empty split", () => {
  expect(defaultLayout({ images: ["/camera"] }).layout).toEqual({
    direction: "row",
    first: "3D!scene",
    second: "Image!/camera",
    splitPercentage: 65,
  });
});

it("plots numeric score on the shared recording timeline and shows final status", () => {
  const { configById, layout } = defaultLayout({ evaluation: true });
  expect(configById["Plot!score"]).toMatchObject({
    xAxisVal: "timestamp",
    isSynced: true,
    paths: [
      {
        value: "/sim/score.data",
        showLine: false,
      },
    ],
  });
  expect(JSON.stringify(layout)).toContain("Plot!score");
  expect(JSON.stringify(layout)).toContain("RawMessages!evaluation");
});

it("frames the map when it is shown and draws every robot with its frame prefix", () => {
  const model = (topic: string, framePrefix: string) => ({
    topic,
    framePrefix,
    frame: `${framePrefix}base_footprint`,
    distance: 1,
    target: [0, 0, 0],
  });
  const arms = [
    model("/arm1/robot_description", "arm1/"),
    model("/arm2/robot_description", "arm2/"),
  ];
  const { configById } = defaultLayout({ models: arms });
  expect(configById["3D!scene"]).toMatchObject({
    followTf: "arm1/base_footprint",
    layers: {
      robot0: { topic: "/arm1/robot_description", framePrefix: "arm1/" },
      robot1: { topic: "/arm2/robot_description", framePrefix: "arm2/" },
    },
  });
  const mapped = defaultLayout({ models: arms, show: ["/map"] }).configById;
  expect(mapped["3D!scene"]).toMatchObject({
    followTf: "map",
    topics: { "/map": { visible: true } },
  });
});
