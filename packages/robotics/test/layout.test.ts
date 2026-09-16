import { expect, it } from "vitest";
import { defaultLayout } from "../src/cli/layout.js";

it("shows every scan in the 3D panel and gives each image its own panel", () => {
  const { configById, layout } = defaultLayout(
    ["/robot/scan", "/rear/scan"],
    ["/robot/camera/image/compressed", "/other/image"],
  );
  expect(layout).toEqual({
    direction: "row",
    first: "3D!scene",
    second: {
      direction: "column",
      first: "Image!/robot/camera/image/compressed",
      second: "Image!/other/image",
    },
    splitPercentage: 65,
  });
  expect(Object.keys(configById["3D!scene"]?.topics ?? {})).toEqual(
    expect.arrayContaining(["/robot/scan", "/rear/scan"]),
  );
  expect(configById["Image!/robot/camera/image/compressed"]?.imageMode).toEqual(
    {
      imageTopic: "/robot/camera/image/compressed",
      calibrationTopic: "/robot/camera/camera_info",
    },
  );
  expect(configById["Image!/other/image"]?.imageMode).toEqual({
    imageTopic: "/other/image",
  });
});

it("is the single 3D panel without images", () => {
  expect(defaultLayout(["/robot/scan"], []).layout).toBe("3D!scene");
});

it("puts a single image beside the 3D panel without an empty split", () => {
  expect(defaultLayout([], ["/camera"]).layout).toEqual({
    direction: "row",
    first: "3D!scene",
    second: "Image!/camera",
    splitPercentage: 65,
  });
});
