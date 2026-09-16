import { expect, it } from "vitest";
import { defaultLayout } from "../src/cli/layout.js";

type Layout = {
  configById: Record<
    string,
    { topics?: Record<string, unknown>; imageMode?: Record<string, string> }
  >;
  layout: unknown;
};

it("shows every scan in the 3D panel and gives each image its own panel", () => {
  const { configById } = defaultLayout(
    ["/robot/scan", "/rear/scan"],
    ["/robot/camera/image/compressed", "/other/image"],
  ) as Layout;
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
  expect((defaultLayout(["/robot/scan"], []) as Layout).layout).toBe(
    "3D!scene",
  );
});
