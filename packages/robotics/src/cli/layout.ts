/** A layout for the standard topics: map, scans, costmap, plan and robot in 3D, one image panel per camera. */
export function defaultLayout(
  lasers: { topic: string }[],
  cameras: { name: string; imageTopic: string; infoTopic: string }[],
): unknown {
  const configById: Record<string, unknown> = {
    "3D!scene": {
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
          lasers.map((laser) => [
            laser.topic,
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
    },
  };
  for (const { name, imageTopic, infoTopic } of cameras)
    configById[`Image!${name}`] = {
      imageMode: { imageTopic, calibrationTopic: infoTopic },
      scene: { transforms: { enablePreloading: true } },
    };
  const column = (ids: string[]): unknown =>
    ids.length === 1
      ? ids[0]
      : { direction: "column", first: ids[0], second: column(ids.slice(1)) };
  const images = cameras.map(({ name }) => `Image!${name}`);
  return {
    configById,
    globalVariables: {},
    userNodes: {},
    playbackConfig: { speed: 1 },
    layout: images.length
      ? {
          direction: "row",
          first: "3D!scene",
          second: column(images),
          splitPercentage: 65,
        }
      : "3D!scene",
  };
}
