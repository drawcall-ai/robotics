import { Box3, Vector3 } from "three";
import type { Robot } from "../devices/robot.js";
import { rosPosition } from "../coordinates.js";

export interface ModelView {
  topic: string;
  /** Prefix of the robot's TF frames, empty for plain names. */
  framePrefix: string;
  frame: string;
  distance: number;
  target: number[];
}

/** Saved in an XML comment so live and recorded descriptions carry identical framing. */
export function modelView(robot: Robot): ModelView {
  robot.updateWorldMatrix(true, true);
  const offset = new Vector3().setFromMatrixPosition(robot.tree.footprint);
  const box = new Box3().setFromObject(robot);
  const center = box.isEmpty()
    ? new Vector3()
    : box
        .getCenter(new Vector3())
        .sub(robot.base.getWorldPosition(new Vector3()))
        .add(offset);
  const size = box.isEmpty() ? 1 : box.getSize(new Vector3()).length();
  return {
    topic: robot.descriptionTopic,
    framePrefix: robot.framePrefix,
    frame: robot.frame("base_footprint"),
    distance: Math.max(0.3, size * 2),
    target: rosPosition(center).toArray(),
  };
}

export function descriptionView(xml: string): ModelView | undefined {
  const match = /<!-- drawcall-view: (.*?) -->/.exec(xml);
  if (!match?.[1]) return undefined;
  const value: unknown = JSON.parse(match[1]);
  if (
    typeof value !== "object" ||
    value === null ||
    !("topic" in value) ||
    typeof value.topic !== "string" ||
    !("framePrefix" in value) ||
    typeof value.framePrefix !== "string" ||
    !("frame" in value) ||
    typeof value.frame !== "string" ||
    !("distance" in value) ||
    typeof value.distance !== "number" ||
    !Number.isFinite(value.distance) ||
    value.distance <= 0 ||
    !("target" in value) ||
    !Array.isArray(value.target) ||
    value.target.length !== 3 ||
    !value.target.every(
      (n: unknown) => typeof n === "number" && Number.isFinite(n),
    )
  )
    throw new Error("Invalid robot view metadata");
  return {
    topic: value.topic,
    framePrefix: value.framePrefix,
    frame: value.frame,
    distance: value.distance,
    target: value.target.map(Number),
  };
}
