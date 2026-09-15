import "./style.css";
import { setupWorld } from "@drawcall/physics-rapier";
import {
  CameraSensor,
  JointStateSensor,
  LaserSensor,
  MotorBinding,
  setupROS2,
} from "@drawcall/robotics";
import { Vector3 } from "three";
import { createViewer } from "./viewer";
import { createRobot } from "./robot";
import { createWarehouse } from "./warehouse";
import { readStatus } from "./status";
import { drawMap } from "./map";

function element<T extends Element>(selector: string, type: { new (): T }): T {
  const result = document.querySelector(selector);
  if (!(result instanceof type))
    throw new Error(`Missing UI element: ${selector}`);
  return result;
}
const viewport = element(".viewport", HTMLElement);
const error = element("#error", HTMLParagraphElement);
const status = element("#status", HTMLParagraphElement);
const button = element("#connect", HTMLButtonElement);
const badge = element("#connection", HTMLDivElement);
const speed = element("#speed", HTMLSpanElement);
const time = element("#time", HTMLSpanElement);
const x = element("#pos-x", HTMLSpanElement);
const y = element("#pos-y", HTMLSpanElement);
function showError(value: unknown) {
  error.textContent = value instanceof Error ? value.message : String(value);
}
window.addEventListener("error", (event) =>
  showError(event.error ?? event.message),
);
window.addEventListener("unhandledrejection", (event) =>
  showError(event.reason),
);

async function start() {
  const world = await setupWorld({
    gravity: [0, -9.81, 0],
    fixedDelta: 1 / 120,
    maxSubsteps: 16,
    solverIterations: 12,
  });
  const viewer = createViewer(viewport);
  const { scene, renderer } = viewer;
  element("#overview", HTMLButtonElement).onclick = viewer.overview;
  element("#follow", HTMLButtonElement).onclick = viewer.toggleFollow;
  createWarehouse(world, scene);
  const robot = createRobot(world, scene);
  let ros: Awaited<ReturnType<typeof setupROS2>> | undefined;
  let connecting = false;
  function disconnect() {
    ros?.dispose();
    ros = undefined;
    element("#camera", HTMLDivElement).textContent =
      "Camera available after connection";
    status.textContent = "Disconnected. Motor actuation disabled.";
    speed.textContent = x.textContent = y.textContent = "—";
  }
  async function connect() {
    if (connecting) return;
    if (ros) {
      disconnect();
      return;
    }
    connecting = true;
    button.disabled = true;
    error.textContent = "";
    status.textContent = "Connecting to the ROS bridge…";
    try {
      ros = await setupROS2(element("#endpoint", HTMLInputElement).value, {
        world,
        scene,
        renderer,
        namespace: "/robot",
      });
      new MotorBinding({
        ros,
        motor: robot.left.motor,
        name: "left_wheel",
        mode: "velocity",
      });
      new MotorBinding({
        ros,
        motor: robot.right.motor,
        name: "right_wheel",
        mode: "velocity",
      });
      new JointStateSensor({
        ros,
        joints: {
          left_wheel: robot.left.joint,
          right_wheel: robot.right.joint,
        },
        rate: 50,
      });
      const camera = new CameraSensor({
        ros,
        width: 320,
        height: 180,
        fov: 70,
        rate: 5,
        frameId: "camera_optical",
      });
      camera.position.set(0, 0.28, -0.38);
      robot.chassis.add(camera);
      const laser = new LaserSensor({
        ros,
        rate: 10,
        samples: 361,
        range: 12,
        excludeBodies: robot.bodies,
      });
      laser.position.set(0, 0.24, -0.32);
      robot.chassis.add(laser);
      element("#camera", HTMLDivElement).replaceChildren(camera.preview);
      ros.subscribe("/map", "nav_msgs/msg/OccupancyGrid", (message) =>
        drawMap(element("#map", HTMLCanvasElement), message),
      );
      ros.subscribe("status", "std_msgs/msg/String", (message) => {
        const mission = readStatus(message);
        speed.textContent = mission.speed.toFixed(2);
        x.textContent = mission.pose?.[0].toFixed(2) ?? "—";
        y.textContent = mission.pose?.[1].toFixed(2) ?? "—";
        status.textContent = `${mission.state} · waypoint ${mission.waypoint + 1} / 4`;
        element("#camera-frames", HTMLElement).textContent =
          `${mission.images} frames`;
        element("#scan-frames", HTMLElement).textContent =
          `${mission.scans} scans · ${mission.maps} map updates`;
        element("#mission-target", HTMLParagraphElement).textContent =
          `Target (${mission.goal}) m in the ROS map frame.`;
        document.querySelectorAll(".route span").forEach((item, i) => {
          item.classList.toggle("active", i === mission.waypoint);
          if (i === mission.waypoint) item.setAttribute("aria-current", "step");
          else item.removeAttribute("aria-current");
        });
      });
      status.textContent = "Bridge connected. Waiting for controller feedback.";
    } catch (cause) {
      disconnect();
      showError(cause);
      status.textContent =
        "Connection failed. Check the Docker stack and endpoint.";
    } finally {
      connecting = false;
      button.disabled = false;
    }
  }
  button.onclick = () => {
    void connect();
  };
  let last: number | undefined;
  const position = new Vector3();
  renderer.setAnimationLoop((now) => {
    const delta = last === undefined ? 0 : (now - last) / 1000;
    last = now;
    try {
      world.update(delta);
    } catch (cause) {
      renderer.setAnimationLoop(null);
      showError(ros?.error ?? cause);
      return;
    }
    robot.chassis.getWorldPosition(position);
    viewer.render(position);
    time.textContent = world.time.toFixed(1);
    if (ros && !ros.connected) {
      const fault = ros.error;
      disconnect();
      showError(fault ?? "ROS bridge disconnected. Wheel targets stopped.");
    }
    const connected = ros?.connected ?? false;
    badge.classList.toggle("online", connected);
    badge.innerHTML = `<i></i>${connected ? "CONNECTED" : "DISCONNECTED"}`;
    button.textContent = connected
      ? "Disconnect"
      : connecting
        ? "Connecting…"
        : "Connect to ROS 2 ↗";
  });
  window.addEventListener(
    "pagehide",
    () => {
      renderer.setAnimationLoop(null);
      disconnect();
      world.dispose();
      viewer.dispose();
    },
    { once: true },
  );
  if (new URLSearchParams(location.search).get("connect") === "1")
    await connect();
}
void start().catch(showError);
