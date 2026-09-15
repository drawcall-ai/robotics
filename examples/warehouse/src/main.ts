import "./style.css";
import { setupWorld } from "@drawcall/physics-rapier";
import {
  CameraSensor,
  JointStateSensor,
  LaserSensor,
  MotorBinding,
  PoseSensor,
  setupROS2,
} from "@drawcall/robotics";
import {
  AmbientLight,
  Color,
  DirectionalLight,
  Fog,
  HemisphereLight,
  PerspectiveCamera,
  Scene,
  SRGBColorSpace,
  Vector3,
  WebGLRenderer,
  PCFShadowMap,
} from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { createRobot } from "./robot";
import { createWarehouse } from "./warehouse";
import { readStatus } from "./status";

function element<T extends Element>(selector: string, type: { new (): T }): T {
  const result = document.querySelector(selector);
  if (!(result instanceof type))
    throw new Error(`Missing UI element: ${selector}`);
  return result;
}
const app = element("#app", HTMLDivElement);
app.innerHTML = `
<header><div class="brand"><div class="mark">d.</div><div><h1>drawcall <span style="color:#82999d">/ robotics</span></h1><small>Simulation workspace</small></div></div><div class="header-right"><span>THREE.JS × ROS 2</span><div class="badge" id="connection"><i></i>DISCONNECTED</div></div></header>
<main><section class="viewport" aria-label="Interactive warehouse simulation"><div class="scene-heading"><div class="eyebrow">Environment / 001</div><h2>The warehouse</h2><p>A browser world. A real ROS 2 control loop.</p></div><div class="view-tools"><button id="overview">Overview</button><button id="follow">Follow robot</button></div><div class="scene-footer"><span><strong>DRAG</strong> orbit</span><span><strong>SCROLL</strong> zoom</span><span><strong>RIGHT DRAG</strong> pan</span></div></section>
<aside class="aside"><section class="panel"><div class="eyebrow">ROS bridge</div><h3>Connect the controller</h3><label class="eyebrow" for="endpoint">WebSocket endpoint</label><input id="endpoint" class="endpoint" value="ws://localhost:9090" spellcheck="false"><button id="connect" class="connect">Connect to ROS 2 ↗</button><p class="hint">Start the Docker stack, then connect. ROS publishes wheel commands; native physics moves the robot.</p><p id="error" class="error" role="alert"></p></section>
<section class="panel"><div class="eyebrow">Robot / DC–01</div><h3>Differential drive</h3><p id="status" class="status">Standing by for a ROS controller.</p><div class="metrics"><div class="metric"><span class="value" id="speed">0.00</span><span class="unit">m/s</span><label>Ground speed</label></div><div class="metric"><span class="value" id="time">0.0</span><span class="unit">s</span><label>Simulation time</label></div><div class="metric"><span class="value" id="pos-x">0.00</span><span class="unit">m</span><label>ROS position X</label></div><div class="metric"><span class="value" id="pos-y">0.00</span><span class="unit">m</span><label>ROS position Y</label></div></div></section>
<section class="panel"><div class="eyebrow">Sensor / RGB</div><h3>Onboard camera</h3><div class="camera" id="camera">Camera available after connection</div><div class="legend"><span>320 × 180 · 5 Hz</span><b id="camera-frames">No frames</b></div></section>
<section class="panel"><div class="eyebrow">Mission / patrol</div><h3>Four-point circuit</h3><div class="route"><span>01</span><span>02</span><span>03</span><span>04</span></div><p class="hint">Pose feedback → waypoint controller → wheel velocity commands.</p><p class="hint" id="mission-target">Waiting for mission telemetry.</p><div class="legend"><span>Laser scan</span><b id="scan-frames">61 rays · 10 Hz</b></div></section></aside></main>
<footer><span><b>Native physics</b> · fixed substeps · simulation clock</span><span>ROS 2 / Docker</span></footer>`;
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
  const scene = new Scene();
  scene.background = new Color("#cbd2c9");
  scene.fog = new Fog("#cbd2c9", 20, 48);
  scene.add(
    new HemisphereLight("#ecf6ee", "#617471", 2.6),
    new AmbientLight("#ffffff", 0.35),
  );
  const sun = new DirectionalLight("#fff5dd", 3.5);
  sun.position.set(0, 12, 5);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  Object.assign(sun.shadow.camera, {
    left: -12,
    right: 12,
    top: 12,
    bottom: -12,
    near: 0.1,
    far: 40,
  });
  sun.shadow.normalBias = 0.025;
  sun.target.position.set(-1, 0, -2);
  scene.add(sun, sun.target);
  const renderer = new WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.outputColorSpace = SRGBColorSpace;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = PCFShadowMap;
  viewport.prepend(renderer.domElement);
  const view = new PerspectiveCamera(43, 1, 0.1, 100);
  const controls = new OrbitControls(view, renderer.domElement);
  controls.enableDamping = true;
  controls.maxPolarAngle = Math.PI / 2.1;
  controls.minDistance = 3;
  controls.maxDistance = 28;
  let follow = false;
  function overview() {
    follow = false;
    view.position.set(10, 11, 12);
    controls.target.set(-1.4, 0.25, -2.4);
    controls.update();
  }
  overview();
  element("#overview", HTMLButtonElement).onclick = overview;
  element("#follow", HTMLButtonElement).onclick = () => {
    follow = !follow;
  };
  const resize = new ResizeObserver(() => {
    const { width, height } = viewport.getBoundingClientRect();
    renderer.setSize(width, height);
    view.aspect = width / height;
    view.updateProjectionMatrix();
  });
  resize.observe(viewport);
  createWarehouse(world, scene);
  const robot = createRobot(world, scene);
  let ros: Awaited<ReturnType<typeof setupROS2>> | undefined;
  let devices: { dispose(): void }[] = [];
  let unsubscribe: (() => void) | undefined;
  let connecting = false;
  function disconnect() {
    unsubscribe?.();
    unsubscribe = undefined;
    for (const device of devices) device.dispose();
    devices = [];
    ros?.dispose();
    ros = undefined;
    robot.left.motor.setTarget({ velocity: 0 });
    robot.right.motor.setTarget({ velocity: 0 });
    element("#camera", HTMLDivElement).textContent =
      "Camera available after connection";
    status.textContent = "Disconnected. Wheel targets stopped.";
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
      devices.push(
        new MotorBinding({
          motor: robot.left.motor,
          name: "left_wheel",
          mode: "velocity",
        }),
      );
      devices.push(
        new MotorBinding({
          motor: robot.right.motor,
          name: "right_wheel",
          mode: "velocity",
        }),
      );
      devices.push(
        new JointStateSensor({
          joints: {
            left_wheel: robot.left.joint,
            right_wheel: robot.right.joint,
          },
          rate: 20,
        }),
      );
      devices.push(
        new PoseSensor({
          object: robot.chassis,
          frameId: "map",
          childFrameId: "base_link",
          rate: 20,
        }),
      );
      const camera = new CameraSensor({
        width: 320,
        height: 180,
        fov: 70,
        rate: 5,
        frameId: "camera_optical",
      });
      camera.position.set(0, 0.28, -0.38);
      robot.chassis.add(camera);
      devices.push(camera);
      const laser = new LaserSensor({
        rate: 10,
        samples: 61,
        range: 6,
        excludeBodies: robot.bodies,
      });
      laser.position.set(0, 0.52, 0.06);
      robot.chassis.add(laser);
      devices.push(laser);
      element("#camera", HTMLDivElement).replaceChildren(camera.preview);
      unsubscribe = ros.subscribe(
        "status",
        "std_msgs/msg/String",
        (message) => {
          const mission = readStatus(message);
          status.textContent = `${mission.state} · waypoint ${mission.waypoint + 1} / 4`;
          element("#camera-frames", HTMLElement).textContent =
            `${mission.images} frames · orange ${(mission.orange * 100).toFixed(1)}%`;
          element("#scan-frames", HTMLElement).textContent =
            `${mission.scans} scans · ${mission.distance.toFixed(2)} m ahead`;
          element("#mission-target", HTMLParagraphElement).textContent =
            `Target (${mission.goal}) m in the ROS map frame.`;
          document.querySelectorAll(".route span").forEach((item, i) => {
            item.classList.toggle("active", i === mission.waypoint);
            if (i === mission.waypoint)
              item.setAttribute("aria-current", "step");
            else item.removeAttribute("aria-current");
          });
        },
      );
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
  let previousConnected = false;
  const position = new Vector3();
  const followOffset = new Vector3(3, 3, 4);
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
    if (follow) {
      controls.target.lerp(position, 0.08);
      view.position.lerp(position.clone().add(followOffset), 0.035);
    }
    controls.update();
    renderer.render(scene, view);
    const velocity = robot.chassis.getVelocity().linear;
    speed.textContent = Math.hypot(velocity.x, velocity.z).toFixed(2);
    time.textContent = world.time.toFixed(1);
    x.textContent = (-position.z).toFixed(2);
    y.textContent = (-position.x).toFixed(2);
    const connected = ros?.connected ?? false;
    const fault = ros?.error;
    if (fault || (previousConnected && !connected)) {
      disconnect();
      showError(fault ?? "ROS bridge disconnected. Wheel targets stopped.");
    }
    previousConnected = connected;
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
      resize.disconnect();
      controls.dispose();
      world.dispose();
      renderer.dispose();
    },
    { once: true },
  );
  if (new URLSearchParams(location.search).get("connect") === "1")
    await connect();
}
void start().catch(showError);
