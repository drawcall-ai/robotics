# @drawcall/robotics

ROS 2 devices for Three.js and Drawcall physics. Connect once, attach spatial sensors to your scene, and bind ROS commands to native physics motors.

## Minimal application

This example assumes a rosbridge server at `ws://localhost:9090`. The [warehouse example](../../README.md#run-the-warehouse) supplies one in Docker.

```ts
import { JointMotor, RevoluteJoint, RigidBody } from "@drawcall/physics";
import { setupWorld } from "@drawcall/physics-rapier";
import {
  CameraSensor,
  JointStateSensor,
  MotorBinding,
  setupROS2,
} from "@drawcall/robotics";
import { PerspectiveCamera, Scene, WebGLRenderer } from "three";

const world = await setupWorld({ gravity: [0, -9.81, 0], fixedDelta: 1 / 120 });
const scene = new Scene();
const renderer = new WebGLRenderer({ antialias: true });
renderer.setSize(640, 360);
document.body.append(renderer.domElement);
const viewer = new PerspectiveCamera(60, 640 / 360, 0.1, 100);
viewer.position.set(3, 2, 3);
viewer.lookAt(0, 1, 0);

const rotor = new RigidBody({
  mass: 1,
  centerOfMass: [0, 0, 0],
  diagonalInertia: [1, 1, 1],
});
rotor.position.y = 1;
const joint = new RevoluteJoint({ body0: null, body1: rotor, axis: "Y" });
joint.position.y = 1;
scene.add(rotor, joint);
const motor = new JointMotor({ joint, damping: 10, maxForce: 20 });

const ros = await setupROS2("ws://localhost:9090", { world, scene, renderer });
new MotorBinding({ motor, name: "rotor", mode: "velocity" });
new JointStateSensor({ joints: { rotor: joint } });
const camera = new CameraSensor({
  name: "camera",
  width: 320,
  height: 180,
  rate: 5,
});
rotor.add(camera);
// Add application-owned meshes, lights and scenery as usual.

let previous = performance.now();
renderer.setAnimationLoop(() => {
  const now = performance.now();
  world.update((now - previous) / 1000);
  previous = now;
  renderer.render(scene, viewer);
});

function dispose() {
  renderer.setAnimationLoop(null);
  ros.dispose();
  world.dispose();
  renderer.dispose();
  renderer.domElement.remove();
}
```

Publish repeated `std_msgs/msg/Float64` messages to `/robot/rotor/command` to drive the motor. Positive values follow the physics joint's positive axis; velocity units are rad/s for revolute joints and m/s for prismatic joints. Commands expire after 0.5 seconds by default. Expiry **disables actuation and permits coasting**; it does not brake. Use `mode: "position"` with nonzero stiffness for position targets, respecting the backend's angular target restrictions.

## Devices and lifetime

`setupROS2` connects and captures `world`, `scene` and `renderer`. `getROS2()` retrieves the default connected runtime or throws. Device constructors register automatically with that runtime; `{ ros }` selects an explicit one. There is no scene auto-discovery and no upfront device array.

| Device             | Configuration and output                                                                                                                      |
| ------------------ | --------------------------------------------------------------------------------------------------------------------------------------------- |
| `MotorBinding`     | Existing `motor`, `name`, `mode`, optional `timeout`; subscribes to `<name>/command`                                                          |
| `CameraSensor`     | PerspectiveCamera with `name`, dimensions, `fov`, `rate`, `frameId`; publishes `<name>/image/compressed`, `<name>/camera_info` and optical TF |
| `LaserSensor`      | Object3D with `name`, `samples`, `range`, `rate`, `frameId`, `excludeBodies`; publishes scan and TF                                           |
| `JointStateSensor` | `joints` record mapping names to axis joints, optional `rate`; publishes `joint_states`                                                       |
| `PoseSensor`       | `object`, optional `rate`, `childFrameId`; publishes ground-truth `pose` and TF beneath `map`                                                 |

For multiple spatial sensors, choose distinct names and frame IDs:

```ts
const rear = new CameraSensor({ name: "rear", frameId: "rear_optical" });
rear.rotation.y = Math.PI;
rotor.add(rear);
```

Removing a sensor from its Three.js parent does not unregister it. Call `sensor.dispose()` to stop publication and release its resources. Runtime disposal disposes all registered devices and subscriptions, removes hooks and closes the connection; application-owned motors, joints, bodies, scene and renderer remain intact.

Relative topics use the default namespace `/robot`; `{ namespace: "/other_robot" }` changes it. Absolute topics bypass the namespace. Frame IDs do not automatically inherit the topic namespace. `/clock` and `/tf` remain global.

## Custom devices

Extend `Device` to inherit registration, the `ros` reference and disposal. For example, publish simulation time as a custom diagnostic:

```ts
import { Device, Schedule } from "@drawcall/robotics";

class Diagnostic extends Device {
  private readonly schedule = new Schedule(1);
  afterStep(time: number): void {
    if (this.schedule.due(time)) {
      this.ros.publish("diagnostic/time", "std_msgs/msg/Float64", {
        data: time,
      });
    }
  }
}

const diagnostic = new Diagnostic();
// Later: diagnostic.dispose(), or dispose the entire runtime.
```

Devices may implement `beforeStep(time, delta)` for commands, `afterStep(time, delta)` for measurements, and `stop()` for disconnect/fault handling. Actuator extensions should implement `stop()` and remove their own actuation. `ros.subscribe(name, type, callback)` returns an unsubscribe function; validate incoming data in the callback and release it during disposal.

For Three.js subclasses, use the same lifecycle through `withDevice`:

```ts
import { PerspectiveCamera } from "three";
import { withDevice } from "@drawcall/robotics";

class CustomCamera extends withDevice(PerspectiveCamera) {
  afterStep(time: number) {
    // Capture using this.ros.renderer and this camera.
  }
}

// args forwards the base constructor arguments.
const camera = new CustomCamera({ ros, args: [70, 16 / 9, 0.05, 100] });
scene.add(camera);
```

`new Diagnostic()` uses the default runtime; `new Diagnostic({ ros })` selects one explicitly. `args` preserves the base constructor’s tuple types and is optional when that constructor accepts no arguments. Native Three.js methods and `instanceof PerspectiveCamera` still work. If a device owns resources, override `dispose()` to release them and call `super.dispose()` to unregister. The mixin handles ROS registration only.

Registration happens in `super()`. Validate constructor inputs before it; if later initialization throws, call `super.dispose()` in the error path before rethrowing. The runtime accepts structurally compatible devices too, so inheritance is optional.

## Runtime semantics

Before each physics substep, bindings apply fresh commands. After a completed substep, `/clock` and due sensor measurements use `world.time`. Enable `use_sim_time` in ROS nodes. The display still renders at its own frame rate. ROS communication is asynchronous: no acknowledgement barrier or lockstep is implied.

Body frames map Three.js forward `-Z`, left `-X`, up `+Y` to ROS forward `+X`, left `+Y`, up `+Z`. Camera optical frames use forward `+Z`, right `+X`, down `+Y`. Exported conversion helpers are `rosPosition`, `rosRotation` and `rosOpticalRotation`.

The initial transport uses roslib/rosbridge, per-topic queues of one and a 4 MiB websocket publication threshold. Inspect `ros.connected`, `ros.error` and `ros.dropped`. Reconnection requires disposing the runtime and calling setup again. HTTPS applications need a compatible WSS endpoint.

This package currently provides direct `Float64` motor commands, not `ros2_control`. Pose is ground truth. Lidar is a simultaneous front 180° scan with maximum range for misses. Camera capture uses synchronous WebGL readback and JPEG encoding; defaults are 320×180 at 5 Hz. Reload for a fresh simulation clock. The demo controller resets on backwards sensor timestamps; other ROS nodes must handle clock jumps or restart. Physics reset alone is not coordinated with ROS.
