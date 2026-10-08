// The SO-101 follower's Feetech STS3215 servo bus as LeRobot sees it, simulated simply:
//
// - Readings and goals in LeRobot units: degrees (0 = middle of the calibrated range) for the arm,
//   0..100 for the gripper, quantised to one raw step, goals clamped to the calibrated range.
// - The readings carry a hidden, pose-dependent zero error (`Calibration`): true angle = reading +
//   error(reading). The policy never sees true angles.
// - P-only position control, torque = K·(goal − present) capped at the torque limit, K set by
//   the P coefficient (16 by default, K = 7.9 N·m/rad). Gravity makes the arm sag by degrees.
// - Stiction as a dead band: a joint at rest ignores goal changes smaller than its static
//   friction torque over K (±3° at P=16), then jumps to the full goal at once.
// - Present_Load: the commanded torque K·(present − goal), in tenths of a percent of stall torque.
// - Temperature: dT/dt = 0.3125·load² − (T − 25)/1440 s; at 70 °C the motor cuts its torque and
//   stays off until it is below 55 °C and receives a new goal. Heat follows simulated time; a
//   pause between turns charges the wall-clock time it lasted (up to 120 s) as holding time,
//   since a real arm keeps heating while its operator thinks.
//
// Topics: /follower/joint_states (readings, deg/s, Present_Load in effort),
// /follower/status (JSON: temperatures, torque on/off, P coefficients), and the inputs
// /follower/goal and /follower/p_gain (sensor_msgs/JointState, position = goal or P).
//
// Ground truth: a policy must not read this file.
import { JointDrive, type RevoluteJoint } from "@drawcall/physics";
import { getROS2, header, type ROS2 } from "@drawcall/robotics";

/** STS3215 stall torque at 12 V, N·m: 30 kg·cm. Present_Load 1000 is this torque. */
export const STALL = 2.94;
/** Joint stiffness per unit of P coefficient, N·m/rad: P=16 gives 47 load units per degree (§2). */
const STIFFNESS_PER_P = 7.9 / 16;
/** Static friction of a geared servo, N·m: §2's load hysteresis (±120 units) and a 3° step that does not move. */
const STATIC_FRICTION = 0.42;
/** How long a joint keeps following goals after it last moved, s. */
const SETTLE = 0.3;
const AMBIENT = 25;
const HEATING = 0.3125;
const COOLING = 1440;
const TRIP = 70;
const RECOVER = 55;
const RAW = 360 / 4096;

export interface ServoJoint {
  name: string;
  joint: RevoluteJoint;
  /** The calibrated range of readings: goals are clamped to it. */
  range: readonly [number, number];
  /** Fraction of stall torque the servo may exert (Max_Torque_Limit). */
  torqueLimit: number;
  /** Reading units per degree of motor rotation: 1 for the arm, 100/130 for the gripper. */
  unitsPerDegree: number;
  /** Whether stiction holds the joint at rest; off for the light, fast gripper. */
  stiction: boolean;
}

/** The hidden map between readings and true joint angles, both in a joint order. */
export interface Calibration {
  /** True joint angles, radians, for readings. */
  angles(readings: readonly number[]): number[];
  /** Readings for true joint angles. */
  readings(angles: readonly number[]): number[];
}

interface State {
  goal: number;
  /** The goal the motor actually drives to: the goal, or where stiction holds it. */
  held: number;
  moved: number;
  p: number;
  temperature: number;
  torque: boolean;
  load: number;
}

export class ServoBus {
  readonly ros: ROS2;
  readonly drives: JointDrive[];
  readonly states: State[] = [];
  private readonly unsubscribe: (() => void)[];
  private readonly unregister: () => void;
  private last = -Infinity;
  private lastStatus = -Infinity;
  private wall?: number;
  /** Listeners for audit events: trips and recoveries. */
  readonly events: ((message: string) => void)[] = [];
  readonly joints: readonly ServoJoint[];
  readonly calibration: Calibration;
  constructor(joints: readonly ServoJoint[], calibration: Calibration) {
    this.joints = joints;
    this.calibration = calibration;
    this.ros = getROS2();
    this.drives = joints.map(({ joint, torqueLimit }) => {
      const drive = new JointDrive({
        stiffness: 16 * STIFFNESS_PER_P,
        damping: 0.1,
        maxForce: torqueLimit * STALL,
        // No-load speed of the STS3215 at 12 V, 0.222 s per 60°.
        maxVelocity: 4.7,
      });
      joint.setDrive(drive);
      return drive;
    });
    this.unsubscribe = [
      this.ros.subscribe("/follower/goal", "sensor_msgs/msg/JointState", (m) =>
        this.receive(m, (state, value, i) => {
          const [low, high] = this.joints[i]!.range;
          state.goal = Math.min(high, Math.max(low, value));
          if (!state.torque && state.temperature < RECOVER) {
            state.torque = true;
            state.held = state.goal;
            this.emit(`${this.joints[i]!.name} torque on again at ${state.temperature.toFixed(0)} C`);
          }
        }),
      ),
      this.ros.subscribe("/follower/p_gain", "sensor_msgs/msg/JointState", (m) =>
        this.receive(m, (state, value) => {
          if (!(value >= 1 && value <= 64)) throw new Error("P coefficient must lie in [1, 64]");
          state.p = value;
        }),
      ),
    ];
    this.unregister = this.ros.register(this);
  }
  private emit(message: string): void {
    this.ros.log(message);
    for (const listener of this.events) listener(message);
  }
  private receive(
    message: unknown,
    apply: (state: State, value: number, index: number) => void,
  ): void {
    const { name, position } = message as { name: string[]; position: ArrayLike<number> };
    const values = Array.from(position);
    name.forEach((n, k) => {
      const i = this.joints.findIndex((j) => j.name === n);
      if (i < 0) throw new Error(`Unknown joint ${n}`);
      const value = values[k];
      if (typeof value !== "number" || !Number.isFinite(value))
        throw new Error(`Joint ${n} needs a finite value`);
      if (this.states[i]) apply(this.states[i]!, value, i);
    });
  }
  /** True joint angles and velocities, radians. */
  private measure() {
    const measured = this.joints.map(({ joint }) => joint.getState());
    return {
      angles: measured.map((m) => m.position),
      velocities: measured.map((m) => m.velocity),
    };
  }
  beforeStep(time: number, dt: number): void {
    const { angles } = this.measure();
    const present = this.calibration.readings(angles);
    // On power-up the servos hold where they are, as LeRobot's connect does.
    if (!this.states.length)
      for (const reading of present)
        this.states.push({
          goal: reading,
          held: reading,
          moved: -Infinity,
          p: 16,
          temperature: 30,
          torque: true,
          load: 0,
        });
    // A pause between turns: the arm held its pose all along.
    const now = performance.now() / 1000;
    const paused = this.wall === undefined ? 0 : Math.min(120, now - this.wall - dt);
    this.wall = now;
    const goals = this.calibration.angles(this.states.map((s) => s.goal));
    this.joints.forEach(({ name, unitsPerDegree, stiction, torqueLimit }, i) => {
      const state = this.states[i]!;
      const k = state.p * STIFFNESS_PER_P;
      const band = stiction ? (STATIC_FRICTION / k) * (180 / Math.PI) * unitsPerDegree : 0;
      if (Math.abs(state.goal - state.held) > band || time - state.moved < SETTLE) {
        if (state.held !== state.goal) state.moved = time;
        state.held = state.goal;
      }
      const torque = state.torque
        ? Math.max(-torqueLimit, Math.min(torqueLimit, (k * (goals[i]! - angles[i]!)) / STALL))
        : 0;
      // Feetech reports the load positive while pushing towards decreasing readings (§2).
      state.load = -Math.round(1000 * torque);
      const heat = (seconds: number) => {
        const t = state.temperature;
        state.temperature += seconds * (HEATING * torque * torque - (t - AMBIENT) / COOLING);
      };
      if (paused > 1) heat(paused);
      heat(dt);
      if (state.torque && state.temperature >= TRIP) {
        state.torque = false;
        this.emit(`${name} overheated at ${state.temperature.toFixed(0)} C: torque off`);
      }
    });
    const held = this.calibration.angles(this.states.map((s) => s.held));
    this.drives.forEach((drive, i) => {
      const state = this.states[i]!;
      if (!state.torque) {
        if (drive.target) drive.setTarget(undefined);
        return;
      }
      const k = state.p * STIFFNESS_PER_P;
      // The P coefficient sets the stiffness, fixed at construction; scale the target instead.
      const base = 16 * STIFFNESS_PER_P;
      const position = angles[i]! + (k / base) * (held[i]! - angles[i]!);
      if (drive.target?.position !== position) drive.setTarget({ position });
    });
  }
  afterStep(time: number): void {
    if (time - this.last < 0.02 - 1e-9) return;
    this.last = time;
    const { angles, velocities } = this.measure();
    const readings = this.calibration.readings(angles);
    const units = this.joints.map((j) => j.unitsPerDegree);
    this.ros.publish("/follower/joint_states", "sensor_msgs/msg/JointState", {
      header: header(time, ""),
      name: this.joints.map((j) => j.name),
      position: readings.map((r, i) => Math.round(r / (RAW * units[i]!)) * RAW * units[i]!),
      velocity: velocities.map((v, i) => (v * 180 * units[i]!) / Math.PI),
      effort: this.states.map((s) => s.load),
    });
    if (time - this.lastStatus < 0.5 - 1e-9) return;
    this.lastStatus = time;
    const by = <T>(f: (s: State) => T) =>
      Object.fromEntries(this.joints.map((j, i) => [j.name, f(this.states[i]!)]));
    this.ros.publish("/follower/status", "std_msgs/msg/String", {
      data: JSON.stringify({
        temperature: by((s) => Math.round(s.temperature)),
        torque: by((s) => s.torque),
        p: by((s) => s.p),
      }),
    });
  }
  dispose(): void {
    for (const release of this.unsubscribe) release();
    this.unregister();
  }
}
