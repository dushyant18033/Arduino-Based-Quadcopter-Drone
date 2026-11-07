const GRAVITY = 9.81;
const MASS = 1;
const ARM = 0.22;
const MAX_MOTOR_THRUST = 7;
const YAW_ARM = 0.018;
const INERTIA_ROLL_PITCH = 0.025;
const INERTIA_YAW = 0.045;
const clamp = (value, min, max) => Math.max(min, Math.min(max, value));

export class QuadcopterSimulation {
  constructor() {
    this.reset();
  }

  reset() {
    this.position = [0, 0.04, 0];
    this.velocity = [0, 0, 0];
    this.roll = 0;
    this.pitch = 0;
    this.yaw = 0;
    this.rollRate = 0;
    this.pitchRate = 0;
    this.yawRate = 0;
    this.motorThrusts = [0, 0, 0, 0];
    this.setControls({ throttle: 0, roll: 0, pitch: 0, yaw: 0, stabilized: true });
  }

  setControls(controls) {
    this.controls = {
      throttle: clamp(controls.throttle ?? 0, 0, 100),
      roll: clamp(controls.roll ?? 0, -20, 20) * Math.PI / 180,
      pitch: clamp(controls.pitch ?? 0, -20, 20) * Math.PI / 180,
      yaw: clamp(controls.yaw ?? 0, -100, 100) / 100,
      stabilized: controls.stabilized ?? true,
    };
  }

  step(dt) {
    if (!Number.isFinite(dt) || dt <= 0) return;
    const h = Math.min(dt, 0.02);
    const { throttle, roll, pitch, yaw, stabilized } = this.controls;
    const baseThrust = MAX_MOTOR_THRUST * (throttle / 100) ** 2;

    let rollTorque;
    let pitchTorque;
    let yawTorque;
    if (stabilized) {
      rollTorque = INERTIA_ROLL_PITCH * (14 * (roll - this.roll) - 5 * this.rollRate);
      pitchTorque = INERTIA_ROLL_PITCH * (14 * (pitch - this.pitch) - 5 * this.pitchRate);
      yawTorque = INERTIA_YAW * (3 * yaw - 2 * this.yawRate);
    } else {
      rollTorque = roll / (20 * Math.PI / 180) * 0.12;
      pitchTorque = pitch / (20 * Math.PI / 180) * 0.12;
      yawTorque = yaw * 0.035;
    }

    const pitchDelta = pitchTorque / (4 * ARM);
    const rollDelta = rollTorque / (4 * ARM);
    const yawDelta = yawTorque / (4 * YAW_ARM);
    const targets = [
      baseThrust + pitchDelta - rollDelta - yawDelta,
      baseThrust + pitchDelta + rollDelta + yawDelta,
      baseThrust - pitchDelta + rollDelta - yawDelta,
      baseThrust - pitchDelta - rollDelta + yawDelta,
    ].map((thrust) => clamp(thrust, 0, MAX_MOTOR_THRUST));

    const motorResponse = 1 - Math.exp(-h / 0.035);
    this.motorThrusts = this.motorThrusts.map((current, i) => current + (targets[i] - current) * motorResponse);
    const [frontLeft, frontRight, rearRight, rearLeft] = this.motorThrusts;
    const totalThrust = frontLeft + frontRight + rearRight + rearLeft;
    const actualPitchTorque = ARM * (frontLeft + frontRight - rearRight - rearLeft);
    const actualRollTorque = ARM * (-frontLeft + frontRight + rearRight - rearLeft);
    const actualYawTorque = YAW_ARM * (-frontLeft + frontRight - rearRight + rearLeft);

    this.rollRate += (actualRollTorque / INERTIA_ROLL_PITCH - 0.7 * this.rollRate) * h;
    this.pitchRate += (actualPitchTorque / INERTIA_ROLL_PITCH - 0.7 * this.pitchRate) * h;
    this.yawRate += (actualYawTorque / INERTIA_YAW - 0.5 * this.yawRate) * h;
    this.roll += this.rollRate * h;
    this.pitch += this.pitchRate * h;
    this.yaw += this.yawRate * h;

    const horizontalX = Math.sin(this.roll) * Math.cos(this.yaw) - Math.sin(this.pitch) * Math.sin(this.yaw);
    const horizontalZ = Math.sin(this.roll) * Math.sin(this.yaw) + Math.sin(this.pitch) * Math.cos(this.yaw);
    const vertical = Math.cos(this.roll) * Math.cos(this.pitch);
    const acceleration = [
      totalThrust / MASS * horizontalX - 0.28 * this.velocity[0],
      totalThrust / MASS * vertical - GRAVITY - 0.35 * this.velocity[1],
      totalThrust / MASS * horizontalZ - 0.28 * this.velocity[2],
    ];

    this.velocity = this.velocity.map((v, i) => v + acceleration[i] * h);
    this.position = this.position.map((p, i) => p + this.velocity[i] * h);
    if (this.position[1] < 0.04) {
      this.position[1] = 0.04;
      this.velocity[1] = Math.max(0, this.velocity[1]);
      this.velocity[0] *= Math.max(0, 1 - 4 * h);
      this.velocity[2] *= Math.max(0, 1 - 4 * h);
      this.rollRate *= Math.max(0, 1 - 7 * h);
      this.pitchRate *= Math.max(0, 1 - 7 * h);
      this.yawRate *= Math.max(0, 1 - 5 * h);
      this.roll *= Math.max(0, 1 - 5 * h);
      this.pitch *= Math.max(0, 1 - 5 * h);
    }
  }

  get altitude() {
    return Math.max(0, this.position[1] - 0.04);
  }

  get motorOutputs() {
    return this.motorThrusts.map((thrust) => Math.round(1000 + 1000 * Math.sqrt(thrust / MAX_MOTOR_THRUST)));
  }
}
