import test from "node:test";
import assert from "node:assert/strict";
import { QuadcopterSimulation } from "./physics.mjs";

function run(simulation, duration, step = 0.004) {
  const count = Math.ceil(duration / step);
  for (let i = 0; i < count; i++) simulation.step(Math.min(step, duration - i * step));
}

test("rests on the ground with motors off", () => {
  const sim = new QuadcopterSimulation();
  run(sim, 2);
  assert.equal(sim.altitude, 0);
  assert.equal(sim.position[1], 0.04);
  assert.deepEqual(sim.motorOutputs, [1000, 1000, 1000, 1000]);
});

test("hover throttle approximately balances gravity", () => {
  const sim = new QuadcopterSimulation();
  sim.setControls({ throttle: 59.19, stabilized: true });
  run(sim, 3);
  assert.ok(Math.abs(sim.altitude) < 0.02, `altitude was ${sim.altitude}`);
  assert.ok(Math.abs(sim.velocity[1]) < 0.02, `vertical speed was ${sim.velocity[1]}`);
});

test("throttle above hover lifts the quadcopter", () => {
  const sim = new QuadcopterSimulation();
  sim.setControls({ throttle: 70, stabilized: true });
  run(sim, 1);
  assert.ok(sim.altitude > 1, `altitude was ${sim.altitude}`);
});

test("angle stabilization returns a disturbed attitude toward level", () => {
  const sim = new QuadcopterSimulation();
  sim.roll = 0.2;
  sim.pitch = -0.15;
  sim.setControls({ throttle: 59.19, stabilized: true });
  const initialError = Math.hypot(sim.roll, sim.pitch);
  run(sim, 2);
  assert.ok(Math.hypot(sim.roll, sim.pitch) < initialError * 0.1);
});

test("positive yaw input produces positive yaw rotation", () => {
  const sim = new QuadcopterSimulation();
  sim.setControls({ throttle: 59.19, yaw: 100, stabilized: true });
  run(sim, 1);
  assert.ok(sim.yawRate > 0, `yaw rate was ${sim.yawRate}`);
});

test("reset returns flight state and controls to safe defaults", () => {
  const sim = new QuadcopterSimulation();
  sim.setControls({ throttle: 100, roll: 20, pitch: -20, yaw: 100, stabilized: false });
  run(sim, 1);
  sim.reset();
  assert.equal(sim.altitude, 0);
  assert.equal(sim.controls.throttle, 0);
  assert.equal(sim.controls.stabilized, true);
  assert.deepEqual(sim.motorOutputs, [1000, 1000, 1000, 1000]);
});
