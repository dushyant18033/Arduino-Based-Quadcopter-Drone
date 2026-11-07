import { QuadcopterSimulation } from "./physics.mjs";

const mount = document.querySelector("#viewport");
const canvas = document.createElement("canvas");
canvas.setAttribute("aria-label", "Interactive perspective view of the simulated quadcopter");
mount.appendChild(canvas);
const context = canvas.getContext("2d");
const simulation = new QuadcopterSimulation();
const inputs = {
  throttle: document.querySelector("#throttle"),
  pitch: document.querySelector("#pitch"),
  roll: document.querySelector("#roll"),
  yaw: document.querySelector("#yaw"),
  stabilized: document.querySelector("#stabilized"),
};
const outputs = {
  throttle: document.querySelector("#throttle-value"),
  pitch: document.querySelector("#pitch-value"),
  roll: document.querySelector("#roll-value"),
  yaw: document.querySelector("#yaw-value"),
};
const sceneState = document.querySelector("#flight-state");
const altitude = document.querySelector("#altitude");
const verticalSpeed = document.querySelector("#vertical-speed");
const attitude = document.querySelector("#attitude");
const pauseButton = document.querySelector("#pause-button");
const modeLabel = document.querySelector("#mode-label");
const rotors = [
  [-0.22, -0.22], [0.22, -0.22], [0.22, 0.22], [-0.22, 0.22],
];
let width = 0;
let height = 0;
let pixelRatio = 1;
let cameraTheta = 0.68;
let cameraElevation = 0.57;
let cameraDistance = 8.6;
let target = [0, 0.8, 0];
let paused = false;
let lastFrame = 0;
let lastTelemetry = 0;
let heldKeys = new Set();
let pointer = null;
let rotorAngles = [0, 0, 0, 0];

const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const subtract = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const scale = (a, n) => [a[0] * n, a[1] * n, a[2] * n];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const normalize = (a) => scale(a, 1 / Math.hypot(...a));

function resize() {
  width = mount.clientWidth;
  height = mount.clientHeight;
  if (!width || !height) return;
  pixelRatio = Math.min(window.devicePixelRatio || 1, 2);
  canvas.width = Math.round(width * pixelRatio);
  canvas.height = Math.round(height * pixelRatio);
  context.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
}

function cameraBasis() {
  const horizontal = Math.cos(cameraElevation) * cameraDistance;
  const camera = add(target, [
    Math.sin(cameraTheta) * horizontal,
    Math.sin(cameraElevation) * cameraDistance,
    Math.cos(cameraTheta) * horizontal,
  ]);
  const forward = normalize(subtract(target, camera));
  const right = normalize(cross(forward, [0, 1, 0]));
  const up = cross(right, forward);
  return { camera, forward, right, up, focal: height / (2 * Math.tan(42 * Math.PI / 360)) };
}

function project(point, basis) {
  const relative = subtract(point, basis.camera);
  const depth = dot(relative, basis.forward);
  if (depth < 0.1) return null;
  return {
    x: width / 2 + dot(relative, basis.right) * basis.focal / depth,
    y: height / 2 - dot(relative, basis.up) * basis.focal / depth,
    depth,
  };
}

function line3d(a, b, color, lineWidth, basis) {
  const first = project(a, basis);
  const second = project(b, basis);
  if (!first || !second) return;
  context.strokeStyle = color;
  context.lineWidth = Math.max(0.5, lineWidth * basis.focal / ((first.depth + second.depth) / 2));
  context.beginPath();
  context.moveTo(first.x, first.y);
  context.lineTo(second.x, second.y);
  context.stroke();
}

function drawGround(basis) {
  const sky = context.createLinearGradient(0, 0, 0, height);
  sky.addColorStop(0, "#111a21");
  sky.addColorStop(1, "#18232a");
  context.fillStyle = sky;
  context.fillRect(0, 0, width, height);
  context.lineCap = "round";
  for (let i = -24; i <= 24; i++) {
    const color = i === 0 ? "#426554" : "#293941";
    const lineWidth = i === 0 ? 1.5 : 0.7;
    line3d([i, 0, -24], [i, 0, 24], color, lineWidth, basis);
    line3d([-24, 0, i], [24, 0, i], color, lineWidth, basis);
  }
}

function rotateLocal(point) {
  const [x, y, z] = point;
  const cr = Math.cos(simulation.roll);
  const sr = Math.sin(-simulation.roll);
  const cp = Math.cos(simulation.pitch);
  const sp = Math.sin(simulation.pitch);
  const cy = Math.cos(simulation.yaw);
  const sy = Math.sin(-simulation.yaw);
  const rolled = [cr * x - sr * y, sr * x + cr * y, z];
  const pitched = [rolled[0], cp * rolled[1] - sp * rolled[2], sp * rolled[1] + cp * rolled[2]];
  return [
    cy * pitched[0] + sy * pitched[2],
    pitched[1],
    -sy * pitched[0] + cy * pitched[2],
  ];
}

function worldPoint(point) {
  return add(simulation.position, rotateLocal(point));
}

function drawShadow(basis) {
  const center = [simulation.position[0], 0.008, simulation.position[2]];
  const radius = 0.42 + Math.min(simulation.altitude, 8) * 0.13;
  const points = [];
  for (let i = 0; i <= 32; i++) {
    const angle = i / 32 * Math.PI * 2;
    points.push(project([center[0] + Math.cos(angle) * radius, center[1], center[2] + Math.sin(angle) * radius], basis));
  }
  if (points.some((point) => !point)) return;
  context.fillStyle = `rgba(0, 0, 0, ${Math.max(0.035, 0.2 - simulation.altitude * 0.018)})`;
  context.beginPath();
  context.moveTo(points[0].x, points[0].y);
  for (const point of points.slice(1)) context.lineTo(point.x, point.y);
  context.closePath();
  context.fill();
}

function drawRotorRing(position, radius, basis) {
  const points = [];
  for (let i = 0; i <= 36; i++) {
    const angle = i / 36 * Math.PI * 2;
    points.push(project(worldPoint([position[0] + Math.cos(angle) * radius, 0.065, position[1] + Math.sin(angle) * radius]), basis));
  }
  context.strokeStyle = "rgba(112, 224, 176, 0.28)";
  context.lineWidth = 0.8;
  context.beginPath();
  for (const [i, point] of points.entries()) {
    if (!point) continue;
    if (!points[i - 1]) context.moveTo(point.x, point.y);
    else context.lineTo(point.x, point.y);
  }
  context.stroke();
}

function drawDrone(basis) {
  const motors = rotors;
  context.lineCap = "round";
  for (const [mx, mz] of motors) {
    line3d(worldPoint([0, 0.015, 0]), worldPoint([mx, 0.015, mz]), "#111b20", 0.037, basis);
    line3d(worldPoint([0, 0.023, 0]), worldPoint([mx, 0.023, mz]), "#b5c5c6", 0.019, basis);
  }

  const bodyCorners = [
    [-0.105, 0.065, -0.13], [0.105, 0.065, -0.13],
    [0.105, 0.065, 0.13], [-0.105, 0.065, 0.13],
  ].map(worldPoint);
  const projectedBody = bodyCorners.map((point) => project(point, basis));
  if (projectedBody.every(Boolean)) {
    context.fillStyle = "#29373e";
    context.strokeStyle = "#bac6c5";
    context.lineWidth = 1.1;
    context.beginPath();
    context.moveTo(projectedBody[0].x, projectedBody[0].y);
    for (const point of projectedBody.slice(1)) context.lineTo(point.x, point.y);
    context.closePath();
    context.fill();
    context.stroke();
  }

  line3d(worldPoint([0, 0.07, 0]), worldPoint([0, 0.07, -0.2]), "#70e0b0", 0.035, basis);

  motors.forEach(([mx, mz], i) => {
    const motor = project(worldPoint([mx, 0.045, mz]), basis);
    if (!motor) return;
    const radius = 0.035 * basis.focal / motor.depth;
    context.fillStyle = "#121b20";
    context.beginPath();
    context.arc(motor.x, motor.y, Math.max(radius, 2), 0, Math.PI * 2);
    context.fill();
    drawRotorRing([mx, mz], 0.15, basis);

    const angle = rotorAngles[i];
    for (const offset of [0, Math.PI]) {
      const direction = angle + offset;
      const start = worldPoint([mx - Math.cos(direction) * 0.13, 0.066, mz - Math.sin(direction) * 0.13]);
      const end = worldPoint([mx + Math.cos(direction) * 0.13, 0.066, mz + Math.sin(direction) * 0.13]);
      line3d(start, end, i < 2 ? "#70e0b0" : "#a8b8ba", 0.012, basis);
    }
  });
}

function updateInputDisplay() {
  for (const [name, input] of Object.entries(inputs)) {
    if (name === "stabilized") continue;
    const min = Number(input.min);
    const max = Number(input.max);
    const value = Number(input.value);
    input.style.setProperty("--fill", `${((value - min) / (max - min)) * 100}%`);
  }
  outputs.throttle.value = `${inputs.throttle.value}%`;
  outputs.pitch.value = `${inputs.pitch.value}°`;
  outputs.roll.value = `${inputs.roll.value}°`;
  outputs.yaw.value = `${Math.round(Number(inputs.yaw.value) * 1.8)}°/s`;
  modeLabel.textContent = inputs.stabilized.checked ? "ANGLE · STABILIZED" : "ACRO · RATE";
}

function syncControls() {
  simulation.setControls({
    throttle: Number(inputs.throttle.value),
    pitch: Number(inputs.pitch.value),
    roll: Number(inputs.roll.value),
    yaw: Number(inputs.yaw.value),
    stabilized: inputs.stabilized.checked,
  });
}

function updateTelemetry(now) {
  const flightAltitude = simulation.altitude;
  altitude.innerHTML = `${flightAltitude.toFixed(2)} <small>m</small>`;
  verticalSpeed.innerHTML = `${simulation.velocity[1].toFixed(2)} <small>m/s</small>`;
  attitude.innerHTML = `${Math.round(simulation.roll * 180 / Math.PI)}° / ${Math.round(simulation.pitch * 180 / Math.PI)}° <small>R / P</small>`;
  const airborne = flightAltitude > 0.15;
  sceneState.classList.toggle("airborne", airborne);
  sceneState.innerHTML = `<span></span> ${airborne ? "AIRBORNE" : "GROUNDED"}`;
  if (now - lastTelemetry > 250) {
    document.title = `Quadcopter Flight Lab — ${flightAltitude.toFixed(1)} m`;
    lastTelemetry = now;
  }
}

function render(now = 0) {
  requestAnimationFrame(render);
  const dt = lastFrame ? Math.min((now - lastFrame) / 1000, 0.05) : 0;
  lastFrame = now;
  if (!paused) {
    syncControls();
    const steps = Math.max(1, Math.ceil(dt / 0.004));
    const fixedDt = dt / steps;
    for (let i = 0; i < steps; i++) simulation.step(fixedDt);
    simulation.motorThrusts.forEach((thrust, i) => {
      rotorAngles[i] += (5 + thrust * 8) * dt * (i % 2 === 0 ? 1 : -1);
    });
  }

  target = [simulation.position[0], 0.8 + simulation.altitude * 0.35, simulation.position[2]];
  const basis = cameraBasis();
  drawGround(basis);
  drawShadow(basis);
  drawDrone(basis);
  updateTelemetry(now);
}

function setKeyInput(key, value) {
  const input = inputs[key];
  if (!input) return;
  const next = Number(input.value) + value;
  input.value = String(Math.max(Number(input.min), Math.min(Number(input.max), next)));
  updateInputDisplay();
}

for (const input of Object.values(inputs)) {
  input.addEventListener(input.type === "checkbox" ? "change" : "input", updateInputDisplay);
}
pauseButton.addEventListener("click", () => {
  paused = !paused;
  pauseButton.textContent = paused ? "Resume" : "Pause";
});
document.querySelector("#reset-button").addEventListener("click", () => {
  simulation.reset();
  for (const [key, value] of Object.entries({ throttle: 0, pitch: 0, roll: 0, yaw: 0 })) {
    inputs[key].value = String(value);
  }
  inputs.stabilized.checked = true;
  paused = false;
  pauseButton.textContent = "Pause";
  rotorAngles = [0, 0, 0, 0];
  updateInputDisplay();
  syncControls();
});

canvas.addEventListener("pointerdown", (event) => {
  pointer = { x: event.clientX, y: event.clientY };
  canvas.setPointerCapture(event.pointerId);
});
canvas.addEventListener("pointermove", (event) => {
  if (!pointer) return;
  cameraTheta -= (event.clientX - pointer.x) * 0.008;
  cameraElevation = Math.max(0.12, Math.min(1.38, cameraElevation + (event.clientY - pointer.y) * 0.006));
  pointer = { x: event.clientX, y: event.clientY };
});
canvas.addEventListener("pointerup", () => { pointer = null; });
canvas.addEventListener("pointercancel", () => { pointer = null; });
canvas.addEventListener("wheel", (event) => {
  event.preventDefault();
  cameraDistance = Math.max(4, Math.min(18, cameraDistance * (1 + Math.sign(event.deltaY) * 0.08)));
}, { passive: false });

window.addEventListener("keydown", (event) => {
  if (event.target instanceof HTMLInputElement) return;
  const key = event.key.toLowerCase();
  if (["arrowup", "arrowdown", " "].includes(key)) event.preventDefault();
  if (heldKeys.has(key)) return;
  heldKeys.add(key);
  if (key === "w") setKeyInput("pitch", 1);
  if (key === "s") setKeyInput("pitch", -1);
  if (key === "a") setKeyInput("roll", -1);
  if (key === "d") setKeyInput("roll", 1);
  if (key === "q") setKeyInput("yaw", -5);
  if (key === "e") setKeyInput("yaw", 5);
  if (key === "arrowup") setKeyInput("throttle", 1);
  if (key === "arrowdown") setKeyInput("throttle", -1);
  if (key === " ") pauseButton.click();
});
window.addEventListener("keyup", (event) => heldKeys.delete(event.key.toLowerCase()));
window.addEventListener("blur", () => heldKeys.clear());
new ResizeObserver(resize).observe(mount);
updateInputDisplay();
resize();
render();
