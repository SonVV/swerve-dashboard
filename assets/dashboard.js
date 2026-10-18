/*
 * Dashboard browser application.
 *
 * The file is intentionally dependency-free.  It is split into five sections:
 * state/WebSocket, manual control, charts, occupancy map, and notifications.
 */

const byId = (id) => document.getElementById(id);
const MODULE_IDS = ["FL", "FR", "RL", "RR"];
const MODULE_COLORS = ["#5eead4", "#60a5fa", "#fbbf24", "#f472b6"];

// ---------- Shared browser state and WebSocket telemetry ----------

let socket = null;
let manualEnabled = false;
let joystick = { x: 0, y: 0 };
let pose = { x: 0, y: 0, heading: 0, frame: "waiting" };
let lastPoseFrame = "";
let mission = "idle";
let reconnectTimer = null;
let lastNavigationSignature = "0:idle";

byId("wheels").innerHTML = MODULE_IDS.map((id) => `
  <div class="wheel">
    <b>${id}</b>
    <div id="wheel-${id}">0 m/s<br>0 m/s²<br>0°</div>
  </div>
`).join("");

/** Connect to the bridge and reconnect after a temporary network loss. */
function connect() {
  const scheme = location.protocol === "https:" ? "wss" : "ws";
  socket = new WebSocket(`${scheme}://${location.host}/ws`);

  socket.onopen = () => {
    byId("connection").textContent = "● Connected";
    byId("connection").style.color = "#5eead4";
  };

  socket.onclose = () => {
    byId("connection").textContent = "● Disconnected";
    byId("connection").style.color = "#fb7185";
    manualEnabled = false;
    resetManualInputs();
    clearTimeout(reconnectTimer);
    reconnectTimer = setTimeout(connect, 1500);
  };

  socket.onmessage = (event) => {
    const packet = JSON.parse(event.data);
    if (packet.type === "control") {
      handleControlResponse(packet);
    } else if (packet.type === "navigation") {
      handleNavigationResponse(packet);
    } else if (packet.type === "telemetry") {
      handleTelemetry(packet);
    }
  };
}

/** Apply one 20 Hz telemetry frame to numeric cards, charts and map. */
function handleTelemetry(data) {
  if (lastPoseFrame && lastPoseFrame !== data.pose.frame) {
    robotPath = [];
  }
  lastPoseFrame = data.pose.frame;
  pose = data.pose;
  mission = data.mission;

  // The path is bounded so leaving the dashboard open cannot consume all RAM.
  robotPath.push({ ...pose });
  if (robotPath.length > 12000) robotPath.shift();

  ["x", "y", "heading"].forEach((key) => {
    byId(key).textContent = Number(pose[key]).toFixed(2);
  });
  ["vx", "vy", "omega"].forEach((key) => {
    byId(key).textContent = Number(data.twist[key]).toFixed(2);
  });
  data.wheels.forEach((wheel) => {
    byId(`wheel-${wheel.id}`).innerHTML = [
      `${wheel.speed.toFixed(2)} m/s`,
      `${wheel.accel.toFixed(2)} m/s²`,
      `${wheel.steering.toFixed(1)}°`,
    ].join("<br>");
  });

  if (data.map && data.map.revision !== mapRevision) setMap(data.map);
  const readyTopics = Object.entries(data.status)
    .filter(([, ready]) => ready)
    .map(([name]) => name)
    .join(", ");
  byId("mode").textContent = `ROS 2 · ${pose.frame.toUpperCase()} · ${mission.toUpperCase()}`;
  byId("mapStatus").textContent = mapModel
    ? `/map ${mapModel.width}×${mapModel.height} · ${mapModel.resolution.toFixed(3)} m/cell · topics: ${readyTopics || "waiting"}`
    : `Đang chờ /map · topics: ${readyTopics || "waiting"}`;

  // Autonomous actions and manual commands must not publish at the same time.
  byId("enable").disabled = mission !== "idle" && !manualEnabled;
  if (mission !== "idle" && !manualEnabled) {
    byId("controlStatus").textContent = `Action ${mission} đang chạy`;
  }

  appendHistory(data);
  updateNavigationState(data.navigation);
  if (mapView.followRobot) {
    mapView.centerX = pose.x;
    mapView.centerY = pose.y;
  }
  drawMap();
  if (!historyPaused) drawCharts();
}

function handleControlResponse(response) {
  if (response.accepted) return;
  manualEnabled = false;
  resetManualInputs();
  byId("enable").textContent = "ENABLE";
  byId("controlStatus").textContent = response.message || "Manual command rejected";
}

connect();

// ---------- Manual swerve control ----------

const joystickCanvas = byId("joy");
const joystickContext = joystickCanvas.getContext("2d");
const pressedKeys = new Set();
let joystickDragging = false;

/** Render the joystick crosshair and its current normalized vector. */
function drawJoystick() {
  const context = joystickContext;
  context.clearRect(0, 0, 260, 260);
  context.strokeStyle = "#527398";
  context.lineWidth = 2;
  context.beginPath();
  context.arc(130, 130, 105, 0, Math.PI * 2);
  context.moveTo(130, 15);
  context.lineTo(130, 245);
  context.moveTo(15, 130);
  context.lineTo(245, 130);
  context.stroke();
  context.fillStyle = "#5eead4";
  context.beginPath();
  context.arc(130 + joystick.x * 100, 130 - joystick.y * 100, 22, 0, Math.PI * 2);
  context.fill();
}

/** Convert one pointer position to a vector limited to the unit circle. */
function updateJoystickFromPointer(event) {
  const bounds = joystickCanvas.getBoundingClientRect();
  const x = ((event.clientX - bounds.left) / bounds.width * 260 - 130) / 100;
  const y = (130 - (event.clientY - bounds.top) / bounds.height * 260) / 100;
  const length = Math.max(1, Math.hypot(x, y));
  joystick = { x: x / length, y: y / length };
  drawJoystick();
}

/** Send one command packet; a timer refreshes it before the server watchdog. */
function sendManualCommand() {
  if (socket?.readyState !== WebSocket.OPEN) return;
  const speed = Number(byId("speedLimit").value);
  const keyboardOmega = (pressedKeys.has("e") ? 0.3 : 0) - (pressedKeys.has("q") ? 0.3 : 0);
  socket.send(JSON.stringify({
    type: "command",
    vx: joystick.y * speed,
    vy: joystick.x * speed,
    omega: Number(byId("yawRate").value) + keyboardOmega,
    mode: Number(byId("driveMode").value),
    enabled: manualEnabled,
  }));
}

function resetManualInputs() {
  joystick = { x: 0, y: 0 };
  byId("yawRate").value = "0";
  byId("yawVal").textContent = "0.00";
  drawJoystick();
}

joystickCanvas.addEventListener("pointerdown", (event) => {
  joystickDragging = true;
  joystickCanvas.setPointerCapture(event.pointerId);
  updateJoystickFromPointer(event);
});
joystickCanvas.addEventListener("pointermove", (event) => {
  if (joystickDragging) updateJoystickFromPointer(event);
});
const releaseJoystick = () => {
  joystickDragging = false;
  joystick = { x: 0, y: 0 };
  drawJoystick();
};
joystickCanvas.addEventListener("pointerup", releaseJoystick);
joystickCanvas.addEventListener("pointercancel", releaseJoystick);

window.addEventListener("keydown", (event) => {
  const key = event.key.toLowerCase();
  if (!["w", "a", "s", "d", "q", "e"].includes(key)) return;
  pressedKeys.add(key);
  if (!joystickDragging) {
    joystick = {
      x: Number(pressedKeys.has("d")) - Number(pressedKeys.has("a")),
      y: Number(pressedKeys.has("w")) - Number(pressedKeys.has("s")),
    };
    drawJoystick();
  }
});
window.addEventListener("keyup", (event) => {
  pressedKeys.delete(event.key.toLowerCase());
  if (!joystickDragging) {
    joystick = {
      x: Number(pressedKeys.has("d")) - Number(pressedKeys.has("a")),
      y: Number(pressedKeys.has("w")) - Number(pressedKeys.has("s")),
    };
    drawJoystick();
  }
});
window.addEventListener("blur", () => {
  pressedKeys.clear();
  manualEnabled = false;
  resetManualInputs();
  sendManualCommand();
});

byId("enable").addEventListener("click", () => {
  manualEnabled = !manualEnabled;
  byId("enable").textContent = manualEnabled ? "DISABLE" : "ENABLE";
  byId("controlStatus").textContent = manualEnabled
    ? "Manual control enabled"
    : "Manual control disabled";
  if (!manualEnabled) resetManualInputs();
  sendManualCommand();
});
byId("stop").addEventListener("click", () => {
  manualEnabled = false;
  resetManualInputs();
  byId("enable").textContent = "ENABLE";
  byId("controlStatus").textContent = "Stop sent; publisher releases after 0.6 s";
  sendManualCommand();
});
byId("yawRate").addEventListener("input", () => {
  byId("yawVal").textContent = Number(byId("yawRate").value).toFixed(2);
});
byId("speedLimit").addEventListener("input", () => {
  byId("speedVal").textContent = Number(byId("speedLimit").value).toFixed(2);
});
setInterval(sendManualCommand, 100);
drawJoystick();

// ---------- Wheel history and canvas charts ----------

let history = [];
let historyPaused = false;
let historyLimitSeconds = 1800;
const chartState = {
  speed: { windowSeconds: 30, visible: [true, true, true, true] },
  accel: { windowSeconds: 30, visible: [true, true, true, true] },
  steering: { windowSeconds: 30, visible: [true, true, true, true] },
};

function appendHistory(telemetry) {
  history.push({
    time: Date.now(),
    speed: telemetry.wheels.map((wheel) => wheel.speed),
    accel: telemetry.wheels.map((wheel) => wheel.accel),
    steering: telemetry.wheels.map((wheel) => wheel.steering),
  });
  trimHistory();
}

function trimHistory() {
  const cutoff = Date.now() - historyLimitSeconds * 1000;
  while (history.length && history[0].time < cutoff) history.shift();
}

function timeLabel(timestamp) {
  return new Date(timestamp).toLocaleTimeString("vi-VN", { hour12: false });
}

/** Build each chart once.  Canvas IDs use signal-* and never collide with controls. */
for (const key of ["speed", "accel", "steering"]) {
  const parent = byId(`chart-${key}`);
  parent.innerHTML = `
    <div class="chart-tools">
      <label>Khung thời gian</label>
      <select class="window">
        <option value="10">10 s</option><option value="30" selected>30 s</option>
        <option value="60">1 phút</option><option value="300">5 phút</option>
        <option value="900">15 phút</option><option value="1800">30 phút</option>
        <option value="0">Tất cả</option>
      </select>
      <button class="auto active">Auto Y</button>
      <div class="chart-legend">
        ${MODULE_IDS.map((id, index) => `
          <button class="legend-item" data-index="${index}">
            <span class="legend-dot" style="background:${MODULE_COLORS[index]}"></span>${id}
          </button>`).join("")}
      </div>
    </div>
    <div class="chart-stage">
      <canvas class="chart" id="signal-${key}"></canvas>
      <div class="chart-tooltip"></div>
    </div>
    <div class="chart-foot">
      <span class="range">Đang chờ dữ liệu...</span>
      <span>Hover để xem giá trị</span>
    </div>`;

  const state = chartState[key];
  state.autoScale = true;
  state.hoverRatio = null;
  parent.querySelector(".window").addEventListener("change", (event) => {
    state.windowSeconds = Number(event.target.value);
    drawCharts();
  });
  parent.querySelector(".auto").addEventListener("click", (event) => {
    state.autoScale = !state.autoScale;
    event.currentTarget.classList.toggle("active", state.autoScale);
    drawCharts();
  });
  parent.querySelectorAll(".legend-item").forEach((button) => {
    button.addEventListener("click", () => {
      const index = Number(button.dataset.index);
      state.visible[index] = !state.visible[index];
      button.classList.toggle("off", !state.visible[index]);
      drawCharts();
    });
  });
  const canvas = byId(`signal-${key}`);
  canvas.addEventListener("mousemove", (event) => {
    const bounds = canvas.getBoundingClientRect();
    state.hoverRatio = (event.clientX - bounds.left) / bounds.width;
    drawCharts();
  });
  canvas.addEventListener("mouseleave", () => {
    state.hoverRatio = null;
    parent.querySelector(".chart-tooltip").style.display = "none";
    drawCharts();
  });
}

function drawCharts() {
  ["speed", "accel", "steering"].forEach(drawChart);
}

/** Draw axes, four module traces and an optional hover cursor for one signal. */
function drawChart(key) {
  const canvas = byId(`signal-${key}`);
  const context = canvas.getContext("2d");
  const state = chartState[key];
  const parent = byId(`chart-${key}`);
  const bounds = canvas.getBoundingClientRect();
  const width = Math.max(300, bounds.width);
  const height = Math.max(220, bounds.height);
  const pixelRatio = Math.min(window.devicePixelRatio || 1, 2);
  const nextWidth = Math.round(width * pixelRatio);
  const nextHeight = Math.round(height * pixelRatio);
  if (canvas.width !== nextWidth || canvas.height !== nextHeight) {
    canvas.width = nextWidth;
    canvas.height = nextHeight;
  }
  context.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
  context.clearRect(0, 0, width, height);

  const margin = { left: 62, right: 18, top: 23, bottom: 37 };
  const plotWidth = width - margin.left - margin.right;
  const plotHeight = height - margin.top - margin.bottom;
  const end = history.length ? history.at(-1).time : Date.now();
  const start = state.windowSeconds
    ? end - state.windowSeconds * 1000
    : (history[0]?.time || end - 30000);
  const samples = history.filter((sample) => sample.time >= start && sample.time <= end);

  const values = [];
  samples.forEach((sample) => sample[key].forEach((value, index) => {
    if (state.visible[index]) values.push(value);
  }));
  let minimum = values.length ? Math.min(...values) : 0;
  let maximum = values.length ? Math.max(...values) : 1;
  if (state.autoScale) {
    minimum = Math.min(minimum, 0);
    maximum = Math.max(maximum, 0);
    const padding = Math.max((maximum - minimum) * 0.12, key === "steering" ? 2 : 0.04);
    minimum -= padding;
    maximum += padding;
  } else {
    const bound = key === "steering" ? 180 : 3;
    minimum = -bound;
    maximum = bound;
  }
  if (maximum <= minimum) maximum = minimum + 1;

  const toX = (time) => margin.left + (time - start) / Math.max(1, end - start) * plotWidth;
  const toY = (value) => margin.top + (maximum - value) / (maximum - minimum) * plotHeight;

  // Y grid and numeric labels.
  context.font = "11px system-ui";
  context.lineWidth = 1;
  context.textAlign = "right";
  for (let index = 0; index <= 5; index += 1) {
    const value = minimum + (maximum - minimum) * index / 5;
    const y = toY(value);
    context.strokeStyle = "#263a53";
    context.beginPath();
    context.moveTo(margin.left, y);
    context.lineTo(width - margin.right, y);
    context.stroke();
    context.fillStyle = "#9ab0cc";
    context.fillText(value.toFixed(Math.abs(maximum - minimum) < 2 ? 2 : 1), margin.left - 10, y + 4);
  }

  // Time grid and labels.
  context.textAlign = "center";
  for (let index = 0; index <= 5; index += 1) {
    const time = start + (end - start) * index / 5;
    const x = toX(time);
    context.strokeStyle = "#1d3047";
    context.beginPath();
    context.moveTo(x, margin.top);
    context.lineTo(x, height - margin.bottom);
    context.stroke();
    context.fillStyle = "#90a8c7";
    context.fillText(timeLabel(time), x, height - 14);
  }

  // Clip traces so they never draw over labels.
  context.save();
  context.beginPath();
  context.rect(margin.left, margin.top, plotWidth, plotHeight);
  context.clip();
  for (let moduleIndex = 0; moduleIndex < 4; moduleIndex += 1) {
    if (!state.visible[moduleIndex]) continue;
    context.beginPath();
    context.strokeStyle = MODULE_COLORS[moduleIndex];
    context.lineWidth = 2;
    samples.forEach((sample, sampleIndex) => {
      const x = toX(sample.time);
      const y = toY(sample[key][moduleIndex]);
      if (sampleIndex === 0) context.moveTo(x, y);
      else context.lineTo(x, y);
    });
    context.stroke();
  }

  let nearest = null;
  if (state.hoverRatio !== null && samples.length) {
    const hoverX = state.hoverRatio * width;
    const targetTime = start + (hoverX - margin.left) / plotWidth * (end - start);
    nearest = samples.reduce((best, sample) => (
      !best || Math.abs(sample.time - targetTime) < Math.abs(best.time - targetTime) ? sample : best
    ), null);
    const x = toX(nearest.time);
    context.strokeStyle = "#b9cce7";
    context.setLineDash([4, 4]);
    context.beginPath();
    context.moveTo(x, margin.top);
    context.lineTo(x, height - margin.bottom);
    context.stroke();
    context.setLineDash([]);
    nearest[key].forEach((value, moduleIndex) => {
      if (!state.visible[moduleIndex]) return;
      context.fillStyle = MODULE_COLORS[moduleIndex];
      context.beginPath();
      context.arc(x, toY(value), 4, 0, Math.PI * 2);
      context.fill();
    });
  }
  context.restore();

  const tooltip = parent.querySelector(".chart-tooltip");
  if (nearest) {
    tooltip.innerHTML = `<b>${timeLabel(nearest.time)}</b>${MODULE_IDS.map((id, index) => (
      state.visible[index]
        ? `<div><span style="color:${MODULE_COLORS[index]}">●</span> ${id}: <b>${nearest[key][index].toFixed(3)}</b></div>`
        : ""
    )).join("")}`;
    tooltip.style.display = "block";
    tooltip.style.left = `${Math.min(width - 155, Math.max(70, toX(nearest.time) + 12))}px`;
    tooltip.style.top = "35px";
  } else {
    tooltip.style.display = "none";
  }
  parent.querySelector(".range").textContent = [
    `${samples.length.toLocaleString("vi-VN")} mẫu hiển thị`,
    `${history.length.toLocaleString("vi-VN")} mẫu lưu`,
    samples.length ? `${timeLabel(samples[0].time)} — ${timeLabel(samples.at(-1).time)}` : "Chưa có dữ liệu",
  ].join(" · ");
}

byId("historyLimit").addEventListener("change", (event) => {
  historyLimitSeconds = Number(event.target.value);
  trimHistory();
  drawCharts();
});
byId("pauseCharts").addEventListener("click", () => {
  historyPaused = !historyPaused;
  byId("pauseCharts").textContent = historyPaused ? "▶ Resume" : "⏸ Pause";
  byId("pauseCharts").classList.toggle("active", historyPaused);
  if (!historyPaused) drawCharts();
});
byId("clearHistory").addEventListener("click", () => {
  history = [];
  drawCharts();
});
byId("exportCsv").addEventListener("click", () => {
  const header = [
    "timestamp_iso",
    ...MODULE_IDS.map((id) => `${id}_speed_mps`),
    ...MODULE_IDS.map((id) => `${id}_accel_mps2`),
    ...MODULE_IDS.map((id) => `${id}_steering_deg`),
  ];
  const rows = [header, ...history.map((sample) => [
    new Date(sample.time).toISOString(), ...sample.speed, ...sample.accel, ...sample.steering,
  ])];
  const blob = new Blob([rows.map((row) => row.join(",")).join("\n")], { type: "text/csv;charset=utf-8" });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement("a");
  anchor.href = url;
  anchor.download = `swerve-telemetry-${Date.now()}.csv`;
  anchor.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
});

// ---------- Occupancy map, pan/zoom and goal selection ----------

const mapCanvas = byId("map");
const mapContext = mapCanvas.getContext("2d");
let mapRevision = -1;
let mapModel = null;
let mapCells = null;
let mapImage = null;
let robotPath = [];
let selectedGoal = null;
let selectingGoal = false;
let mapPointer = null;
const mapView = { centerX: 0, centerY: 0, zoom: 28, followRobot: true };

/** Expand the server RLE grid and build a cached transparent occupancy image. */
function setMap(model) {
  mapRevision = model.revision;
  mapModel = model;
  mapCells = new Int8Array(model.width * model.height);
  let cursor = 0;
  for (let index = 0; index < model.rle.length; index += 2) {
    const value = model.rle[index];
    const count = model.rle[index + 1];
    mapCells.fill(value, cursor, cursor + count);
    cursor += count;
  }

  const offscreen = document.createElement("canvas");
  offscreen.width = model.width;
  offscreen.height = model.height;
  const context = offscreen.getContext("2d");
  const image = context.createImageData(model.width, model.height);
  for (let gridY = 0; gridY < model.height; gridY += 1) {
    for (let gridX = 0; gridX < model.width; gridX += 1) {
      const value = mapCells[gridY * model.width + gridX];
      // Canvas Y points down, while OccupancyGrid Y points up.
      const pixel = ((model.height - 1 - gridY) * model.width + gridX) * 4;
      if (value < 0) {
        image.data[pixel + 3] = 0;
      } else if (value >= 50) {
        image.data[pixel] = 248;
        image.data[pixel + 1] = 113;
        image.data[pixel + 2] = 113;
        image.data[pixel + 3] = 230;
      } else {
        image.data[pixel] = 210;
        image.data[pixel + 1] = 225;
        image.data[pixel + 2] = 242;
        image.data[pixel + 3] = 75;
      }
    }
  }
  context.putImageData(image, 0, 0);
  mapImage = offscreen;
  if (mapView.followRobot) {
    mapView.centerX = pose.x;
    mapView.centerY = pose.y;
  }
  drawMap();
}

function worldToScreen(x, y) {
  return {
    x: mapCanvas.width / 2 + (x - mapView.centerX) * mapView.zoom,
    y: mapCanvas.height / 2 - (y - mapView.centerY) * mapView.zoom,
  };
}

function screenToWorld(x, y) {
  return {
    x: mapView.centerX + (x - mapCanvas.width / 2) / mapView.zoom,
    y: mapView.centerY - (y - mapCanvas.height / 2) / mapView.zoom,
  };
}

/** Convert a pointer event from CSS pixels to the canvas backing coordinates. */
function eventCanvasPoint(event) {
  const bounds = mapCanvas.getBoundingClientRect();
  return {
    x: (event.clientX - bounds.left) * mapCanvas.width / bounds.width,
    y: (event.clientY - bounds.top) * mapCanvas.height / bounds.height,
  };
}

/** Read the OccupancyGrid cell at a map-frame world coordinate. */
function cellAtWorld(x, y) {
  if (!mapModel || !mapCells) return { valid: false, message: "Robot chưa scan nên chưa thể chọn tọa độ" };
  const origin = mapModel.origin;
  const dx = x - origin.x;
  const dy = y - origin.y;
  const cosine = Math.cos(origin.yaw);
  const sine = Math.sin(origin.yaw);
  const gridX = Math.floor((cosine * dx + sine * dy) / mapModel.resolution);
  const gridY = Math.floor((-sine * dx + cosine * dy) / mapModel.resolution);
  if (gridX < 0 || gridY < 0 || gridX >= mapModel.width || gridY >= mapModel.height) {
    return { valid: false, message: "Tọa độ nằm ngoài bản đồ đã scan" };
  }
  const value = mapCells[gridY * mapModel.width + gridX];
  if (value < 0) return { valid: false, message: "Tọa độ này chưa được scan" };
  if (value >= 50) return { valid: false, message: "Tọa độ nằm trên vật cản" };
  return { valid: true, value };
}

/** Paint map grid, cached occupancy image, robot path, goal and robot marker. */
function drawMap() {
  const context = mapContext;
  const width = mapCanvas.width;
  const height = mapCanvas.height;
  context.setTransform(1, 0, 0, 1, 0, 0);
  context.clearRect(0, 0, width, height);
  context.fillStyle = "#081321";
  context.fillRect(0, 0, width, height);

  const gridStep = mapView.zoom;
  const worldOrigin = worldToScreen(0, 0);
  context.strokeStyle = "#203650";
  context.lineWidth = 1;
  for (let x = ((worldOrigin.x % gridStep) + gridStep) % gridStep; x < width; x += gridStep) {
    context.beginPath(); context.moveTo(x, 0); context.lineTo(x, height); context.stroke();
  }
  for (let y = ((worldOrigin.y % gridStep) + gridStep) % gridStep; y < height; y += gridStep) {
    context.beginPath(); context.moveTo(0, y); context.lineTo(width, y); context.stroke();
  }

  if (mapModel && mapImage) {
    const resolution = mapModel.resolution;
    const origin = mapModel.origin;
    const cosine = Math.cos(origin.yaw);
    const sine = Math.sin(origin.yaw);
    const scale = mapView.zoom * resolution;
    const mapHeightMeters = mapModel.height * resolution;
    context.save();
    context.imageSmoothingEnabled = false;
    context.setTransform(
      scale * cosine,
      -scale * sine,
      scale * sine,
      scale * cosine,
      width / 2 + mapView.zoom * (origin.x - sine * mapHeightMeters - mapView.centerX),
      height / 2 - mapView.zoom * (origin.y + cosine * mapHeightMeters - mapView.centerY),
    );
    context.drawImage(mapImage, 0, 0);
    context.restore();
  }

  context.strokeStyle = "#5eead4";
  context.lineWidth = 2;
  context.beginPath();
  robotPath.forEach((sample, index) => {
    const point = worldToScreen(sample.x, sample.y);
    if (index === 0) context.moveTo(point.x, point.y);
    else context.lineTo(point.x, point.y);
  });
  context.stroke();

  if (selectedGoal) {
    const point = worldToScreen(selectedGoal.x, selectedGoal.y);
    context.strokeStyle = "#fbbf24";
    context.fillStyle = "#fbbf2433";
    context.lineWidth = 3;
    context.beginPath(); context.arc(point.x, point.y, 10, 0, Math.PI * 2); context.fill(); context.stroke();
    context.beginPath(); context.moveTo(point.x - 15, point.y); context.lineTo(point.x + 15, point.y); context.stroke();
    context.beginPath(); context.moveTo(point.x, point.y - 15); context.lineTo(point.x, point.y + 15); context.stroke();
  }

  const robot = worldToScreen(pose.x, pose.y);
  context.save();
  context.translate(robot.x, robot.y);
  context.rotate(-pose.heading * Math.PI / 180);
  context.fillStyle = "#60a5fa";
  context.beginPath();
  context.moveTo(18, 0); context.lineTo(-14, -10); context.lineTo(-14, 10); context.closePath();
  context.fill();
  context.restore();

  context.fillStyle = "#a9b9ce";
  context.font = "12px system-ui";
  context.fillText(`${pose.frame} · ${mapView.zoom.toFixed(0)} px/m`, 12, 20);
}

mapCanvas.addEventListener("wheel", (event) => {
  event.preventDefault();
  const point = eventCanvasPoint(event);
  const worldBeforeZoom = screenToWorld(point.x, point.y);
  mapView.zoom = Math.max(5, Math.min(180, mapView.zoom * (event.deltaY < 0 ? 1.15 : 0.87)));
  const worldAfterZoom = screenToWorld(point.x, point.y);
  // Keep the world point under the cursor fixed while changing scale.
  mapView.centerX += worldBeforeZoom.x - worldAfterZoom.x;
  mapView.centerY += worldBeforeZoom.y - worldAfterZoom.y;
  mapView.followRobot = false;
  drawMap();
}, { passive: false });

mapCanvas.addEventListener("pointerdown", (event) => {
  const point = eventCanvasPoint(event);
  mapPointer = {
    id: event.pointerId,
    startX: point.x,
    startY: point.y,
    centerX: mapView.centerX,
    centerY: mapView.centerY,
    moved: false,
  };
  mapCanvas.setPointerCapture(event.pointerId);
  mapCanvas.classList.add("dragging");
});

mapCanvas.addEventListener("pointermove", (event) => {
  if (!mapPointer || mapPointer.id !== event.pointerId) return;
  const point = eventCanvasPoint(event);
  const dx = point.x - mapPointer.startX;
  const dy = point.y - mapPointer.startY;
  if (Math.hypot(dx, dy) > 4) mapPointer.moved = true;
  if (!mapPointer.moved) return;
  mapView.centerX = mapPointer.centerX - dx / mapView.zoom;
  mapView.centerY = mapPointer.centerY + dy / mapView.zoom;
  mapView.followRobot = false;
  drawMap();
});

function finishMapPointer(event) {
  if (!mapPointer || mapPointer.id !== event.pointerId) return;
  const point = eventCanvasPoint(event);
  const wasClick = !mapPointer.moved;
  mapPointer = null;
  mapCanvas.classList.remove("dragging");
  if (!wasClick || !selectingGoal) return;

  const world = screenToWorld(point.x, point.y);
  const cell = cellAtWorld(world.x, world.y);
  if (!cell.valid) {
    showModal("Không thể chọn tọa độ", cell.message, "error");
    return;
  }
  selectedGoal = world;
  selectingGoal = false;
  mapCanvas.classList.remove("selecting");
  byId("selectGoal").classList.remove("active");
  byId("goToGoal").disabled = false;
  byId("goalStatus").textContent = `Đích: x=${world.x.toFixed(2)} m, y=${world.y.toFixed(2)} m`;
  drawMap();
}
mapCanvas.addEventListener("pointerup", finishMapPointer);
mapCanvas.addEventListener("pointercancel", finishMapPointer);

byId("centerMap").addEventListener("click", () => {
  mapView.centerX = pose.x;
  mapView.centerY = pose.y;
  mapView.followRobot = true;
  drawMap();
});
byId("selectGoal").addEventListener("click", () => {
  if (!mapModel || !mapCells) {
    showModal("Chưa có bản đồ", "Robot chưa scan nên không thể chọn tọa độ.", "error");
    return;
  }
  selectingGoal = !selectingGoal;
  byId("selectGoal").classList.toggle("active", selectingGoal);
  mapCanvas.classList.toggle("selecting", selectingGoal);
  byId("goalStatus").textContent = selectingGoal
    ? "Nhấp vào một ô trống trên bản đồ"
    : (selectedGoal ? `Đích: x=${selectedGoal.x.toFixed(2)} m, y=${selectedGoal.y.toFixed(2)} m` : "Kéo để pan · cuộn để zoom");
});

byId("goToGoal").addEventListener("click", () => {
  if (!mapModel) {
    showModal("Chưa có bản đồ", "Robot chưa scan nên không thể chọn tọa độ.", "error");
    return;
  }
  if (!selectedGoal) {
    showModal("Chưa chọn đích", "Bấm “Chọn đích”, sau đó nhấp vào ô trống trên bản đồ.", "error");
    return;
  }
  if (socket?.readyState !== WebSocket.OPEN) {
    showModal("Mất kết nối", "Dashboard chưa kết nối tới ROS 2 bridge.", "error");
    return;
  }
  const yawDegrees = Number(byId("goalYaw").value);
  socket.send(JSON.stringify({
    type: "navigate",
    x: selectedGoal.x,
    y: selectedGoal.y,
    yaw: yawDegrees * Math.PI / 180,
  }));
  byId("goToGoal").disabled = true;
  byId("goalStatus").textContent = "Đang gửi mục tiêu tới ROS 2…";
});

/** Handle immediate server validation before a ROS action goal exists. */
function handleNavigationResponse(response) {
  if (!response.accepted) {
    byId("goToGoal").disabled = !selectedGoal;
    byId("goalStatus").textContent = response.message || "Không gửi được mục tiêu";
    showModal("Không thể điều hướng", response.message || "Yêu cầu bị từ chối.", "error");
    return;
  }
  byId("goalStatus").textContent = "ROS 2 đã nhận yêu cầu, đang chờ action server…";
}

/** Turn action feedback/final result in telemetry into status text and modal. */
function updateNavigationState(navigation) {
  if (!navigation || navigation.id <= 0) return;
  const distance = Number.isFinite(navigation.distance_remaining)
    ? ` · còn ${navigation.distance_remaining.toFixed(2)} m`
    : "";
  if (["submitting", "active"].includes(navigation.state)) {
    byId("goalStatus").textContent = `${navigation.message || "Đang điều hướng"}${distance}`;
    byId("goToGoal").disabled = true;
  }

  const signature = `${navigation.id}:${navigation.state}`;
  if (signature === lastNavigationSignature) return;
  lastNavigationSignature = signature;
  if (navigation.state === "succeeded") {
    byId("goalStatus").textContent = navigation.message || "Robot đã đến mục tiêu";
    byId("goToGoal").disabled = false;
    showModal("Đã đến mục tiêu", navigation.message || "Robot đã đến tọa độ đã chọn.", "success");
  } else if (navigation.state === "failed") {
    byId("goalStatus").textContent = navigation.message || "Điều hướng thất bại";
    byId("goToGoal").disabled = false;
    showModal("Điều hướng thất bại", navigation.message || "Robot không thể đến mục tiêu.", "error");
  }
}

// ---------- Accessible notification modal and responsive redraw ----------

function showModal(title, message, kind = "info") {
  const modal = byId("noticeModal");
  modal.className = `modal ${kind}`;
  byId("noticeTitle").textContent = title;
  byId("noticeMessage").textContent = message;
  byId("noticeIcon").textContent = kind === "success" ? "✓" : (kind === "error" ? "!" : "i");
  modal.hidden = false;
  byId("closeModal").focus();
}

function closeModal() {
  byId("noticeModal").hidden = true;
}

byId("closeModal").addEventListener("click", closeModal);
document.querySelector("[data-close-modal]").addEventListener("click", closeModal);
window.addEventListener("keydown", (event) => {
  if (event.key === "Escape") closeModal();
});
window.addEventListener("resize", () => {
  drawCharts();
  drawMap();
});

drawCharts();
drawMap();
