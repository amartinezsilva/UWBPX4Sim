// UWBPX4Sim setup GUI. Vanilla JS, no build step, no framework -- kept
// intentionally simple. State lives in the DOM (data-* attributes); forms
// are read back into plain objects only when saving/continuing.

const DEFAULT_UAV_OFFBOARD = {
  trajectory_csv_file: "trajectory_lemniscate_uav.csv",
  lookahead_distance: 2.0,
  cruise_speed: 0.5,
  odom_error_position: 0.0,
  odom_error_angle: 0.0,
};
const DEFAULT_UGV_OFFBOARD = {
  trajectory_csv_file: "trajectory_lemniscate_agv.csv",
  lookahead_distance: 2.0,
  cruise_speed: 0.52,
  odom_error_position: 0.0,
  odom_error_angle: 0.0,
};

// type -> {min, max, step} for sliders; 'text'/'bool'/'number' for the rest.
const PARAM_UI = {
  topic: { type: "text" },
  ground_truth_topic: { type: "text" },
  gaussian_noise_mean_cm: { type: "number", step: 0.1 },
  apply_pair_bias: { type: "bool" },
  bias_min_cm: { type: "number", step: 0.1 },
  bias_max_cm: { type: "number", step: 0.1 },
  enable_nlos_dropout: { type: "bool" },
  nlos_endpoint_margin_m: { type: "slider", min: 0, max: 1, step: 0.01 },
  los_dropout_start_distance_m: { type: "slider", min: 0, max: 100, step: 0.5 },
  los_dropout_end_distance_m: { type: "slider", min: 0, max: 150, step: 0.5 },
  los_hard_dropout_distance_m: { type: "slider", min: 0, max: 150, step: 0.5 },
  los_max_thickness_m: { type: "slider", min: 0, max: 2, step: 0.01 },
  soft_nlos_max_thickness_m: { type: "slider", min: 0, max: 2, step: 0.01 },
  blackout_thickness_m: { type: "slider", min: 0, max: 3, step: 0.01 },
  hard_nlos_gap_ratio: { type: "slider", min: 0, max: 2, step: 0.01 },
  min_dropout_probability: { type: "slider", min: 0, max: 1, step: 0.01 },
  max_dropout_probability: { type: "slider", min: 0, max: 1, step: 0.01 },
  min_noise_stddev_cm: { type: "slider", min: 0, max: 200, step: 1 },
  max_noise_stddev_cm: { type: "slider", min: 0, max: 200, step: 1 },
  los_stddev_start_distance_m: { type: "slider", min: 0, max: 150, step: 0.5 },
  los_stddev_end_distance_m: { type: "slider", min: 0, max: 150, step: 0.5 },
  nlos_dropout_thickness_extra_weight: { type: "slider", min: 0, max: 2, step: 0.01 },
  nlos_stddev_thickness_extra_weight: { type: "slider", min: 0, max: 2, step: 0.01 },
};

let trajectoryOptions = [];
let layout = { uavs: [], ugvs: [], world: "" };

// ---------------------------------------------------------------------------
// small DOM helper
// ---------------------------------------------------------------------------

function el(tag, attrs = {}, children = []) {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === "class") e.className = v;
    else if (k === "text") e.textContent = v;
    else e.setAttribute(k, v);
  }
  for (const c of children) e.appendChild(typeof c === "string" ? document.createTextNode(c) : c);
  return e;
}

function setStatus(id, msg, ok) {
  const node = document.getElementById(id);
  node.textContent = msg;
  node.className = "status " + (ok === true ? "ok" : ok === false ? "error" : "");
}

async function api(path, opts) {
  const res = await fetch(path, opts);
  const data = await res.json().catch(() => ({}));
  if (!res.ok) throw new Error(data.error || res.statusText);
  return data;
}

// ---------------------------------------------------------------------------
// tabs
// ---------------------------------------------------------------------------

document.querySelectorAll(".tab-btn").forEach((btn) => {
  btn.addEventListener("click", () => {
    document.querySelectorAll(".tab-btn").forEach((b) => b.classList.remove("active"));
    document.querySelectorAll(".tab-panel").forEach((p) => p.classList.remove("active"));
    btn.classList.add("active");
    document.getElementById("tab-" + btn.dataset.tab).classList.add("active");
  });
});

// ---------------------------------------------------------------------------
// layout tab
// ---------------------------------------------------------------------------

function nextId(list) {
  const used = new Set(list.map((v) => v.id));
  let i = 0;
  while (used.has(i)) i++;
  return i;
}

function nextGlobalSensorId() {
  const used = new Set([
    ...layout.uavs.flatMap((v) => (v.tags || []).map((t) => t.id)),
    ...layout.ugvs.flatMap((v) => (v.anchors || []).map((a) => a.id)),
  ]);
  let i = 1;
  while (used.has(i)) i++;
  return i;
}

function addVehicle(kind) {
  const list = kind === "uav" ? layout.uavs : layout.ugvs;
  const vehicle = {
    id: nextId(list),
    spawn_pose: [0, 0, 0, 0, 0, 0],
    offboard: JSON.parse(JSON.stringify(kind === "uav" ? DEFAULT_UAV_OFFBOARD : DEFAULT_UGV_OFFBOARD)),
  };
  vehicle[kind === "uav" ? "tags" : "anchors"] = [];
  list.push(vehicle);
  renderVehicles();
}

function addSensor(kind, vehicle) {
  const key = kind === "uav" ? "tags" : "anchors";
  vehicle[key] = vehicle[key] || [];
  vehicle[key].push({ id: nextGlobalSensorId(), position: [0, 0, 0] });
  renderVehicles();
}

function renderSensorRow(kind, vehicle, sensor) {
  const row = el("div", { class: "sensor-row" });
  const idInput = el("input", { type: "number", step: "1", min: "0" });
  idInput.value = sensor.id;
  idInput.addEventListener("change", () => (sensor.id = parseInt(idInput.value, 10) || 0));
  ["x", "y", "z"].forEach((axis, i) => {
    const inp = el("input", { type: "number", step: "0.001" });
    inp.value = sensor.position[i];
    inp.addEventListener("change", () => (sensor.position[i] = parseFloat(inp.value) || 0));
    row.appendChild(el("label", {}, [axis + " ", inp]));
  });
  row.insertBefore(el("label", {}, ["id ", idInput]), row.firstChild);
  const removeBtn = el("button", { class: "remove-sensor", type: "button", title: "Remove" });
  removeBtn.textContent = "×";
  removeBtn.addEventListener("click", () => {
    const key = kind === "uav" ? "tags" : "anchors";
    vehicle[key] = vehicle[key].filter((s) => s !== sensor);
    renderVehicles();
  });
  row.appendChild(removeBtn);
  return row;
}

function renderOffboardFields(vehicle) {
  const grid = el("div", { class: "offboard-grid" });
  const ob = vehicle.offboard;

  const trajSelect = el("select");
  trajectoryOptions.forEach((name) => {
    const opt = el("option", { value: name, text: name });
    if (name === ob.trajectory_csv_file) opt.selected = true;
    trajSelect.appendChild(opt);
  });
  trajSelect.addEventListener("change", () => (ob.trajectory_csv_file = trajSelect.value));
  grid.appendChild(el("label", {}, ["trajectory_csv_file", trajSelect]));

  [
    ["lookahead_distance", 0.1],
    ["cruise_speed", 0.01],
    ["odom_error_position", 0.1],
    ["odom_error_angle", 0.1],
  ].forEach(([field, step]) => {
    const inp = el("input", { type: "number", step: String(step) });
    inp.value = ob[field];
    inp.addEventListener("change", () => (ob[field] = parseFloat(inp.value) || 0));
    grid.appendChild(el("label", {}, [field, inp]));
  });
  return grid;
}

function renderVehicleCard(kind, vehicle) {
  const card = el("div", { class: "vehicle-card", "data-kind": kind });

  const header = el("div", { class: "vehicle-card-header" });
  const idInput = el("input", { type: "number", step: "1", min: "0" });
  idInput.value = vehicle.id;
  idInput.addEventListener("change", () => (vehicle.id = parseInt(idInput.value, 10) || 0));
  header.appendChild(el("label", {}, [kind === "uav" ? "UAV id " : "UGV id ", idInput]));
  const removeBtn = el("button", { class: "remove-vehicle", type: "button" });
  removeBtn.textContent = "Remove";
  removeBtn.addEventListener("click", () => {
    const list = kind === "uav" ? layout.uavs : layout.ugvs;
    const idx = list.indexOf(vehicle);
    if (idx >= 0) list.splice(idx, 1);
    renderVehicles();
  });
  header.appendChild(removeBtn);
  card.appendChild(header);

  const poseSet = el("fieldset", {}, [el("legend", { text: "spawn_pose [x, y, z, roll, pitch, yaw]" })]);
  const poseGrid = el("div", { class: "pose-grid" });
  ["x", "y", "z", "roll", "pitch", "yaw"].forEach((label, i) => {
    const inp = el("input", { type: "number", step: "0.01" });
    inp.value = vehicle.spawn_pose[i];
    inp.addEventListener("change", () => (vehicle.spawn_pose[i] = parseFloat(inp.value) || 0));
    poseGrid.appendChild(el("label", {}, [label, inp]));
  });
  poseSet.appendChild(poseGrid);
  card.appendChild(poseSet);

  const sensorKey = kind === "uav" ? "tags" : "anchors";
  const sensorSet = el("fieldset", {}, [
    el("legend", { text: kind === "uav" ? "tags" : "anchors" }),
  ]);
  const sensorList = el("div", { class: "sensor-list" });
  (vehicle[sensorKey] || []).forEach((sensor) => sensorList.appendChild(renderSensorRow(kind, vehicle, sensor)));
  sensorSet.appendChild(sensorList);
  const addSensorBtn = el("button", { type: "button" });
  addSensorBtn.textContent = kind === "uav" ? "+ Add tag" : "+ Add anchor";
  addSensorBtn.addEventListener("click", () => addSensor(kind, vehicle));
  sensorSet.appendChild(addSensorBtn);
  card.appendChild(sensorSet);

  const obSet = el("fieldset", {}, [el("legend", { text: "offboard" })]);
  obSet.appendChild(renderOffboardFields(vehicle));
  card.appendChild(obSet);

  return card;
}

function renderVehicles() {
  const uavList = document.getElementById("uav-list");
  const ugvList = document.getElementById("ugv-list");
  uavList.innerHTML = "";
  ugvList.innerHTML = "";
  layout.uavs.forEach((v) => uavList.appendChild(renderVehicleCard("uav", v)));
  layout.ugvs.forEach((v) => ugvList.appendChild(renderVehicleCard("ugv", v)));
}

async function refreshLayoutPresets() {
  const names = await api("/api/layouts");
  const select = document.getElementById("layout-preset");
  select.innerHTML = "";
  names.forEach((n) => select.appendChild(el("option", { value: n, text: n })));
}

async function refreshWorlds() {
  const names = await api("/api/worlds");
  const datalist = document.getElementById("world-options");
  names.forEach((n) => datalist.appendChild(el("option", { value: n })));
}

async function refreshTrajectories() {
  trajectoryOptions = await api("/api/trajectories");
}

document.getElementById("add-uav").addEventListener("click", () => addVehicle("uav"));
document.getElementById("add-ugv").addEventListener("click", () => addVehicle("ugv"));

document.getElementById("layout-new").addEventListener("click", () => {
  layout = { uavs: [], ugvs: [], world: "" };
  renderVehicles();
  document.getElementById("world-input").value = "";
  setStatus("layout-status", "New empty layout.");
});

document.getElementById("layout-load").addEventListener("click", async () => {
  const name = document.getElementById("layout-preset").value;
  if (!name) return;
  try {
    const data = await api("/api/layouts/" + encodeURIComponent(name));
    layout = { uavs: data.uavs || [], ugvs: data.ugvs || [], world: data.world || "" };
    layout.uavs.forEach((v) => (v.offboard = v.offboard || JSON.parse(JSON.stringify(DEFAULT_UAV_OFFBOARD))));
    layout.ugvs.forEach((v) => (v.offboard = v.offboard || JSON.parse(JSON.stringify(DEFAULT_UGV_OFFBOARD))));
    renderVehicles();
    document.getElementById("layout-filename").value = name;
    document.getElementById("world-input").value = layout.world;
    setStatus("layout-status", "Loaded " + name + ".", true);
  } catch (e) {
    setStatus("layout-status", e.message, false);
  }
});

document.getElementById("world-input").addEventListener("input", (e) => {
  layout.world = e.target.value.trim();
});

async function saveLayout(name) {
  await api("/api/layouts/" + encodeURIComponent(name), {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(layout),
  });
}

document.getElementById("layout-save").addEventListener("click", async () => {
  const name = document.getElementById("layout-filename").value.trim() || "uwb_layout_gui.yaml";
  try {
    await saveLayout(name);
    document.getElementById("layout-filename").value = name;
    await refreshLayoutPresets();
    setStatus("layout-status", "Saved " + name + ".", true);
  } catch (e) {
    setStatus("layout-status", e.message, false);
  }
});

// ---------------------------------------------------------------------------
// plugin params tab
// ---------------------------------------------------------------------------

let paramGroups = [];
let paramValues = {};

function renderParamRow(key) {
  const ui = PARAM_UI[key] || { type: "text" };
  const row = el("div", { class: "param-row" });
  row.appendChild(el("label", { text: key }));

  if (ui.type === "bool") {
    const cb = el("input", { type: "checkbox" });
    cb.checked = !!paramValues[key];
    cb.addEventListener("change", () => (paramValues[key] = cb.checked));
    row.appendChild(cb);
    row.appendChild(el("span"));
  } else if (ui.type === "slider") {
    const range = el("input", {
      type: "range",
      min: String(ui.min),
      max: String(ui.max),
      step: String(ui.step),
    });
    const num = el("input", { type: "number", step: String(ui.step) });
    range.value = paramValues[key];
    num.value = paramValues[key];
    range.addEventListener("input", () => {
      num.value = range.value;
      paramValues[key] = parseFloat(range.value);
    });
    num.addEventListener("change", () => {
      range.value = num.value;
      paramValues[key] = parseFloat(num.value) || 0;
    });
    row.appendChild(range);
    row.appendChild(num);
  } else if (ui.type === "text") {
    const inp = el("input", { type: "text" });
    inp.value = paramValues[key];
    inp.addEventListener("change", () => (paramValues[key] = inp.value));
    row.appendChild(inp);
    row.appendChild(el("span"));
  } else {
    const inp = el("input", { type: "number", step: String(ui.step || 0.1) });
    inp.value = paramValues[key];
    inp.addEventListener("change", () => (paramValues[key] = parseFloat(inp.value) || 0));
    row.appendChild(inp);
    row.appendChild(el("span"));
  }
  return row;
}

function renderParamGroups() {
  const container = document.getElementById("params-groups");
  container.innerHTML = "";
  paramGroups.forEach((group) => {
    const box = el("div", { class: "param-group" }, [el("h3", { text: group.label })]);
    group.keys.forEach((key) => box.appendChild(renderParamRow(key)));
    container.appendChild(box);
  });
}

async function refreshParamPresets() {
  const names = await api("/api/params/presets");
  const select = document.getElementById("params-preset");
  select.querySelectorAll("option:not(:first-child)").forEach((o) => o.remove());
  names.forEach((n) => select.appendChild(el("option", { value: n, text: n })));
}

document.getElementById("params-load").addEventListener("click", async () => {
  const name = document.getElementById("params-preset").value;
  try {
    if (name) {
      paramValues = await api("/api/params/presets/" + encodeURIComponent(name));
      setStatus("params-status", "Loaded preset " + name + ".", true);
    } else {
      const data = await api("/api/params");
      paramValues = data.values;
      setStatus("params-status", "Loaded current params.yaml.", true);
    }
    renderParamGroups();
  } catch (e) {
    setStatus("params-status", e.message, false);
  }
});

document.getElementById("params-save-preset").addEventListener("click", async () => {
  const name = document.getElementById("params-preset-name").value.trim();
  if (!name) {
    setStatus("params-status", "Enter a preset name first.", false);
    return;
  }
  try {
    await api("/api/params/presets/" + encodeURIComponent(name), {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(paramValues),
    });
    await refreshParamPresets();
    setStatus("params-status", "Saved preset " + name + ".", true);
  } catch (e) {
    setStatus("params-status", e.message, false);
  }
});

async function applyParams() {
  await api("/api/params", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(paramValues),
  });
}

document.getElementById("params-apply").addEventListener("click", async () => {
  try {
    await applyParams();
    setStatus("params-status", "Applied to params.yaml.", true);
  } catch (e) {
    setStatus("params-status", e.message, false);
  }
});

// ---------------------------------------------------------------------------
// continue
// ---------------------------------------------------------------------------

document.getElementById("continue-setup").addEventListener("click", async () => {
  const filename = document.getElementById("layout-filename").value.trim() || "uwb_layout_gui.yaml";
  setStatus("continue-status", "Saving...");
  try {
    await saveLayout(filename);
    await applyParams();
    await api("/api/continue", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ layout_file: filename }),
    });
    setStatus("continue-status", "Done -- continuing setup in your terminal. You can close this tab.", true);
    document.getElementById("continue-setup").disabled = true;
  } catch (e) {
    setStatus("continue-status", e.message, false);
  }
});

// ---------------------------------------------------------------------------
// init
// ---------------------------------------------------------------------------

(async function init() {
  await Promise.all([refreshLayoutPresets(), refreshWorlds(), refreshTrajectories(), refreshParamPresets()]);
  renderVehicles();
  const data = await api("/api/params");
  paramGroups = data.groups;
  paramValues = data.values;
  renderParamGroups();
})();
