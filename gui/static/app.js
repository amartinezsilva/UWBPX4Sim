// UWBPX4Sim setup GUI. Vanilla JS, no build step, no framework -- kept
// intentionally simple. State lives in the DOM (data-* attributes); forms
// are read back into plain objects only when saving/continuing.

const DEFAULT_UAV_OFFBOARD = {
  trajectory_csv_file: "trajectory_lemniscate_uav.csv",
  lookahead_distance: 2.0,
  cruise_speed: 0.5,
  odom_error_position: 0.0,
  odom_error_angle: 0.0,
  odom_noise_position: 0.0,
  odom_noise_angle: 0.0,
  odom_gyro_bias: 0.0,
};
const DEFAULT_UGV_OFFBOARD = {
  trajectory_csv_file: "trajectory_lemniscate_agv.csv",
  lookahead_distance: 2.0,
  cruise_speed: 0.52,
  odom_error_position: 0.0,
  odom_error_angle: 0.0,
  odom_noise_position: 0.0,
  odom_noise_angle: 0.0,
  odom_gyro_bias: 0.0,
};

// type -> {min, max, step} for sliders; 'text'/'bool'/'number' for the rest.
const PARAM_UI = {
  topic: { type: "text" },
  ground_truth_topic: { type: "text" },
  gaussian_noise_mean_cm: { type: "number", step: 0.1 },
  apply_pair_bias: { type: "bool" },
  bias_min_cm: { type: "number", step: 0.1 },
  bias_max_cm: { type: "number", step: 0.1 },
  uwb_mode: { type: "select", options: ["twr", "tdoa"] },
  tdoa_topic: { type: "text" },
  tdoa_ground_truth_topic: { type: "text" },
  update_rate_hz: { type: "slider", min: 1, max: 75, step: 1 },
  path_loss_exponent: { type: "slider", min: 1, max: 4, step: 0.05 },
  max_link_loss_db: { type: "slider", min: 0, max: 80, step: 0.5 },
  min_noise_stddev_cm: { type: "slider", min: 0, max: 200, step: 1 },
  max_noise_stddev_cm: { type: "slider", min: 0, max: 200, step: 1 },
  noise_start_loss_db: { type: "slider", min: 0, max: 80, step: 0.5 },
  noise_end_loss_db: { type: "slider", min: 0, max: 80, step: 0.5 },
  min_dropout_probability: { type: "slider", min: 0, max: 1, step: 0.01 },
  dropout_center_loss_db: { type: "slider", min: 0, max: 80, step: 0.5 },
  dropout_width_db: { type: "slider", min: 0.1, max: 10, step: 0.1 },
  enable_nlos_dropout: { type: "bool" },
  moving_outlier_probability: { type: "slider", min: 0, max: 0.5, step: 0.005 },
  moving_outlier_mean_cm: { type: "slider", min: 0, max: 100, step: 1 },
  moving_outlier_min_speed_mps: { type: "slider", min: 0, max: 1, step: 0.01 },
  nlos_endpoint_margin_m: { type: "slider", min: 0, max: 1, step: 0.01 },
  nlos_min_loss_db: { type: "slider", min: 0, max: 20, step: 0.5 },
  body_shadow_max_loss_db: { type: "slider", min: 0, max: 40, step: 0.5 },
  enable_material_attenuation: { type: "bool" },
  carrier_frequency_ghz: { type: "slider", min: 3, max: 10, step: 0.1 },
  material_metal_detour_loss_db: { type: "slider", min: 0, max: 60, step: 0.5 },
  enable_dielectric_delay: { type: "bool" },
  power_bias_cm_per_db: { type: "number", step: 0.05 },
  power_bias_ref_loss_db: { type: "number", step: 0.5 },
  random_seed: { type: "number", step: 1 },
};

let trajectoryOptions = [];
// [{name, role}] from models/vehicles.yaml; the first one per role is the default.
let vehicleTypes = [];
let layout = { uavs: [], ugvs: [], static_anchors: [], world: "" };

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
  if (!res.ok) {
    const err = new Error(data.error || res.statusText);
    err.status = res.status;
    throw err;
  }
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
    ...[...layout.uavs, ...layout.ugvs].flatMap((v) => [...(v.tags || []), ...(v.anchors || [])].map((s) => s.id)),
    ...layout.static_anchors.map((a) => a.id),
  ]);
  let i = 1;
  while (used.has(i)) i++;
  return i;
}

function typesForRole(kind) {
  return vehicleTypes.filter((t) => t.role === kind);
}

function addVehicle(kind) {
  const list = kind === "uav" ? layout.uavs : layout.ugvs;
  const defaultType = typesForRole(kind)[0];
  const vehicle = {
    id: nextId(list),
    ...(defaultType ? { model: defaultType.name } : {}),
    spawn_pose: [0, 0, 0, 0, 0, 0],
    offboard: JSON.parse(JSON.stringify(kind === "uav" ? DEFAULT_UAV_OFFBOARD : DEFAULT_UGV_OFFBOARD)),
  };
  vehicle.tags = [];
  vehicle.anchors = [];
  list.push(vehicle);
  renderVehicles();
}

// `owner[key]` holds the sensor list: a robot's "tags" or "anchors", or the
// layout's own "static_anchors".
function addSensor(key, owner) {
  owner[key] = owner[key] || [];
  owner[key].push({ id: nextGlobalSensorId(), position: [0, 0, 0] });
  renderVehicles();
}

function renderSensorRow(key, owner, sensor) {
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
  // Optional fixed range offset; empty keeps the preset's random draw.
  const biasInput = el("input", { type: "number", step: "0.1", placeholder: "random" });
  biasInput.value = sensor.bias_cm ?? "";
  biasInput.title = "Fixed range offset of this transceiver in cm. Empty: drawn at random from the preset.";
  biasInput.addEventListener("change", () => {
    const value = parseFloat(biasInput.value);
    if (Number.isFinite(value)) sensor.bias_cm = value;
    else delete sensor.bias_cm;
  });
  row.appendChild(el("label", {}, ["bias cm ", biasInput]));
  row.insertBefore(el("label", {}, ["id ", idInput]), row.firstChild);
  const removeBtn = el("button", { class: "remove-sensor", type: "button", title: "Remove" });
  removeBtn.textContent = "×";
  removeBtn.addEventListener("click", () => {
    owner[key] = owner[key].filter((s) => s !== sensor);
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
    ["odom_noise_position", 0.01],
    ["odom_noise_angle", 0.05],
    ["odom_gyro_bias", 0.001],
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
  const modelSelect = el("select");
  const roleTypes = typesForRole(kind);
  roleTypes.forEach((t) => {
    const opt = el("option", { value: t.name, text: t.name });
    // A vehicle without `model` uses the role's default (first) type.
    if (t.name === (vehicle.model || roleTypes[0].name)) opt.selected = true;
    modelSelect.appendChild(opt);
  });
  modelSelect.addEventListener("change", () => (vehicle.model = modelSelect.value));
  header.appendChild(el("label", {}, ["model ", modelSelect]));
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

  [["tags", "+ Add tag"], ["anchors", "+ Add anchor"]].forEach(([key, addLabel]) => {
    const sensorSet = el("fieldset", {}, [el("legend", { text: key })]);
    const sensorList = el("div", { class: "sensor-list" });
    (vehicle[key] || []).forEach((sensor) => sensorList.appendChild(renderSensorRow(key, vehicle, sensor)));
    sensorSet.appendChild(sensorList);
    const addSensorBtn = el("button", { type: "button" });
    addSensorBtn.textContent = addLabel;
    addSensorBtn.addEventListener("click", () => addSensor(key, vehicle));
    sensorSet.appendChild(addSensorBtn);
    card.appendChild(sensorSet);
  });

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
  const staticList = document.getElementById("static-anchor-list");
  staticList.innerHTML = "";
  layout.static_anchors.forEach((a) => staticList.appendChild(renderSensorRow("static_anchors", layout, a)));
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

async function refreshVehicleTypes() {
  vehicleTypes = await api("/api/vehicle-types");
}

document.getElementById("add-uav").addEventListener("click", () => addVehicle("uav"));
document.getElementById("add-ugv").addEventListener("click", () => addVehicle("ugv"));
document.getElementById("add-static-anchor").addEventListener("click", () => addSensor("static_anchors", layout));

document.getElementById("layout-new").addEventListener("click", () => {
  layout = { uavs: [], ugvs: [], static_anchors: [], world: "" };
  renderVehicles();
  document.getElementById("world-input").value = "";
  setStatus("layout-status", "New empty layout.");
});

document.getElementById("layout-load").addEventListener("click", async () => {
  const name = document.getElementById("layout-preset").value;
  if (!name) return;
  try {
    const data = await api("/api/layouts/" + encodeURIComponent(name));
    layout = {
      uavs: data.uavs || [],
      ugvs: data.ugvs || [],
      static_anchors: data.static_anchors || [],
      world: data.world || "",
    };
    // Fields a layout leaves out show their defaults, which match configure_uwb_layout.py's.
    layout.uavs.forEach((v) => (v.offboard = { ...DEFAULT_UAV_OFFBOARD, ...(v.offboard || {}) }));
    layout.ugvs.forEach((v) => (v.offboard = { ...DEFAULT_UGV_OFFBOARD, ...(v.offboard || {}) }));
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
let currentParamsPreset = null;
// [{name, builtin}] from /api/params/presets. Built-in presets are
// read-only (the backend refuses to overwrite them).
let paramPresets = [];
// paramValues as last loaded/saved, to tell whether the form has been
// edited since -- Continue only re-saves a preset when it has.
let paramValuesSnapshot = "{}";

function markParamsClean() {
  paramValuesSnapshot = JSON.stringify(paramValues);
}

function paramsDirty() {
  return JSON.stringify(paramValues) !== paramValuesSnapshot;
}

function isBuiltinPreset(name) {
  return paramPresets.some((p) => p.name === name && p.builtin);
}

function suggestCustomPresetName(name) {
  const input = document.getElementById("params-preset-name");
  input.value = name + "_custom";
  input.focus();
  input.select();
}

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
  } else if (ui.type === "select") {
    const select = el("select", {});
    ui.options.forEach((optValue) => select.appendChild(el("option", { value: optValue, text: optValue })));
    select.value = paramValues[key];
    select.addEventListener("change", () => (paramValues[key] = select.value));
    row.appendChild(select);
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

async function refreshParamPresets(selected) {
  paramPresets = await api("/api/params/presets");
  const select = document.getElementById("params-preset");
  select.innerHTML = "";
  paramPresets.forEach((p) =>
    select.appendChild(el("option", { value: p.name, text: p.builtin ? p.name + " (built-in)" : p.name })),
  );
  if (selected && paramPresets.some((p) => p.name === selected)) select.value = selected;
}

document.getElementById("params-load").addEventListener("click", async () => {
  const name = document.getElementById("params-preset").value;
  if (!name) return;
  try {
    paramValues = await api("/api/params/presets/" + encodeURIComponent(name));
    currentParamsPreset = name;
    markParamsClean();
    document.getElementById("params-preset-name").value = name;
    renderParamGroups();
    setStatus(
      "params-status",
      isBuiltinPreset(name)
        ? "Loaded built-in preset " + name + " (read-only: save changes under a new name)."
        : "Loaded preset " + name + ".",
      true,
    );
  } catch (e) {
    setStatus("params-status", e.message, false);
  }
});

// Saves paramValues as preset `name`. Returns false if the user declined
// to overwrite an existing preset; throws on any other failure (including
// trying to save over a built-in preset, which also pre-fills a
// "<name>_custom" suggestion in the name field).
async function saveParamPreset(name) {
  if (isBuiltinPreset(name)) {
    suggestCustomPresetName(name);
    throw new Error(name + " is a built-in preset and can't be overwritten. Save your changes under a new name.");
  }
  const url = "/api/params/presets/" + encodeURIComponent(name);
  const opts = {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(paramValues),
  };
  try {
    await api(url, opts);
  } catch (e) {
    if (e.status !== 409) throw e;
    if (!confirm('Preset "' + name + '" already exists. Overwrite it?')) return false;
    await api(url + "?overwrite=1", opts);
  }
  currentParamsPreset = name;
  markParamsClean();
  await refreshParamPresets(name);
  return true;
}

document.getElementById("params-save-preset").addEventListener("click", async () => {
  const name = document.getElementById("params-preset-name").value.trim();
  if (!name) {
    setStatus("params-status", "Enter a preset name first.", false);
    return;
  }
  try {
    if (await saveParamPreset(name)) {
      setStatus("params-status", "Saved preset " + name + ".", true);
    } else {
      setStatus("params-status", "Not saved: " + name + " was left unchanged.", false);
    }
  } catch (e) {
    setStatus("params-status", e.message, false);
  }
});

// ---------------------------------------------------------------------------
// continue
// ---------------------------------------------------------------------------

document.getElementById("continue-setup").addEventListener("click", async () => {
  const filename = document.getElementById("layout-filename").value.trim() || "uwb_layout_gui.yaml";
  const presetName = document.getElementById("params-preset-name").value.trim() || currentParamsPreset;
  if (!presetName) {
    setStatus("continue-status", "Pick or save a plugin-parameter preset first.", false);
    return;
  }
  setStatus("continue-status", "Saving...");
  try {
    // Re-save the preset only if the form was edited or a different name
    // was typed -- continuing with an untouched preset (built-in or not)
    // uses it as-is. Done before saving the layout, so declining an
    // overwrite doesn't leave a half-finished Continue behind.
    if (presetName !== currentParamsPreset || paramsDirty()) {
      if (!(await saveParamPreset(presetName))) {
        setStatus("continue-status", "Cancelled: pick another preset name, or confirm the overwrite.", false);
        return;
      }
    }
    await saveLayout(filename);
    await api("/api/continue", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        layout_file: filename,
        params_preset: presetName,
        build_px4: document.getElementById("build-px4").checked,
      }),
    });
    setStatus("continue-status", "");
    document.getElementById("continue-setup").disabled = true;
    followSetup();
  } catch (e) {
    setStatus("continue-status", e.message, false);
  }
});

// Shows setup_simulator.sh's progress until it ends, then closes the page.
async function followSetup() {
  const $ = (id) => document.getElementById(id);
  $("progress-overlay").hidden = false;
  let last = null;
  while (true) {
    try {
      last = await api("/api/progress");
    } catch (e) {
      break; // setup stops the GUI once it has reported the end
    }
    $("progress-step").textContent = last.step;
    const log = $("progress-log");
    const atBottom = log.scrollTop + log.clientHeight >= log.scrollHeight - 20;
    log.textContent = last.lines.join("\n");
    if (atBottom) log.scrollTop = log.scrollHeight;
    $("progress-bar").hidden = last.percent === null;
    if (last.percent !== null) $("progress-fill").style.width = last.percent + "%";
    if (last.state !== "running") break;
    await new Promise((r) => setTimeout(r, 1000));
  }
  const final = $("progress-final");
  $("progress-bar").hidden = true;
  if (last && last.state === "done") {
    $("progress-spinner").className = "spinner done";
    $("progress-title").textContent = "Setup finished";
    final.className = "progress-final ok";
    final.textContent = "Launch the simulation from the terminal menu (option 2). This page will close.";
    setTimeout(() => {
      window.close();
      final.textContent = "Launch the simulation from the terminal menu (option 2). You can close this tab.";
    }, 3000);
  } else if (last && last.state === "failed") {
    $("progress-spinner").className = "spinner failed";
    $("progress-title").textContent = "Setup failed";
    final.className = "progress-final error";
    final.textContent = `setup_simulator.sh stopped with exit code ${last.exit_code}. See the log above.`;
  } else {
    $("progress-spinner").className = "spinner failed";
    $("progress-title").textContent = "Lost contact with setup";
    final.className = "progress-final error";
    final.textContent = "Check the terminal for how setup ended.";
  }
}

// ---------------------------------------------------------------------------
// init
// ---------------------------------------------------------------------------

// Re-compiling PX4 is required while the plugin hasn't been built in this container.
api("/api/build-info").then((info) => {
  if (info.plugin_built === false) {
    document.getElementById("build-px4").checked = true;
    document.getElementById("build-px4-note").textContent = "(required: the plugin isn't built yet)";
  }
}).catch(() => {});

(async function init() {
  await Promise.all([
    refreshLayoutPresets(),
    refreshWorlds(),
    refreshTrajectories(),
    refreshVehicleTypes(),
  ]);
  renderVehicles();
  const data = await api("/api/params");
  paramGroups = data.groups;
  paramValues = data.values;
  currentParamsPreset = data.preset || null;
  markParamsClean();
  await refreshParamPresets(currentParamsPreset);
  if (currentParamsPreset) document.getElementById("params-preset-name").value = currentParamsPreset;
  renderParamGroups();
})();
