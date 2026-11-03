#!/usr/bin/env python3
"""gui/app.py - Local configuration GUI for UWBPX4Sim.

A small Flask app, launched by setup_simulator.sh as one step in its own
pipeline (not a standalone service): it lets the user build/edit a layout
YAML and tune the plugin's parameters (picking/saving a named preset under
uwb_gazebo_plugin/presets/ -- there's no single standalone params.yaml;
every preset is an equal, named file there, same as layouts under config/)
in a browser, then hands control back to setup_simulator.sh via POST
/api/continue, which writes the chosen settings to .gui_selection and
shuts this process down. There is deliberately no database, no build step,
no JS framework -- just Flask + PyYAML (both already project dependencies)
serving a static, vanilla-JS page, kept intentionally small so it costs the
image as little as possible.
"""

from __future__ import annotations

import argparse
import os
import re
import threading
from pathlib import Path
from typing import Any

import yaml
from flask import Flask, jsonify, request, send_from_directory


class LayoutDumper(yaml.SafeDumper):
    """Flow-style ([1, 2, 3]) for flat numeric lists (spawn_pose, position),
    block style for everything else -- matches the hand-written example
    layout files instead of PyYAML's default all-or-nothing choice."""


def _represent_list(dumper: yaml.SafeDumper, data: list) -> yaml.Node:
    flow = len(data) > 0 and all(isinstance(x, (int, float)) for x in data)
    return dumper.represent_sequence("tag:yaml.org,2002:seq", data, flow_style=flow)


LayoutDumper.add_representer(list, _represent_list)

# ---------------------------------------------------------------------------
# Layout schema helpers
# ---------------------------------------------------------------------------

DEFAULT_UAV_OFFBOARD = {
    "trajectory_csv_file": "trajectory_lemniscate_uav.csv",
    "lookahead_distance": 2.0,
    "cruise_speed": 0.5,
    "odom_error_position": 0.0,
    "odom_error_angle": 0.0,
}

DEFAULT_UGV_OFFBOARD = {
    "trajectory_csv_file": "trajectory_lemniscate_agv.csv",
    "lookahead_distance": 2.0,
    "cruise_speed": 0.52,
    "odom_error_position": 0.0,
    "odom_error_angle": 0.0,
}


def load_vehicle_types(vehicles_file: Path) -> list[dict[str, str]]:
    """[{name, role}] from models/vehicles.yaml, in file order (first per role is the default)."""
    if not vehicles_file.is_file():
        return []
    with open(vehicles_file) as f:
        data = yaml.safe_load(f) or {}
    return [
        {"name": name, "role": entry.get("role", "")}
        for name, entry in data.items()
        if isinstance(entry, dict)
    ]


def validate_layout(layout: dict[str, Any], vehicle_types: list[dict[str, str]]) -> list[str]:
    """Returns a list of human-readable errors; empty means valid."""
    errors: list[str] = []
    uavs = layout.get("uavs") or []
    ugvs = layout.get("ugvs") or []

    roles = {t["name"]: t["role"] for t in vehicle_types}
    for role, label, vehicles in (("uav", "UAV", uavs), ("ugv", "UGV", ugvs)):
        for v in vehicles:
            model = v.get("model")
            if model is not None and roles.get(model) != role:
                errors.append(f"{label} {v.get('id')}: model '{model}' is not a {role} type in models/vehicles.yaml.")

    uav_ids = [v.get("id") for v in uavs]
    if len(uav_ids) != len(set(uav_ids)):
        errors.append("UAV ids must be unique.")
    ugv_ids = [v.get("id") for v in ugvs]
    if len(ugv_ids) != len(set(ugv_ids)):
        errors.append("UGV ids must be unique.")

    tag_ids = [t.get("id") for v in uavs for t in (v.get("tags") or [])]
    if len(tag_ids) != len(set(tag_ids)):
        errors.append("Tag ids must be globally unique across all UAVs.")

    anchor_ids = [a.get("id") for v in ugvs for a in (v.get("anchors") or [])]
    if len(anchor_ids) != len(set(anchor_ids)):
        errors.append("Anchor ids must be globally unique across all UGVs.")

    for v in uavs:
        if len(v.get("spawn_pose") or []) != 6:
            errors.append(f"UAV {v.get('id')}: spawn_pose must have 6 values.")
    for v in ugvs:
        if len(v.get("spawn_pose") or []) != 6:
            errors.append(f"UGV {v.get('id')}: spawn_pose must have 6 values.")

    world = layout.get("world")
    if world is not None and not isinstance(world, str):
        errors.append("world must be a string (a world name).")

    return errors


def create_app(uwb_root: Path, selection_file: Path) -> Flask:
    app = Flask(__name__, static_folder=str(Path(__file__).parent / "static"), static_url_path="")

    config_dir = uwb_root / "config"
    worlds_dir = uwb_root / "worlds"
    trajectories_dir = uwb_root / "ROS2" / "px4_sim_offboard" / "trajectories"
    presets_dir = uwb_root / "uwb_gazebo_plugin" / "presets"
    presets_dir.mkdir(parents=True, exist_ok=True)
    vehicles_file = uwb_root / "models" / "vehicles.yaml"
    # No single params.yaml: every plugin-parameter set, including the
    # "default" one, is a named preset under presets_dir, same as layouts
    # are all equal named files under config_dir. Preferred when no ?preset
    # is given and nothing else has been picked yet -- mirrors
    # setup_simulator.sh's own -y fallback (see select_params_preset there).
    DEFAULT_PRESET_NAME = "eliko_identification"

    # (group label, [(key, default)]) -- defines both the canonical write
    # order/section comments for a rendered preset file and the ordered
    # param list the frontend renders. Keep in sync with the "Plugin
    # parameters" table in README.md if either changes.
    PARAM_GROUPS: list[tuple[str, list[tuple[str, Any]]]] = [
        ("Base GZ topics for simulated / ground-truth ranges.", [
            ("topic", "/uwb_gz_simulator/distances"),
            ("ground_truth_topic", "/uwb_gz_simulator/distances_ground_truth"),
        ]),
        ("Additive Gaussian noise mean.", [
            ("gaussian_noise_mean_cm", 0.0),
        ]),
        ("Persistent per-pair bias.", [
            ("apply_pair_bias", False),
            ("bias_min_cm", -27.0),
            ("bias_max_cm", 20.0),
        ]),
        ("NLOS-aware degradation.", [
            ("enable_nlos_dropout", True),
            ("nlos_endpoint_margin_m", 0.02),
        ]),
        ("LOS distance-based dropout curve.", [
            ("los_dropout_start_distance_m", 10.0),
            ("los_dropout_end_distance_m", 70.0),
            ("los_hard_dropout_distance_m", 75.0),
        ]),
        ("Blocked-thickness thresholds.", [
            ("los_max_thickness_m", 0.10),
            ("soft_nlos_max_thickness_m", 0.50),
            ("blackout_thickness_m", 1.0),
            ("hard_nlos_gap_ratio", 0.50),
        ]),
        ("Dropout probability bounds.", [
            ("min_dropout_probability", 0.02),
            ("max_dropout_probability", 0.90),
        ]),
        ("Noise standard-deviation curve.", [
            ("min_noise_stddev_cm", 12.0),
            ("max_noise_stddev_cm", 50.0),
            ("los_stddev_start_distance_m", 15.0),
            ("los_stddev_end_distance_m", 75.0),
        ]),
        ("Extra weight given to blocked thickness vs. distance.", [
            ("nlos_dropout_thickness_extra_weight", 0.50),
            ("nlos_stddev_thickness_extra_weight", 0.50),
        ]),
        ("Ranging mode.", [
            ("uwb_mode", "twr"),
            ("tdoa_topic", "/uwb_gz_simulator/tdoa"),
            ("tdoa_ground_truth_topic", "/uwb_gz_simulator/tdoa_ground_truth"),
        ]),
        ("Material-aware NLOS attenuation.", [
            ("enable_material_attenuation", False),
            ("material_attenuation_default", 1.0),
            ("material_attenuation_metal", 1.0),
            ("material_attenuation_wood", 0.08),
            ("material_attenuation_cardboard", 0.03),
            ("material_attenuation_plastic", 0.15),
            ("material_attenuation_foam", 0.02),
        ]),
    ]
    PARAM_DEFAULTS = {k: v for _label, kv in PARAM_GROUPS for k, v in kv}

    def render_params_yaml(values: dict[str, Any]) -> str:
        lines = [
            "# UWBGazeboSystem plugin parameters -- a preset.",
            "#",
            "# Generated/edited via the setup GUI. setup_simulator.sh reads whichever",
            "# preset was selected (--params, or the terminal/GUI picker) and writes",
            "# its values into the <plugin name=\"custom::UWBGazeboSystem\"> block in",
            "# PX4's server.config on every run.",
            "",
        ]
        for label, kv in PARAM_GROUPS:
            lines.append(f"# {label}")
            for key, default in kv:
                value = values.get(key, default)
                if isinstance(value, bool):
                    text = "true" if value else "false"
                elif isinstance(value, str):
                    text = value
                else:
                    text = str(value)
                lines.append(f"{key}: {text}")
            lines.append("")
        return "\n".join(lines).rstrip() + "\n"

    def safe_yaml_name(name: str) -> str:
        name = os.path.basename(name or "").strip()
        if not name:
            raise ValueError("Empty filename.")
        if not re.fullmatch(r"[A-Za-z0-9_.\-]+", name):
            raise ValueError("Filename may only contain letters, digits, '_', '-', '.'.")
        if not name.endswith((".yaml", ".yml")):
            name += ".yaml"
        return name

    # -- static page ---------------------------------------------------

    @app.get("/")
    def index():
        return send_from_directory(app.static_folder, "index.html")

    # -- layouts ---------------------------------------------------------

    @app.get("/api/layouts")
    def list_layouts():
        names = sorted(p.name for p in config_dir.glob("*.y*ml") if p.is_file())
        return jsonify(names)

    @app.get("/api/layouts/<name>")
    def get_layout(name: str):
        try:
            path = config_dir / safe_yaml_name(name)
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        if not path.is_file():
            return jsonify({"error": f"{name} not found"}), 404
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        return jsonify(data)

    @app.post("/api/layouts/<name>")
    def save_layout(name: str):
        try:
            fname = safe_yaml_name(name)
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        layout = request.get_json(force=True, silent=True) or {}
        errors = validate_layout(layout, load_vehicle_types(vehicles_file))
        if errors:
            return jsonify({"error": " ".join(errors)}), 400
        if not layout.get("world"):
            layout.pop("world", None)  # omit rather than write "world: ''"
        path = config_dir / fname
        with open(path, "w") as f:
            yaml.dump(layout, f, Dumper=LayoutDumper, sort_keys=False, default_flow_style=False)
        return jsonify({"saved": fname})

    # -- worlds / trajectories (read-only listings) -----------------------

    @app.get("/api/worlds")
    def list_worlds():
        return jsonify(sorted(p.stem for p in worlds_dir.glob("*.sdf") if p.is_file()))

    @app.get("/api/trajectories")
    def list_trajectories():
        return jsonify(sorted(p.name for p in trajectories_dir.glob("*.csv") if p.is_file()))

    @app.get("/api/vehicle-types")
    def list_vehicle_types():
        return jsonify(load_vehicle_types(vehicles_file))

    # -- plugin params -----------------------------------------------------

    def list_preset_names() -> list[str]:
        return sorted(p.stem for p in presets_dir.glob("*.yaml") if p.is_file())

    def default_preset_name() -> str | None:
        """Mirrors setup_simulator.sh's select_params_preset -y fallback:
        prefer DEFAULT_PRESET_NAME if present, else the first preset found
        alphabetically, else None if there are no presets at all yet."""
        names = list_preset_names()
        if DEFAULT_PRESET_NAME in names:
            return DEFAULT_PRESET_NAME
        return names[0] if names else None

    @app.get("/api/params")
    def get_params():
        # ?preset=<name> loads that one explicitly; omitted falls back to
        # default_preset_name() -- used on first page load, before the user
        # has picked anything. "preset" in the response is which one (if
        # any) actually got loaded, so the frontend can initialize its
        # current-preset state correctly either way.
        requested = request.args.get("preset")
        preset_name = requested or default_preset_name()
        values = dict(PARAM_DEFAULTS)
        if preset_name:
            try:
                path = presets_dir / safe_yaml_name(preset_name)
            except ValueError as exc:
                return jsonify({"error": str(exc)}), 400
            if requested and not path.is_file():
                return jsonify({"error": f"{requested} not found"}), 404
            if path.is_file():
                with open(path) as f:
                    values.update(yaml.safe_load(f) or {})
        return jsonify({"groups": [{"label": label, "keys": [k for k, _ in kv]} for label, kv in PARAM_GROUPS],
                        "values": values, "preset": preset_name})

    @app.get("/api/params/presets")
    def list_param_presets():
        return jsonify(list_preset_names())

    @app.get("/api/params/presets/<name>")
    def get_param_preset(name: str):
        try:
            path = presets_dir / safe_yaml_name(name)
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        if not path.is_file():
            return jsonify({"error": f"{name} not found"}), 404
        with open(path) as f:
            values = dict(PARAM_DEFAULTS)
            values.update(yaml.safe_load(f) or {})
        return jsonify(values)

    @app.post("/api/params/presets/<name>")
    def save_param_preset(name: str):
        try:
            fname = safe_yaml_name(name)
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        values = request.get_json(force=True, silent=True) or {}
        merged = dict(PARAM_DEFAULTS)
        merged.update(values)
        (presets_dir / fname).write_text(render_params_yaml(merged))
        return jsonify({"saved": fname})

    # -- hand control back to setup_simulator.sh ---------------------------

    @app.post("/api/continue")
    def continue_setup():
        # No separate world handoff here -- the layout YAML itself is the
        # single source of truth for which world to use (its own `world`
        # field, saved by save_layout above); setup_simulator.sh reads that
        # straight from LAYOUT_FILE via `configure_uwb_layout.py --emit-world`.
        body = request.get_json(force=True, silent=True) or {}
        layout_file = body.get("layout_file") or ""
        if not layout_file or not (config_dir / layout_file).is_file():
            return jsonify({"error": "Save the layout before continuing."}), 400

        params_preset = body.get("params_preset") or ""
        try:
            preset_path = presets_dir / safe_yaml_name(params_preset) if params_preset else None
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        if not preset_path or not preset_path.is_file():
            return jsonify({"error": "Save/pick a plugin-parameter preset before continuing."}), 400

        layout_path = str((config_dir / layout_file).resolve())
        with open(selection_file, "w") as f:
            f.write(f"LAYOUT_FILE={_shell_quote(layout_path)}\n")
            f.write(f"PARAMS_PRESET={_shell_quote(preset_path.stem)}\n")

        def _shutdown():
            os._exit(0)

        threading.Timer(0.3, _shutdown).start()
        return jsonify({"ok": True})

    return app


def _shell_quote(value: str) -> str:
    # Minimal POSIX single-quote escaping (files here are simple paths/
    # names produced by this same app, but quote defensively regardless).
    return "'" + value.replace("'", "'\\''") + "'"


def main() -> None:
    parser = argparse.ArgumentParser(description="UWBPX4Sim configuration GUI")
    parser.add_argument("--uwb-root", type=Path, required=True)
    parser.add_argument("--port", type=int, default=5050)
    parser.add_argument("--selection-file", type=Path, required=True)
    args = parser.parse_args()

    app = create_app(args.uwb_root.resolve(), args.selection_file)
    app.run(host="0.0.0.0", port=args.port, debug=False)


if __name__ == "__main__":
    main()
