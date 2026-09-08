#!/usr/bin/env python3
"""gui/app.py - Local configuration GUI for UWBPX4Sim.

A small Flask app, launched by setup_simulator.sh as one step in its own
pipeline (not a standalone service): it lets the user build/edit a layout
YAML and tune the plugin's params.yaml in a browser, then hands control
back to setup_simulator.sh via POST /api/continue, which writes the chosen
settings to .gui_selection and shuts this process down. There is
deliberately no database, no build step, no JS framework -- just Flask +
PyYAML (both already project dependencies) serving a static, vanilla-JS
page, kept intentionally small so it costs the image as little as possible.
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


def validate_layout(layout: dict[str, Any]) -> list[str]:
    """Returns a list of human-readable errors; empty means valid."""
    errors: list[str] = []
    uavs = layout.get("uavs") or []
    ugvs = layout.get("ugvs") or []

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

    return errors


def create_app(uwb_root: Path, selection_file: Path) -> Flask:
    app = Flask(__name__, static_folder=str(Path(__file__).parent / "static"), static_url_path="")

    config_dir = uwb_root / "config"
    worlds_dir = uwb_root / "worlds"
    trajectories_dir = uwb_root / "ROS2" / "px4_sim_offboard" / "trajectories"
    params_file = uwb_root / "uwb_gazebo_plugin" / "params.yaml"
    presets_dir = uwb_root / "uwb_gazebo_plugin" / "presets"
    presets_dir.mkdir(parents=True, exist_ok=True)

    # (group label, [(key, default)]) -- defines both the canonical write
    # order/section comments for params.yaml and the ordered param list the
    # frontend renders. Keep in sync with uwb_gazebo_plugin/params.yaml and
    # the "Plugin parameters" table in README.md if either changes.
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
    ]
    PARAM_DEFAULTS = {k: v for _label, kv in PARAM_GROUPS for k, v in kv}

    def render_params_yaml(values: dict[str, Any]) -> str:
        lines = [
            "# UWBGazeboSystem plugin parameters.",
            "#",
            "# Generated/edited via the setup GUI. setup_simulator.sh reads this file",
            "# and writes its values into the <plugin name=\"custom::UWBGazeboSystem\">",
            "# block in PX4's server.config on every run.",
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
        errors = validate_layout(layout)
        if errors:
            return jsonify({"error": " ".join(errors)}), 400
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

    # -- plugin params -----------------------------------------------------

    @app.get("/api/params")
    def get_params():
        values = dict(PARAM_DEFAULTS)
        if params_file.is_file():
            with open(params_file) as f:
                values.update(yaml.safe_load(f) or {})
        return jsonify({"groups": [{"label": label, "keys": [k for k, _ in kv]} for label, kv in PARAM_GROUPS],
                        "values": values})

    @app.post("/api/params")
    def save_params():
        values = request.get_json(force=True, silent=True) or {}
        merged = dict(PARAM_DEFAULTS)
        merged.update(values)
        params_file.parent.mkdir(parents=True, exist_ok=True)
        params_file.write_text(render_params_yaml(merged))
        return jsonify({"saved": True})

    @app.get("/api/params/presets")
    def list_param_presets():
        return jsonify(sorted(p.stem for p in presets_dir.glob("*.yaml") if p.is_file()))

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
        body = request.get_json(force=True, silent=True) or {}
        layout_file = body.get("layout_file") or ""
        world = body.get("world") or ""
        if not layout_file or not (config_dir / layout_file).is_file():
            return jsonify({"error": "Save the layout before continuing."}), 400

        layout_path = str((config_dir / layout_file).resolve())
        with open(selection_file, "w") as f:
            f.write(f"LAYOUT_FILE={_shell_quote(layout_path)}\n")
            if world:
                f.write(f"WORLD_ARG={_shell_quote(world)}\n")

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
