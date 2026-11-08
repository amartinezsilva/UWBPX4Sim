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

    tag_ids = [t.get("id") for v in uavs + ugvs for t in (v.get("tags") or [])]
    if len(tag_ids) != len(set(tag_ids)):
        errors.append("Tag ids must be globally unique.")

    static_anchors = layout.get("static_anchors") or []
    anchor_ids = [a.get("id") for v in uavs + ugvs for a in (v.get("anchors") or [])]
    anchor_ids += [a.get("id") for a in static_anchors]
    if len(anchor_ids) != len(set(anchor_ids)):
        errors.append("Anchor ids must be globally unique.")

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


def create_app(uwb_root: Path, selection_file: Path, progress_log: Path | None = None,
               status_file: Path | None = None, px4_dir: Path | None = None) -> Flask:
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
    DEFAULT_PRESET_NAME = "eliko_twr"
    # Hand-written, calibration-annotated presets shipped with the repo.
    # Read-only from the GUI: saving re-renders a preset from PARAM_GROUPS,
    # which would drop their explanatory comments -- tune one, then save it
    # under a new name instead.
    BUILTIN_PRESETS = frozenset({"eliko_twr", "lps_tdoa", "lps_twr"})

    # (group label, [(key, default)]) -- defines both the canonical write
    # order/section comments for a rendered preset file and the ordered
    # param list the frontend renders. Keep in sync with the "Parameter
    # reference" table in README.md if either changes. Per-material
    # material_<name>_attenuation_db_per_m / _permittivity overrides are
    # deliberately not listed: every listed key is written into a saved
    # preset, which would freeze the rates the plugin otherwise derives from
    # carrier_frequency_ghz. Add them to a preset by hand when needed.
    PARAM_GROUPS: list[tuple[str, list[tuple[str, Any]]]] = [
        ("Ranging mode and topics.", [
            ("uwb_mode", "twr"),
            ("topic", "/uwb_gz_simulator/distances"),
            ("ground_truth_topic", "/uwb_gz_simulator/distances_ground_truth"),
            ("tdoa_topic", "/uwb_gz_simulator/tdoa"),
            ("tdoa_ground_truth_topic", "/uwb_gz_simulator/tdoa_ground_truth"),
            ("update_rate_hz", 10.0),
        ]),
        ("Link budget (dB; with n = 2, 23.5 dB = 15 m and 37.5 dB = 75 m).", [
            ("path_loss_exponent", 2.0),
            ("max_link_loss_db", 37.5),
        ]),
        ("Noise standard deviation vs. loss.", [
            ("min_noise_stddev_cm", 12.0),
            ("max_noise_stddev_cm", 50.0),
            ("noise_start_loss_db", 23.5),
            ("noise_end_loss_db", 37.5),
        ]),
        ("Dropout vs. loss.", [
            ("min_dropout_probability", 0.02),
            ("dropout_center_loss_db", 35.5),
            ("dropout_width_db", 1.4),
            ("enable_nlos_dropout", True),
        ]),
        ("Obstacles.", [
            ("nlos_endpoint_margin_m", 0.02),
            ("nlos_min_loss_db", 3.0),
            ("body_shadow_max_loss_db", 10.0),
        ]),
        ("Materials (obstacles tagged <uwb:material> or named *__material_<name>).", [
            ("enable_material_attenuation", False),
            ("carrier_frequency_ghz", 4.0),
            ("material_metal_detour_loss_db", 20.0),
            ("enable_dielectric_delay", False),
        ]),
        ("Additive noise mean and persistent bias.", [
            ("gaussian_noise_mean_cm", 0.0),
            ("moving_outlier_probability", 0.0),
            ("moving_outlier_mean_cm", 30.0),
            ("moving_outlier_min_speed_mps", 0.1),
            ("apply_pair_bias", False),
            ("bias_min_cm", -27.0),
            ("bias_max_cm", 20.0),
            ("power_bias_cm_per_db", 0.0),
            ("power_bias_ref_loss_db", 0.0),
            ("random_seed", 0),
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
        def fmt(value: Any) -> str:
            if isinstance(value, bool):
                return "true" if value else "false"
            return str(value)

        for label, kv in PARAM_GROUPS:
            lines.append(f"# {label}")
            for key, default in kv:
                lines.append(f"{key}: {fmt(values.get(key, default))}")
            lines.append("")
        # Keys the GUI doesn't render (e.g. hand-added material_<name>_*
        # overrides) are kept as-is rather than dropped on save.
        extra = [k for k in values if k not in PARAM_DEFAULTS]
        if extra:
            lines.append("# Other parameters (not editable in the GUI).")
            lines.extend(f"{key}: {fmt(values[key])}" for key in extra)
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

    # The repo's own logo, served from images/ rather than copied here.
    @app.get("/logo.png")
    def logo():
        return send_from_directory(uwb_root / "images", "Logo.png")

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
        if not layout.get("static_anchors"):
            layout.pop("static_anchors", None)
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
        return jsonify([{"name": n, "builtin": n in BUILTIN_PRESETS} for n in list_preset_names()])

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
        path = presets_dir / fname
        if path.stem in BUILTIN_PRESETS:
            return jsonify({"error": f"{path.stem} is a built-in preset and can't be overwritten; "
                                     f"save your changes under a new name (e.g. {path.stem}_custom)."}), 403
        # ?overwrite=1 is the frontend's confirmation that replacing an
        # existing preset is intended; without it, an existing file is a 409
        # the frontend turns into an "Overwrite?" prompt.
        if path.is_file() and request.args.get("overwrite") != "1":
            return jsonify({"error": f"{path.stem} already exists.", "exists": True}), 409
        values = request.get_json(force=True, silent=True) or {}
        merged = dict(PARAM_DEFAULTS)
        merged.update(values)
        path.write_text(render_params_yaml(merged))
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
            f.write(f"BUILD_PX4={'yes' if body.get('build_px4') else 'no'}\n")

        # setup_simulator.sh keeps this GUI running to show its progress and
        # stops it when done. Without a progress log, hand back right away.
        if progress_log is None:
            threading.Timer(0.3, lambda: os._exit(0)).start()
        return jsonify({"ok": True})

    # -- setup progress, shown in the GUI after "Continue Setup" -----------

    @app.get("/api/build-info")
    def build_info():
        plugin = None
        if px4_dir is not None:
            plugin = (px4_dir / "build" / "px4_sitl_default" / "src" / "modules" / "simulation"
                      / "gz_plugins" / "libUWBGazeboPlugin.so").is_file()
        return jsonify({"plugin_built": plugin})

    @app.get("/api/progress")
    def progress():
        text = ""
        if progress_log is not None and progress_log.is_file():
            with open(progress_log, "rb") as f:
                f.seek(0, os.SEEK_END)
                f.seek(max(0, f.tell() - 200_000))
                text = f.read().decode("utf-8", errors="replace")
        lines = [ANSI_RE.sub("", line).rstrip("\r") for line in text.split("\n")]
        lines = [line for line in lines if line.strip()]
        step = next((line[4:] for line in reversed(lines) if line.startswith("==> ")), "Starting...")
        step_start = max((i for i, line in enumerate(lines) if line.startswith("==> ")), default=0)
        percent = None
        for line in reversed(lines[step_start:]):
            match = BUILD_PROGRESS_RE.search(line)
            if match and int(match.group(2)) > 0:
                percent = round(100 * int(match.group(1)) / int(match.group(2)))
                break
        exit_code = None
        if status_file is not None and status_file.is_file():
            content = status_file.read_text().strip()
            exit_code = int(content) if content.lstrip("-").isdigit() else None
        state = "running" if exit_code is None else ("done" if exit_code == 0 else "failed")
        return jsonify({"state": state, "exit_code": exit_code, "step": step,
                        "percent": percent, "lines": lines[-300:]})

    return app


# Colour codes in setup's output, and build progress like "[123/456]".
ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
BUILD_PROGRESS_RE = re.compile(r"\[(\d+)/(\d+)\]")


def _shell_quote(value: str) -> str:
    # Minimal POSIX single-quote escaping (files here are simple paths/
    # names produced by this same app, but quote defensively regardless).
    return "'" + value.replace("'", "'\\''") + "'"


def main() -> None:
    parser = argparse.ArgumentParser(description="UWBPX4Sim configuration GUI")
    parser.add_argument("--uwb-root", type=Path, required=True)
    parser.add_argument("--port", type=int, default=5050)
    parser.add_argument("--selection-file", type=Path, required=True)
    parser.add_argument("--progress-log", type=Path, help="setup_simulator.sh output to show after Continue")
    parser.add_argument("--status-file", type=Path, help="Gets setup_simulator.sh's exit code when it ends")
    parser.add_argument("--px4-dir", type=Path, help="PX4 checkout, to tell whether the plugin is built")
    args = parser.parse_args()

    app = create_app(args.uwb_root.resolve(), args.selection_file, args.progress_log, args.status_file, args.px4_dir)
    app.run(host="0.0.0.0", port=args.port, debug=False)


if __name__ == "__main__":
    main()
