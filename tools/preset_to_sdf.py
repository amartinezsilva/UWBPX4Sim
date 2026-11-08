#!/usr/bin/env python3

"""Print a plugin-parameter preset as the plugin's SDF <plugin> block.

Paste the output into a world file to use a preset without the full simulator:

  python3 tools/preset_to_sdf.py eliko_twr

A preset is a name under uwb_gazebo_plugin/presets/, or a path to a YAML file.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, Mapping

import yaml

PLUGIN_NAME = "custom::UWBGazeboSystem"
PLUGIN_FILENAME = "libUWBGazeboPlugin.so"
PRESETS_DIR = Path(__file__).resolve().parents[1] / "uwb_gazebo_plugin" / "presets"


def load_preset(preset: str | Path) -> dict[str, Any]:
    path = Path(preset)
    if not path.is_file():
        path = PRESETS_DIR / f"{preset}.yaml"
    if not path.is_file():
        raise ValueError(f"Preset not found: {preset}")

    params = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(params, dict):
        raise ValueError(f"{path} must contain a mapping of parameter: value")
    return params


def _fmt(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def plugin_block(params: Mapping[str, Any], *, server_config: bool = False) -> str:
    """`server_config` adds the attributes PX4's server.config needs to attach
    the plugin to the world. Inside a world file it is already attached."""
    attributes = 'entity_name="*" entity_type="world" ' if server_config else ""
    lines = [f'    <plugin {attributes}filename="{PLUGIN_FILENAME}" name="{PLUGIN_NAME}">']
    lines += [f"      <{key}>{_fmt(value)}</{key}>" for key, value in params.items()]
    lines.append("    </plugin>")
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description="Print a preset as the plugin's SDF block.")
    parser.add_argument("preset", help="Preset name under uwb_gazebo_plugin/presets/, or a path to a YAML file.")
    args = parser.parse_args()
    print(plugin_block(load_preset(args.preset)), end="")


if __name__ == "__main__":
    main()
