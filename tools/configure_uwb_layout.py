#!/usr/bin/env python3

"""Configure UWB anchor/tag layout for Gazebo PX4 model variants.

This script updates:
  - models/r1_rover/model.sdf   (uwb_anchor_* links + fixed joints)
  - models/x500_base/model.sdf  (uwb_tag_* links + fixed joints)
  - uwb_bridge.yaml             (all /uwb_gz_simulator/distances/aItJ bridges)

Layout input must be JSON with:
{
  "anchors": [[x, y, z], ...],
  "tags": [[x, y, z], ...]
}

Also regenerates bridge configs for:
  - uwb_gz_simulation/uwb_bridge.yaml
  - px4_sim_offboard/config/uwb_bridge.yaml (if package directory exists)
"""

from __future__ import annotations

import argparse
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence, Tuple


@dataclass(frozen=True)
class UwbLayout:
    anchors: List[Tuple[float, float, float]]
    tags: List[Tuple[float, float, float]]


@dataclass(frozen=True)
class SensorVisualSpec:
    mesh_uri: str
    mesh_scale: Tuple[float, float, float]
    mesh_color: str
    collision_size: Tuple[float, float, float]
    bar_radius: float
    bar_color: str


def _to_xyz_list(raw_points: Sequence[Sequence[float]], field_name: str) -> List[Tuple[float, float, float]]:
    points: List[Tuple[float, float, float]] = []
    for idx, raw in enumerate(raw_points, start=1):
        if not isinstance(raw, (list, tuple)) or len(raw) != 3:
            raise ValueError(f"{field_name}[{idx}] must be a 3-element array [x,y,z].")
        try:
            x = float(raw[0])
            y = float(raw[1])
            z = float(raw[2])
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{field_name}[{idx}] contains non-numeric values.") from exc
        points.append((x, y, z))
    return points


def load_layout(layout_path: Path) -> UwbLayout:
    data = json.loads(layout_path.read_text(encoding="utf-8"))
    if "anchors" not in data or "tags" not in data:
        raise ValueError("Layout JSON must include 'anchors' and 'tags' arrays.")

    anchors = _to_xyz_list(data["anchors"], "anchors")
    tags = _to_xyz_list(data["tags"], "tags")

    if not anchors:
        raise ValueError("Layout must include at least one anchor.")
    if not tags:
        raise ValueError("Layout must include at least one tag.")

    return UwbLayout(anchors=anchors, tags=tags)


@dataclass(frozen=True)
class BlockBounds:
    start: int
    end: int
    base_indent: str
    indent_step: str


def _find_block_bounds(text: str, prefix: str) -> BlockBounds:
    first_link = re.search(rf'^(?P<indent>[ \t]*)<link name="{re.escape(prefix)}_\d+">', text, flags=re.M)
    if not first_link:
        raise RuntimeError(f"Could not find first link for prefix '{prefix}'.")

    first_link_start = first_link.start()
    base_indent = first_link.group("indent")

    joint_pattern = re.compile(
        rf'^[ \t]*<joint name="{re.escape(prefix)}_\d+_joint" type="fixed">[\s\S]*?^[ \t]*</joint>\n?',
        flags=re.M,
    )
    joint_matches = list(joint_pattern.finditer(text))
    if not joint_matches:
        raise RuntimeError(f"Could not find any fixed joints for prefix '{prefix}'.")

    end = joint_matches[-1].end()

    link_head_pattern = re.compile(
        rf'^(?P<link_indent>[ \t]*)<link name="{re.escape(prefix)}_\d+">\n(?P<inner_indent>[ \t]+)<pose>',
        flags=re.M,
    )
    head = link_head_pattern.search(text, pos=first_link_start)
    if head and head.group("inner_indent").startswith(head.group("link_indent")):
        indent_step = head.group("inner_indent")[len(head.group("link_indent")) :]
        if not indent_step:
            indent_step = "  "
    else:
        indent_step = "\t" if "\t" in base_indent else "  "

    return BlockBounds(start=first_link_start, end=end, base_indent=base_indent, indent_step=indent_step)


def _fmt_pose(x: float, y: float, z: float) -> str:
    return f"{x:g} {y:g} {z:g} 0 0 0"


def _rotation_matrix_from_axis_angle(
    axis: Tuple[float, float, float], angle: float
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]:
    ax, ay, az = axis
    c = math.cos(angle)
    s = math.sin(angle)
    one_c = 1.0 - c
    return (
        (c + ax * ax * one_c, ax * ay * one_c - az * s, ax * az * one_c + ay * s),
        (ay * ax * one_c + az * s, c + ay * ay * one_c, ay * az * one_c - ax * s),
        (az * ax * one_c - ay * s, az * ay * one_c + ax * s, c + az * az * one_c),
    )


def _rpy_from_rotation_matrix(
    rot: Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]
) -> Tuple[float, float, float]:
    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    r00, r01, r02 = rot[0]
    r10, r11, r12 = rot[1]
    r20, r21, r22 = rot[2]
    pitch = math.asin(max(-1.0, min(1.0, -r20)))
    roll = math.atan2(r21, r22)
    yaw = math.atan2(r10, r00)
    return roll, pitch, yaw


def _rotation_rpy_from_z_to_vector(vec: Tuple[float, float, float]) -> Tuple[float, float, float]:
    vx, vy, vz = vec
    norm = math.sqrt(vx * vx + vy * vy + vz * vz)
    if norm < 1e-9:
        return 0.0, 0.0, 0.0

    ux, uy, uz = vx / norm, vy / norm, vz / norm
    # Rotate local +Z to target unit vector u.
    # axis = z x u = (-uy, ux, 0), angle = acos(z.u) = acos(uz)
    axis_x, axis_y, axis_z = -uy, ux, 0.0
    axis_norm = math.sqrt(axis_x * axis_x + axis_y * axis_y + axis_z * axis_z)

    if axis_norm < 1e-9:
        if uz >= 0.0:
            return 0.0, 0.0, 0.0
        # 180 deg around X maps +Z to -Z
        return math.pi, 0.0, 0.0

    axis = (axis_x / axis_norm, axis_y / axis_norm, axis_z / axis_norm)
    angle = math.acos(max(-1.0, min(1.0, uz)))
    rot = _rotation_matrix_from_axis_angle(axis, angle)
    return _rpy_from_rotation_matrix(rot)


def _gen_sensor_link_block(
    *,
    prefix: str,
    idx: int,
    position: Tuple[float, float, float],
    base_indent: str,
    step: str,
    spec: SensorVisualSpec,
    label: str,
) -> str:
    i1 = base_indent + step
    i2 = i1 + step
    i3 = i2 + step
    i4 = i3 + step

    pose = _fmt_pose(*position)
    # Visual bar from sensor location to base_link origin so outboard sensors don't look floating.
    sx, sy, sz = position
    bar_vec = (-sx, -sy, -sz)
    bar_len = math.sqrt(bar_vec[0] * bar_vec[0] + bar_vec[1] * bar_vec[1] + bar_vec[2] * bar_vec[2])
    bar_block = ""
    if bar_len > 1e-6:
        bar_roll, bar_pitch, bar_yaw = _rotation_rpy_from_z_to_vector(bar_vec)
        bar_block = (
            f"{i1}<visual name=\"{prefix}_{idx}_bar_vis\">\n"
            f"{i2}<pose>{bar_vec[0] / 2.0:g} {bar_vec[1] / 2.0:g} {bar_vec[2] / 2.0:g} "
            f"{bar_roll:g} {bar_pitch:g} {bar_yaw:g}</pose>\n"
            f"{i2}<geometry>\n"
            f"{i3}<cylinder>\n"
            f"{i4}<radius>{spec.bar_radius:g}</radius>\n"
            f"{i4}<length>{bar_len:g}</length>\n"
            f"{i3}</cylinder>\n"
            f"{i2}</geometry>\n"
            f"{i2}<material>\n"
            f"{i3}<ambient>{spec.bar_color}</ambient>\n"
            f"{i3}<diffuse>{spec.bar_color}</diffuse>\n"
            f"{i2}</material>\n"
            f"{i1}</visual>\n"
        )

    return (
        f'{base_indent}<link name="{prefix}_{idx}">\n'
        f"{i1}<pose>{pose}</pose> <!-- {label} position relative to base_link -->\n"
        f"{i1}<inertial>\n"
        f"{i2}<mass>0.0001</mass> <!-- Very small mass -->\n"
        f"{i2}<inertia>\n"
        f"{i3}<ixx>1e-6</ixx>\n"
        f"{i3}<iyy>1e-6</iyy>\n"
        f"{i3}<izz>1e-6</izz>\n"
        f"{i3}<ixy>0</ixy>\n"
        f"{i3}<ixz>0</ixz>\n"
        f"{i3}<iyz>0</iyz>\n"
        f"{i2}</inertia>\n"
        f"{i1}</inertial>\n"
        f"{i1}<visual name=\"{prefix}_{idx}_mesh_vis\">\n"
        f"{i2}<pose>0 0 0 0 0 0</pose>\n"
        f"{i2}<geometry>\n"
        f"{i3}<mesh>\n"
        f"{i4}<uri>{spec.mesh_uri}</uri>\n"
        f"{i4}<scale>{spec.mesh_scale[0]:g} {spec.mesh_scale[1]:g} {spec.mesh_scale[2]:g}</scale>\n"
        f"{i3}</mesh>\n"
        f"{i2}</geometry>\n"
        f"{i2}<material>\n"
        f"{i3}<ambient>{spec.mesh_color}</ambient>\n"
        f"{i3}<diffuse>{spec.mesh_color}</diffuse>\n"
        f"{i2}</material>\n"
        f"{i1}</visual>\n"
        f"{bar_block}"
        f"{i1}<collision name=\"{prefix}_{idx}_collision\">\n"
        f"{i2}<pose>0 0 0 0 0 0</pose>\n"
        f"{i2}<geometry>\n"
        f"{i3}<box>\n"
        f"{i4}<size>{spec.collision_size[0]:g} {spec.collision_size[1]:g} {spec.collision_size[2]:g}</size>\n"
        f"{i3}</box>\n"
        f"{i2}</geometry>\n"
        f"{i1}</collision>\n"
        f"{base_indent}</link>\n"
    )


def _gen_sensor_joint_block(prefix: str, idx: int, base_indent: str, step: str) -> str:
    i1 = base_indent + step
    return (
        f'{base_indent}<joint name="{prefix}_{idx}_joint" type="fixed">\n'
        f"{i1}<parent>base_link</parent>\n"
        f"{i1}<child>{prefix}_{idx}</child>\n"
        f"{base_indent}</joint>\n"
    )


def _gen_sensor_block(
    *,
    prefix: str,
    positions: Sequence[Tuple[float, float, float]],
    base_indent: str,
    step: str,
    spec: SensorVisualSpec,
    label: str,
) -> str:
    blocks: List[str] = []
    for idx, pos in enumerate(positions, start=1):
        blocks.append(
            _gen_sensor_link_block(
                prefix=prefix,
                idx=idx,
                position=pos,
                base_indent=base_indent,
                step=step,
                spec=spec,
                label=label,
            )
        )
        blocks.append(_gen_sensor_joint_block(prefix, idx, base_indent, step))
        blocks.append("")
    return "\n".join(blocks).rstrip() + "\n"


def update_model_file(
    model_path: Path,
    *,
    prefix: str,
    positions: Sequence[Tuple[float, float, float]],
    spec: SensorVisualSpec,
    label: str,
) -> None:
    text = model_path.read_text(encoding="utf-8")
    bounds = _find_block_bounds(text, prefix)

    replacement = _gen_sensor_block(
        prefix=prefix,
        positions=positions,
        base_indent=bounds.base_indent,
        step=bounds.indent_step,
        spec=spec,
        label=label,
    )

    updated = text[: bounds.start] + replacement + text[bounds.end :]
    model_path.write_text(updated, encoding="utf-8")


def generate_bridge_yaml(layout: UwbLayout) -> str:
    lines: List[str] = []
    lines.append("## ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=uwb_bridge.yaml")
    lines.append("")
    lines.append("# UWB distance topic bridges (GZ_TO_ROS only)")

    for anchor_idx in range(1, len(layout.anchors) + 1):
        for tag_idx in range(1, len(layout.tags) + 1):
            topic = f"/uwb_gz_simulator/distances/a{anchor_idx}t{tag_idx}"
            lines.extend(
                [
                    f'- ros_topic_name: "{topic}"',
                    f'  gz_topic_name: "{topic}"',
                    '  ros_type_name: "std_msgs/msg/Float64"',
                    '  gz_type_name: "gz.msgs.Double"',
                    "  subscriber_queue: 10",
                    "  publisher_queue: 10",
                    "  lazy: false",
                    "  direction: GZ_TO_ROS",
                    "",
                ]
            )

    while lines and lines[-1] == "":
        lines.pop()
    lines.append("")
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description="Configure UWB anchor/tag layout in Gazebo model files.")
    parser.add_argument(
        "--layout",
        required=True,
        type=Path,
        help="Path to layout JSON file with 'anchors' and 'tags' arrays.",
    )
    parser.add_argument(
        "--uwb-root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
        help="Path to uwb_gz_simulation directory (default: script's parent).",
    )
    parser.add_argument(
        "--skip-bridge",
        action="store_true",
        help="Do not regenerate uwb_bridge.yaml.",
    )
    args = parser.parse_args()

    layout = load_layout(args.layout)
    uwb_root = args.uwb_root.resolve()

    rover_model = uwb_root / "models" / "r1_rover" / "model.sdf"
    uav_model = uwb_root / "models" / "x500_base" / "model.sdf"
    bridge_yaml = uwb_root / "uwb_bridge.yaml"
    offboard_bridge_yaml = uwb_root.parent / "px4_sim_offboard" / "config" / "uwb_bridge.yaml"

    for path in (rover_model, uav_model):
        if not path.exists():
            raise FileNotFoundError(f"Model file not found: {path}")
    if not args.skip_bridge and not bridge_yaml.exists():
        raise FileNotFoundError(f"Bridge config file not found: {bridge_yaml}")

    update_model_file(
        rover_model,
        prefix="uwb_anchor",
        positions=layout.anchors,
        spec=SensorVisualSpec(
            mesh_uri="model://r1_rover/meshes/uwb_anchor_eliko.stl",
            mesh_scale=(1.0, 1.0, 1.0),
            mesh_color="0.92 0.92 0.92 1",
            collision_size=(0.062, 0.046, 0.040),
            bar_radius=0.004,
            bar_color="0.28 0.28 0.30 1",
        ),
        label="Anchor",
    )
    update_model_file(
        uav_model,
        prefix="uwb_tag",
        positions=layout.tags,
        spec=SensorVisualSpec(
            mesh_uri="model://x500_base/meshes/uwb_tag_eliko.stl",
            mesh_scale=(1.0, 1.0, 1.0),
            mesh_color="0.96 0.55 0.13 1",
            collision_size=(0.052, 0.036, 0.030),
            bar_radius=0.0035,
            bar_color="0.24 0.24 0.24 1",
        ),
        label="Tag",
    )

    if not args.skip_bridge:
        bridge_text = generate_bridge_yaml(layout)
        bridge_yaml.write_text(bridge_text, encoding="utf-8")

        if offboard_bridge_yaml.parent.exists():
            offboard_bridge_yaml.write_text(bridge_text, encoding="utf-8")

    print(
        f"Updated UWB layout: {len(layout.anchors)} anchors, {len(layout.tags)} tags\n"
        f"  - {rover_model}\n"
        f"  - {uav_model}"
        + (
            f"\n  - {bridge_yaml}"
            + (f"\n  - {offboard_bridge_yaml}" if offboard_bridge_yaml.parent.exists() else "")
            if not args.skip_bridge
            else ""
        )
    )


if __name__ == "__main__":
    main()
