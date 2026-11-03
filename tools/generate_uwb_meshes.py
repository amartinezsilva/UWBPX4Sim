#!/usr/bin/env python3

"""Generate approximate Eliko-style UWB anchor/tag meshes as ASCII STL."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import List, Sequence, Tuple

Vec3 = Tuple[float, float, float]
Tri = Tuple[Vec3, Vec3, Vec3]


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(a: Vec3) -> Vec3:
    n = math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
    if n < 1e-12:
        return (0.0, 0.0, 0.0)
    return (a[0] / n, a[1] / n, a[2] / n)


def _triangle_normal(tri: Tri) -> Vec3:
    v0, v1, v2 = tri
    return _norm(_cross(_sub(v1, v0), _sub(v2, v0)))


def add_box(tris: List[Tri], *, center: Vec3, size: Vec3) -> None:
    cx, cy, cz = center
    sx, sy, sz = size
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0

    v = [
        (cx - hx, cy - hy, cz - hz),  # 0
        (cx + hx, cy - hy, cz - hz),  # 1
        (cx + hx, cy + hy, cz - hz),  # 2
        (cx - hx, cy + hy, cz - hz),  # 3
        (cx - hx, cy - hy, cz + hz),  # 4
        (cx + hx, cy - hy, cz + hz),  # 5
        (cx + hx, cy + hy, cz + hz),  # 6
        (cx - hx, cy + hy, cz + hz),  # 7
    ]

    tris.extend(
        [
            # bottom
            (v[0], v[2], v[1]),
            (v[0], v[3], v[2]),
            # top
            (v[4], v[5], v[6]),
            (v[4], v[6], v[7]),
            # -Y
            (v[0], v[1], v[5]),
            (v[0], v[5], v[4]),
            # +X
            (v[1], v[2], v[6]),
            (v[1], v[6], v[5]),
            # +Y
            (v[2], v[3], v[7]),
            (v[2], v[7], v[6]),
            # -X
            (v[3], v[0], v[4]),
            (v[3], v[4], v[7]),
        ]
    )


def add_cylinder(
    tris: List[Tri],
    *,
    center: Vec3,
    radius: float,
    height: float,
    segments: int = 24,
) -> None:
    cx, cy, cz = center
    z0 = cz - height / 2.0
    z1 = cz + height / 2.0

    top_center = (cx, cy, z1)
    bot_center = (cx, cy, z0)

    for i in range(segments):
        a0 = 2.0 * math.pi * i / segments
        a1 = 2.0 * math.pi * (i + 1) / segments

        p0b = (cx + radius * math.cos(a0), cy + radius * math.sin(a0), z0)
        p1b = (cx + radius * math.cos(a1), cy + radius * math.sin(a1), z0)
        p0t = (p0b[0], p0b[1], z1)
        p1t = (p1b[0], p1b[1], z1)

        # Side wall
        tris.append((p0b, p1b, p1t))
        tris.append((p0b, p1t, p0t))
        # Top cap (+Z)
        tris.append((top_center, p0t, p1t))
        # Bottom cap (-Z)
        tris.append((bot_center, p1b, p0b))


def write_ascii_stl(path: Path, solid_name: str, tris: Sequence[Tri]) -> None:
    lines: List[str] = [f"solid {solid_name}"]
    for tri in tris:
        nx, ny, nz = _triangle_normal(tri)
        lines.append(f"  facet normal {nx:.7g} {ny:.7g} {nz:.7g}")
        lines.append("    outer loop")
        for vx, vy, vz in tri:
            lines.append(f"      vertex {vx:.7g} {vy:.7g} {vz:.7g}")
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append(f"endsolid {solid_name}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_anchor_mesh() -> List[Tri]:
    tris: List[Tri] = []
    add_box(tris, center=(0.0, 0.0, 0.0), size=(0.060, 0.040, 0.012))
    add_cylinder(tris, center=(0.0, 0.0, 0.008), radius=0.0085, height=0.004, segments=28)
    add_cylinder(tris, center=(0.0, 0.0, 0.019), radius=0.0032, height=0.018, segments=20)
    add_box(tris, center=(0.0, 0.0, -0.009), size=(0.024, 0.020, 0.006))
    add_box(tris, center=(0.0, 0.022, -0.001), size=(0.020, 0.004, 0.003))
    return tris


def build_tag_mesh() -> List[Tri]:
    tris: List[Tri] = []
    add_box(tris, center=(0.0, 0.0, 0.0), size=(0.050, 0.032, 0.010))
    add_cylinder(tris, center=(0.0, 0.0, 0.0065), radius=0.0070, height=0.003, segments=24)
    add_cylinder(tris, center=(0.0, 0.0, 0.0145), radius=0.0028, height=0.013, segments=16)
    add_box(tris, center=(0.0, 0.0, -0.007), size=(0.018, 0.016, 0.004))
    add_box(tris, center=(0.0, -0.0175, -0.001), size=(0.015, 0.003, 0.002))
    return tris


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate approximate UWB anchor/tag STL meshes.")
    parser.add_argument(
        "--uwb-root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
        help="Path to UWBPX4Sim directory (default: script's parent).",
    )
    args = parser.parse_args()

    uwb_root = args.uwb_root.resolve()
    meshes_dir = uwb_root / "models" / "uwb_meshes"
    anchor_out = meshes_dir / "uwb_anchor_eliko.stl"
    tag_out = meshes_dir / "uwb_tag_eliko.stl"

    for out in (anchor_out, tag_out):
        out.parent.mkdir(parents=True, exist_ok=True)

    write_ascii_stl(anchor_out, "uwb_anchor_eliko", build_anchor_mesh())
    write_ascii_stl(tag_out, "uwb_tag_eliko", build_tag_mesh())

    print(f"Wrote {anchor_out}")
    print(f"Wrote {tag_out}")


if __name__ == "__main__":
    main()
