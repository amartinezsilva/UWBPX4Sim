import importlib.util
import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


LAYOUT_FILE_ENV = "UWB_LAYOUT_FILE"
LAYOUT_TOOL_RELATIVE = ("tools", "configure_uwb_layout.py")


def _layout_file() -> Path:
    layout_file = os.environ.get(LAYOUT_FILE_ENV)
    if not layout_file:
        raise RuntimeError(
            f"{LAYOUT_FILE_ENV} is not set. Point it to the master layout YAML before launching offboard nodes."
        )

    path = Path(layout_file)
    if not path.exists():
        raise RuntimeError(f"Layout YAML not found: {path}")
    return path.resolve()


def _uwb_root(layout_file: Path) -> Path:
    # Layouts normally live in <repo>/config/. Elsewhere in the repo, the
    # root is the nearest parent holding the layout tool.
    if layout_file.parent.name == "config":
        return layout_file.parent.parent
    for parent in layout_file.parents:
        if parent.joinpath(*LAYOUT_TOOL_RELATIVE).exists():
            return parent
    raise RuntimeError(
        f"Layout YAML is neither under a config/ directory nor inside a tree holding "
        f"{'/'.join(LAYOUT_TOOL_RELATIVE)}: {layout_file}"
    )


def _layout_tool_module(layout_file: Path):
    uwb_root = _uwb_root(layout_file)
    tool_path = uwb_root.joinpath(*LAYOUT_TOOL_RELATIVE)
    if not tool_path.exists():
        raise RuntimeError(f"Layout tool not found: {tool_path}")

    spec = importlib.util.spec_from_file_location("uwb_layout_tool", tool_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load layout tool module from {tool_path}")

    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def generate_launch_description():
    layout_file = _layout_file()
    layout_tool = _layout_tool_module(layout_file)
    layout = layout_tool.load_layout(layout_file)
    node_entries = layout_tool.build_offboard_launch_entries(layout)

    if not node_entries:
        raise RuntimeError(
            f"No offboard node entries could be derived from {layout_file}."
        )

    actions = []
    for node_name, executable, ros_params in node_entries:
        actions.append(
            Node(
                package='px4_sim_offboard',
                executable=executable,
                name=node_name,
                output='screen',
                parameters=[ros_params],
            )
        )

    # Static anchors belong to no robot, so no offboard node publishes their TF.
    for anchor in layout.static_anchors:
        x, y, z = anchor.position
        actions.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'static_anchor_{anchor.id}_tf',
                arguments=[
                    '--x', str(x), '--y', str(y), '--z', str(z),
                    '--frame-id', 'map',
                    '--child-frame-id', f'static_anchors/a{anchor.id}',
                ],
            )
        )

    actions.append(
        Node(
            package='px4_sim_offboard',
            executable='clock_publisher',
            name='clock_publisher',
        )
    )

    return LaunchDescription(actions)
