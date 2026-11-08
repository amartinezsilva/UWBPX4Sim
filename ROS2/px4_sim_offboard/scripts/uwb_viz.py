#!/usr/bin/env python3
"""RViz markers for any UWBPX4Sim experiment, built from its layout YAML.

Publishes on /uwb_viz/markers (visualization_msgs/MarkerArray):
  - each robot, drawn from the visuals of its generated Gazebo model
    (meshes, boxes, cylinders), or a plain box if none can be drawn
  - anchors as blue spheres, tags as red spheres, with their ids
  - one line per tag-anchor pair, bright while measurements arrive on it and
    faint while it drops out (TWR ranges, or TDoA differences using the leg)
  - each robot's recent path

Robots and transceivers follow the TF frames the offboard nodes publish.
Robots with `offboard: {enabled: false}` have no node, so their frames are
published here at their spawn pose (they don't move).

Run by offboard_launch.py with: layout_file, uwb_root, px4_dir.
"""

from __future__ import annotations

import importlib.util
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import ColorRGBA, Float64
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

ANCHOR_COLOR = (0.16, 0.47, 0.84)
TAG_COLOR = (0.89, 0.29, 0.28)
LINK_COLOR = (0.85, 0.85, 0.85)
ROBOT_COLOR = (0.62, 0.64, 0.68)
LINE_ON_ALPHA = 0.55
LINE_OFF_ALPHA = 0.08
FRESH_S = 0.5          # a link counts as live if measured within this time
TRAIL_PERIOD_S = 0.5
TRAIL_POINTS = 400
SENSOR_DIAMETER_M = 0.08


# --- small pose maths: (x, y, z, qx, qy, qz, qw) ------------------------------

def quat_from_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2),
                              math.sin(p / 2), math.cos(y / 2), math.sin(y / 2))
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def quat_rotate(q, v):
    x, y, z, _ = quat_mul(quat_mul(q, (*v, 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return (x, y, z)


def compose(a, b):
    """Pose b expressed in a's parent frame (a then b)."""
    t = quat_rotate(a[3:], b[:3])
    return (a[0] + t[0], a[1] + t[1], a[2] + t[2], *quat_mul(a[3:], b[3:]))


def inverse(a):
    q = (-a[3], -a[4], -a[5], a[6])
    t = quat_rotate(q, a[:3])
    return (-t[0], -t[1], -t[2], *q)


def parse_pose(element):
    if element is None or not (element.text or "").split():
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    v = [float(x) for x in element.text.split()]
    return (v[0], v[1], v[2], *quat_from_rpy(v[3], v[4], v[5]))


# --- robot visuals from the generated model.sdf -------------------------------

def resolve_uri(uri, model_dirs):
    if uri.startswith("model://"):
        rel = uri[len("model://"):]
        for d in model_dirs:
            path = Path(d) / rel
            if path.is_file():
                return "file://" + str(path.resolve())
        return None
    if uri.startswith("file://"):
        return uri if Path(uri[len("file://"):]).is_file() else None
    return None


def robot_visuals(model_sdf, model_dirs):
    """(kind, pose in base_link, scale or size, mesh uri) for each drawable visual."""
    if not model_sdf.is_file():
        return []
    model = ET.parse(model_sdf).getroot().find("model")
    if model is None:
        return []
    links = {link.get("name"): link for link in model.findall("link")}
    base = links.get("base_link")
    to_base = inverse(parse_pose(base.find("pose"))) if base is not None else parse_pose(None)
    out = []
    for name, link in links.items():
        if name.startswith("uwb_"):
            continue  # transceivers are drawn as spheres
        link_pose = compose(to_base, parse_pose(link.find("pose")))
        for visual in link.findall("visual"):
            pose = compose(link_pose, parse_pose(visual.find("pose")))
            geometry = visual.find("geometry")
            if geometry is None:
                continue
            mesh, box, cyl = geometry.find("mesh"), geometry.find("box"), geometry.find("cylinder")
            if mesh is not None and mesh.find("uri") is not None:
                uri = resolve_uri(mesh.find("uri").text.strip(), model_dirs)
                scale = [float(x) for x in (mesh.findtext("scale") or "1 1 1").split()]
                if uri:
                    out.append(("mesh", pose, scale, uri))
            elif box is not None:
                out.append(("box", pose, [float(x) for x in box.findtext("size").split()], None))
            elif cyl is not None:
                r, h = float(cyl.findtext("radius")), float(cyl.findtext("length"))
                out.append(("cylinder", pose, [2 * r, 2 * r, h], None))
    return out


class UwbViz(Node):
    def __init__(self):
        super().__init__("uwb_viz")
        layout_file = Path(self.declare_parameter("layout_file", "").value)
        uwb_root = Path(self.declare_parameter("uwb_root", "").value)
        px4_dir = Path(self.declare_parameter("px4_dir", str(Path.home() / "PX4-Autopilot")).value)

        spec = importlib.util.spec_from_file_location("uwb_layout_tool", uwb_root / "tools" / "configure_uwb_layout.py")
        tool = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = tool
        spec.loader.exec_module(tool)
        layout = tool.load_layout(layout_file)

        model_dirs = [uwb_root / "models" / "custom_models", px4_dir / tool.PX4_GZ_MODELS_SUBDIR]
        self.world = "map"
        self.robots = []      # (frame_prefix, visuals)
        self.sensors = []     # (frame, kind, id)
        static_tfs = []
        for role, vehicle in tool.iter_spawn_vehicles(layout):
            params = tool.resolve_offboard_params(vehicle.offboard, role=role, vehicle_id=vehicle.id)
            prefix = params["frame_prefix"]
            self.world = params["world_frame_id"]
            model_sdf = uwb_root / "models" / "custom_models" / vehicle.model.model_name(vehicle.id) / "model.sdf"
            self.robots.append((prefix, robot_visuals(model_sdf, model_dirs)))
            for kind, sensors in (("tag", vehicle.tags), ("anchor", vehicle.anchors)):
                for s in sensors:
                    self.sensors.append((f"{prefix}/{'t' if kind == 'tag' else 'a'}{s.id}", kind, s.id))
            if not tool.offboard_enabled(vehicle.offboard):
                # No offboard node publishes this robot's frames: it stays at its spawn pose.
                p = vehicle.spawn_pose
                static_tfs.append(self.transform(self.world, f"{prefix}/base_link",
                                                 (p.x, p.y, p.z, *quat_from_rpy(p.roll, p.pitch, p.yaw))))
                for kind, sensors in (("t", vehicle.tags), ("a", vehicle.anchors)):
                    for s in sensors:
                        static_tfs.append(self.transform(f"{prefix}/base_link", f"{prefix}/{kind}{s.id}",
                                                         (*s.position, 0.0, 0.0, 0.0, 1.0)))
        for s in layout.static_anchors:
            self.sensors.append((f"static_anchors/a{s.id}", "anchor", s.id))
        if static_tfs:
            self.static_broadcaster = StaticTransformBroadcaster(self)
            self.static_broadcaster.sendTransform(static_tfs)

        self.tags = sorted(s.id for s in layout.all_tags)
        self.anchors = sorted(s.id for s in layout.all_anchors)
        self.frame = {(k, i): f for f, k, i in self.sensors}
        self.last_rx = {}
        for t in self.tags:
            for a in self.anchors:
                self.create_subscription(Float64, f"/uwb_gz_simulator/distances/a{a}t{t}",
                                         lambda _m, a=a, t=t: self.mark(((a, t),)), 10)
            ref = self.anchors[0]
            for a in self.anchors[1:]:
                self.create_subscription(Float64, f"/uwb_gz_simulator/tdoa/a{ref}a{a}t{t}",
                                         lambda _m, a=a, t=t, r=ref: self.mark(((r, t), (a, t))), 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.trails = {prefix: [] for prefix, _ in self.robots}
        self.last_trail = None
        self.pub = self.create_publisher(MarkerArray, "/uwb_viz/markers", 10)
        self.static_markers = self.build_static_markers()
        self.create_timer(0.1, self.publish)
        self.get_logger().info(f"Visualising {len(self.robots)} robots, {len(self.tags)} tags, "
                               f"{len(self.anchors)} anchors in frame '{self.world}'.")

    def transform(self, parent, child, pose):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = parent, child
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = pose[:3]
        (tf.transform.rotation.x, tf.transform.rotation.y,
         tf.transform.rotation.z, tf.transform.rotation.w) = pose[3:]
        return tf

    def mark(self, legs):
        now = self.get_clock().now()
        for leg in legs:
            self.last_rx[leg] = now

    @staticmethod
    def marker(ns, mid, frame, mtype, color, alpha):
        m = Marker()
        m.header.frame_id = frame  # stamp 0: RViz uses the latest transform
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=alpha)
        return m

    def build_static_markers(self):
        markers = []
        for r, (prefix, visuals) in enumerate(self.robots):
            frame = f"{prefix}/base_link"
            if not visuals:
                m = self.marker(f"robot/{prefix}", 0, frame, Marker.CUBE, ROBOT_COLOR, 0.9)
                m.scale.x, m.scale.y, m.scale.z = 0.5, 0.5, 0.2
                markers.append(m)
            for i, (kind, pose, size, uri) in enumerate(visuals):
                mtype = {"mesh": Marker.MESH_RESOURCE, "box": Marker.CUBE, "cylinder": Marker.CYLINDER}[kind]
                m = self.marker(f"robot/{prefix}", i, frame, mtype, ROBOT_COLOR, 1.0)
                (m.pose.position.x, m.pose.position.y, m.pose.position.z) = pose[:3]
                (m.pose.orientation.x, m.pose.orientation.y,
                 m.pose.orientation.z, m.pose.orientation.w) = pose[3:]
                m.scale.x, m.scale.y, m.scale.z = size
                if uri:
                    m.mesh_resource = uri
                    m.mesh_use_embedded_materials = uri.lower().endswith(".dae")
                markers.append(m)
        for i, (frame, kind, sid) in enumerate(self.sensors):
            color = TAG_COLOR if kind == "tag" else ANCHOR_COLOR
            m = self.marker("sensors", i, frame, Marker.SPHERE, color, 1.0)
            m.scale.x = m.scale.y = m.scale.z = SENSOR_DIAMETER_M
            markers.append(m)
            label = self.marker("labels", i, frame, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0), 0.9)
            label.text = f"{'t' if kind == 'tag' else 'a'}{sid}"
            label.pose.position.z = 0.1
            label.scale.z = 0.08
            markers.append(label)
        return markers

    def position(self, frame):
        try:
            t = self.tf_buffer.lookup_transform(self.world, frame, Time()).transform.translation
            return Point(x=t.x, y=t.y, z=t.z)
        except Exception:  # noqa: BLE001 -- frame not published yet
            return None

    def publish(self):
        now = self.get_clock().now()
        out = MarkerArray(markers=list(self.static_markers))

        links = self.marker("links", 0, self.world, Marker.LINE_LIST, LINK_COLOR, 1.0)
        links.scale.x = 0.01
        for t in self.tags:
            pt = self.position(self.frame.get(("tag", t), ""))
            for a in self.anchors:
                pa = self.position(self.frame.get(("anchor", a), ""))
                if pt is None or pa is None:
                    continue
                last = self.last_rx.get((a, t))
                live = last is not None and (now - last).nanoseconds * 1e-9 < FRESH_S
                c = ColorRGBA(r=LINK_COLOR[0], g=LINK_COLOR[1], b=LINK_COLOR[2],
                              a=LINE_ON_ALPHA if live else LINE_OFF_ALPHA)
                links.points += [pt, pa]
                links.colors += [c, c]
        out.markers.append(links)

        if self.last_trail is None or (now - self.last_trail).nanoseconds * 1e-9 >= TRAIL_PERIOD_S:
            self.last_trail = now
            for prefix in self.trails:
                p = self.position(f"{prefix}/base_link")
                if p is not None:
                    self.trails[prefix] = (self.trails[prefix] + [p])[-TRAIL_POINTS:]
        for i, (prefix, points) in enumerate(self.trails.items()):
            if len(points) > 1:
                trail = self.marker("paths", i, self.world, Marker.LINE_STRIP, (1.0, 0.85, 0.3), 0.6)
                trail.scale.x = 0.02
                trail.points = points
                out.markers.append(trail)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = UwbViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
