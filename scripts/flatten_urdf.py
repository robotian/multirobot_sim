#!/usr/bin/env python3
"""Make a Clearpath-generated URDF self-contained for the Isaac Sim URDF importer.

Copies every mesh referenced through package:// or file:// into <out>/meshes/<pkg>/... and rewrites the
URDF to use relative paths, so the Isaac container does not need any ROS packages installed.
Also drops <gazebo>/<ros2_control> blocks, which the importer does not use.
"""
import math
import os
import re
import shutil
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


# Vertex-clustering cell for meshes converted below. d435.dae has 231k triangles for a 90 mm sensor; with three
# robots rendered by a path tracer that alone cost ~30% of the frame rate.
DECIMATE_CELL = 0.001  # m


def dae_to_obj(text, dst):
    """Merge every <geometry> of a Collada file into one Wavefront OBJ (+ .mtl with the diffuse colour).

    The importer silently drops the RealSense d435.dae (it ends up as an empty Xform in the USD, so the camera
    is invisible in the sim) although it accepts Blender-exported Collada. Its geometries carry no node
    transforms, so a plain merge is enough. The mesh is decimated by vertex clustering (DECIMATE_CELL) and
    per-vertex normals are averaged per cluster when the file provides them.
    """
    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    root = ET.fromstring(text)
    diffuse = root.findtext(".//c:diffuse/c:color", default="0.75 0.75 0.75 1", namespaces=ns).split()[:3]
    name = os.path.splitext(os.path.basename(dst))[0]
    clusters = {}  # grid cell -> index; sums of position / normal and member count
    verts, norms, counts = [], [], []
    faces = set()
    has_normals = False
    for mesh in root.iterfind(".//c:geometry/c:mesh", ns):
        sources = {s.get("id"): [float(v) for v in s.findtext("c:float_array", namespaces=ns).split()]
                   for s in mesh.iterfind("c:source", ns)}
        vertices = mesh.find("c:vertices", ns)
        pos_id = next(i.get("source")[1:] for i in vertices.iterfind("c:input", ns) if i.get("semantic") == "POSITION")
        pos = sources[pos_id]
        # a NORMAL source aligned 1:1 with the positions (as in d435.dae)
        normals = next((v for k, v in sources.items() if k != pos_id and len(v) == len(pos)), None)
        has_normals = has_normals or normals is not None
        remap = []
        for i in range(len(pos) // 3):
            p = pos[i * 3:i * 3 + 3]
            key = tuple(round(c / DECIMATE_CELL) for c in p)
            if key not in clusters:
                clusters[key] = len(verts)
                verts.append([0.0, 0.0, 0.0])
                norms.append([0.0, 0.0, 0.0])
                counts.append(0)
            k = clusters[key]
            counts[k] += 1
            for axis in range(3):
                verts[k][axis] += p[axis]
                if normals:
                    norms[k][axis] += normals[i * 3 + axis]
            remap.append(k)
        # <triangles> (uniform 3-vertex faces) and <polylist> (per-face vertex count in <vcount>, fan-
        # triangulated here) are both handled the same way once split into per-face vertex-index groups --
        # velodyne_description's own VLP16 meshes use <polylist> (confirmed: every <vcount> entry is 3, i.e.
        # already all-triangle, but fan-triangulating is correct even if a future mesh has real n-gons).
        for tag in ("c:triangles", "c:polylist"):
            for prim in mesh.iterfind(tag, ns):
                inputs = list(prim.iterfind("c:input", ns))
                stride = 1 + max(int(i.get("offset")) for i in inputs)
                vert_offset = next(int(i.get("offset")) for i in inputs if i.get("semantic") == "VERTEX")
                # Slice out just the VERTEX-offset values *before* remapping: the interleaved <p> list also
                # carries e.g. NORMAL indices, which can run over a larger range than the position/remap array
                # (found on top_assy_rev1.dae -- d435.dae happened to index normals 1:1 with positions, masking
                # this).
                raw = [int(v) for v in prim.findtext("c:p", namespaces=ns).split()]
                idx = [remap[v] for v in raw[vert_offset::stride]]
                if tag == "c:triangles":
                    polys = [idx[i:i + 3] for i in range(0, len(idx), 3)]
                else:
                    vcounts = [int(v) for v in prim.findtext("c:vcount", namespaces=ns).split()]
                    polys, off = [], 0
                    for n in vcounts:
                        polys.append(idx[off:off + n])
                        off += n
                for poly in polys:
                    for a, b, c in zip([poly[0]] * (len(poly) - 2), poly[1:-1], poly[2:]):  # fan triangulation
                        if a != b and b != c and a != c:
                            faces.add((a, b, c))  # keeps winding, drops exact duplicates
    lines = [f"mtllib {name}.mtl", "usemtl body"]
    lines += ["v %g %g %g" % tuple(c / n for c in v) for v, n in zip(verts, counts)]
    if has_normals:
        for nv in norms:
            length = sum(c * c for c in nv) ** 0.5 or 1.0
            lines.append("vn %g %g %g" % tuple(c / length for c in nv))
    for a, b, c in sorted(faces):
        lines.append("f " + " ".join(f"{v + 1}//{v + 1}" if has_normals else f"{v + 1}" for v in (a, b, c)))
    with open(dst, "w") as f:
        f.write("\n".join(lines) + "\n")
    with open(dst[:-4] + ".mtl", "w") as f:
        f.write("newmtl body\nKd %s\nKa 0 0 0\nKs 0.1 0.1 0.1\nNs 20\n" % " ".join(diffuse))
    print(f"{os.path.basename(dst)}: {len(faces)} triangles after decimation")


def copy_mesh(src, dst):
    """Copy a mesh; returns the destination path (a .dae may become an .obj).

    Collada materials without a `name` (e.g. realsense d435.dae) crash the importer, and hand-exported
    (non-Blender) Collada files are dropped by it, so those are converted to OBJ.
    """
    if not src.lower().endswith(".dae"):
        shutil.copy2(src, dst)
        return dst
    text = open(src, encoding="utf-8").read()
    # velodyne_description's own shipped VLP16 meshes (base_1/base_2/scan) use the literal, unescaped string
    # "<STL_BINARY>" as a node id/name -- invalid XML (a bare "<"/">" inside an attribute value), not this
    # project's own asset. A real upstream authoring bug, confirmed identical across all three files; only this
    # exact placeholder token is broken (checked directly, not assumed), so a narrow substitution is enough.
    text = text.replace("<STL_BINARY>", "STL_BINARY")
    if "Blender" not in text[:2000]:
        dst = dst[:-4] + ".obj"
        dae_to_obj(text, dst)
        return dst
    text = re.sub(r'<material id="([^"]+)"(\s*)>', r'<material id="\1" name="\1"\2>', text)
    with open(dst, "w", encoding="utf-8") as f:
        f.write(text)
    return dst


def _origin_matrix(origin):
    """A <origin xyz="x y z" rpy="r p y"/> element (or None) as a 4x4 row-major transform matrix, using
    URDF's own convention: R = Rz(yaw) * Ry(pitch) * Rx(roll), translation applied after rotation."""
    x, y, z = (float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split())
    r, p, ya = (float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split())
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(ya), math.sin(ya)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y],
        [-sp, cp * sr, cp * cr, z],
        [0, 0, 0, 1],
    ]


def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _matrix_to_origin(m):
    """Inverse of _origin_matrix: a 4x4 matrix back to an xyz/rpy string pair, for writing into the URDF."""
    pitch = math.atan2(-m[2][0], math.hypot(m[0][0], m[1][0]))
    if abs(math.cos(pitch)) > 1e-6:
        yaw = math.atan2(m[1][0], m[0][0])
        roll = math.atan2(m[2][1], m[2][2])
    else:  # gimbal lock: roll and yaw trade off; pin yaw to 0
        yaw = 0.0
        roll = math.atan2(-m[1][2], m[1][1])
    return f"{m[0][3]:.9g} {m[1][3]:.9g} {m[2][3]:.9g}", f"{roll:.9g} {pitch:.9g} {yaw:.9g}"


def prune_dangling_joints(root):
    """Drop any joint (and its whole child subtree) whose <parent> link was never actually defined.

    Found on j100_0922: mtu32_description's own robot_description_j100.urdf.xacro unconditionally mounts a
    second camera (camera_1, a RealSense D405) on arm_0_end_effector_link, assuming every Jackal running this
    xacro has the Kinova arm -- but this real robot's own robot.yaml has its whole manipulators.arms section
    commented out (no arm at all), so clearpath_config never defines that link anywhere. xacro is a pure
    text/macro processor with no semantic validation, so a dangling <parent> reference like this reaches the
    flattened URDF unnoticed -- Isaac's importer would choke on it (the same class of "joint references an
    undefined link" bug this project has hit before with a stripped-template's top_mount_link, just from the
    opposite direction: a missing *parent* here, a missing *child* there). Not specific to this one robot/link
    -- a general defensive pass, in case a future real robot.yaml hits the same kind of upstream assumption
    elsewhere.
    """
    changed = True
    while changed:
        changed = False
        link_names = {link.get("name") for link in root.findall("link")}
        for joint in root.findall("joint"):
            parent_name = joint.find("parent").get("link")
            if parent_name in link_names:
                continue
            child_name = joint.find("child").get("link")
            child = next((link for link in root.findall("link") if link.get("name") == child_name), None)
            print(f"dropping joint {joint.get('name')!r}: parent {parent_name!r} is never defined as a link")
            root.remove(joint)
            if child is not None:
                root.remove(child)
            changed = True  # removing this link may orphan another joint that used it as *its* parent
            break


def merge_visual_only_links(root):
    """Fold any link that is *only* a <visual> (no <collision>, no <inertial> -- e.g. Jackal's fender links,
    which come from clearpath_platform_description's fender.urdf.xacro exactly this way) into its parent link,
    dropping the now-empty link and its connecting (always <fixed>, checked below) joint.

    Isaac Sim's URDF importer still makes such a link a genuinely separate simulated body even with zero mass,
    connected to its parent by a PhysX fixed-joint constraint. That constraint isn't perfectly rigid for a
    body this light relative to the rest of the robot: it visibly drifts away from the parent once the robot
    is driven or turned (confirmed live on Jackal's fenders -- static at spawn they looked attached, but
    lagged behind and detached after driving). Giving the link a small synthetic mass/inertia instead (tried
    first) didn't fix this -- the constraint is still there, just less obviously wrong. Merging the geometry
    directly into the parent link's own <visual> list removes the joint entirely, so there is nothing left
    to drift: the mesh is now literally part of the parent body.
    """
    joints = root.findall("joint")
    changed = True
    while changed:
        changed = False
        links_by_name = {link.get("name"): link for link in root.findall("link")}
        for joint in list(joints):
            child_name = joint.find("child").get("link")
            child = links_by_name.get(child_name)
            if child is None or joint.get("type") != "fixed":
                continue
            if child.find("collision") is not None or child.find("inertial") is not None:
                continue
            visuals = child.findall("visual")
            if not visuals:
                continue
            parent_name = joint.find("parent").get("link")
            parent = links_by_name.get(parent_name)
            if parent is None:
                continue
            joint_m = _origin_matrix(joint.find("origin"))
            for visual in visuals:
                combined = _matmul(joint_m, _origin_matrix(visual.find("origin")))
                old_origin = visual.find("origin")
                if old_origin is not None:
                    visual.remove(old_origin)
                xyz, rpy = _matrix_to_origin(combined)
                visual.insert(0, ET.Element("origin", {"xyz": xyz, "rpy": rpy}))
                parent.append(visual)
            # Any other joint that used this (now-removed) link as its own parent must be re-pointed at this
            # link's parent instead, and its origin re-expressed in that link's frame (composed with joint_m) --
            # otherwise it's left referencing a link name that no longer exists (a genuine URDF import error,
            # not just a cosmetic one), or ends up in the wrong place if only the name were rewritten.
            for other in joints:
                if other is not joint and other.find("parent").get("link") == child_name:
                    other.find("parent").set("link", parent_name)
                    other_m = _matmul(joint_m, _origin_matrix(other.find("origin")))
                    old = other.find("origin")
                    if old is not None:
                        other.remove(old)
                    xyz, rpy = _matrix_to_origin(other_m)
                    other.insert(0, ET.Element("origin", {"xyz": xyz, "rpy": rpy}))
            root.remove(child)
            joints.remove(joint)
            root.remove(joint)
            changed = True  # a link merged away might itself have been the parent of another such joint


def apply_mass_deltas(root, spec):
    """Add a delta mass to one or more links' existing <inertial><mass> value.

    `spec` is "link_name:delta_kg" entries separated by ";" (e.g. "chassis_link:10"). A deliberate what-if mass
    change for experimentation (e.g. "how does the sim behave with a 10kg heavier chassis"), not a general
    modeling feature -- callers wire it up per-model in scripts/gen_urdf.sh, not here. The inertia tensor is
    left untouched: recomputing it correctly would need to know the added mass's own shape/distribution, which
    this doesn't model -- a disclosed simplification, not a physically exact reballast.
    """
    for entry in spec.split(";"):
        if not entry:
            continue
        link_name, delta_str = entry.split(":")
        delta = float(delta_str)
        for link in root.findall("link"):
            if link.get("name") != link_name:
                continue
            mass_el = link.find("inertial/mass")
            if mass_el is None:
                raise ValueError(f"{link_name!r} has no <inertial><mass> to adjust")
            old = float(mass_el.get("value"))
            mass_el.set("value", str(old + delta))
            print(f"{link_name}: mass {old:g} -> {old + delta:g} kg (delta {delta:+g}, inertia tensor unchanged)")
            break
        else:
            raise ValueError(f"no link named {link_name!r} found")


def main(urdf_in, out_dir, urdf_name, mass_overrides=""):
    tree = ET.parse(urdf_in)
    root = tree.getroot()

    # The generated name is the serial (e.g. "a200-0000"); USD would mangle it, so use a plain identifier
    # instead -- urdf_name is "<model>.urdf", so this also keeps each model's own name in its own USD.
    root.set("name", urdf_name.removesuffix(".urdf"))

    for tag in ("gazebo", "ros2_control"):
        for el in list(root.findall(tag)):
            root.remove(el)

    if mass_overrides:
        apply_mass_deltas(root, mass_overrides)

    prune_dangling_joints(root)
    merge_visual_only_links(root)

    copied = {}
    for mesh in root.iter("mesh"):
        uri = mesh.get("filename")
        if uri.startswith("package://"):
            pkg, rel = uri[len("package://"):].split("/", 1)
            src = os.path.join(get_package_share_directory(pkg), rel)
        elif uri.startswith("file://"):
            src = uri[len("file://"):]
            pkg = os.path.basename(src.split("/share/")[1].split("/")[0]) if "/share/" in src else "misc"
            rel = src.split("/share/" + pkg + "/", 1)[1] if "/share/" in src else os.path.basename(src)
        else:
            continue
        dst = os.path.join(out_dir, "meshes", pkg, rel)
        if src not in copied:
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            copied[src] = os.path.relpath(copy_mesh(src, dst), out_dir)
        mesh.set("filename", copied[src])

    os.makedirs(out_dir, exist_ok=True)
    tree.write(os.path.join(out_dir, urdf_name), xml_declaration=True, encoding="utf-8")
    print(f"wrote {os.path.join(out_dir, urdf_name)} with {len(copied)} meshes")


if __name__ == "__main__":
    main(*sys.argv[1:5])
