#!/usr/bin/env python3
"""Make a Clearpath-generated URDF self-contained for the Isaac Sim URDF importer.

Copies every mesh referenced through package:// or file:// into <out>/meshes/<pkg>/... and rewrites the
URDF to use relative paths, so the Isaac container does not need any ROS packages installed.
Also drops <gazebo>/<ros2_control> blocks, which the importer does not use.
"""
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
        for tri in mesh.iterfind("c:triangles", ns):
            stride = 1 + max(int(i.get("offset")) for i in tri.iterfind("c:input", ns))
            idx = [remap[int(v)] for v in tri.findtext("c:p", namespaces=ns).split()][::stride]
            for a, b, c in zip(idx[0::3], idx[1::3], idx[2::3]):
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
    if "Blender" not in text[:2000]:
        dst = dst[:-4] + ".obj"
        dae_to_obj(text, dst)
        return dst
    text = re.sub(r'<material id="([^"]+)"(\s*)>', r'<material id="\1" name="\1"\2>', text)
    with open(dst, "w", encoding="utf-8") as f:
        f.write(text)
    return dst


def main(urdf_in, out_dir, urdf_name):
    tree = ET.parse(urdf_in)
    root = tree.getroot()

    # The generated name is the serial ("a300-0000"); USD would mangle it, so use a plain identifier.
    root.set("name", "a300")

    for tag in ("gazebo", "ros2_control"):
        for el in list(root.findall(tag)):
            root.remove(el)

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
    main(*sys.argv[1:4])
