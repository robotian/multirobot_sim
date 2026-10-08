"""Write a scene USD that is an existing farm scene plus its charging stations (public.charging_stations).

Run by scripts/make_charger_scene.py inside the Isaac Sim image with only Isaac's USD libraries on the path (no Kit),
with ./sim mounted at /sim:  add_chargers.py <chargers.json> <farm.usd> <out.usda>
chargers.json: {"source": ..., "chargers": [{"id", "charger_model", "x", "y", "yaw_deg", "apriltag_id",
"apriltag_sz_mm", "frame"}, ...]}.

The output is a small layer with the farm file as its sublayer (the farm stays a generated file of its own:
scripts/make_farm_scene.py rebuilds it without losing the chargers), so it is used like the farm itself
(SIM_SCENE=<out>, build_file_world in setup_scene.py). Per charger it adds /World/charging_stations/charger_<id>:
an Xform at the pose, referencing the model's asset with its AprilTag variant set to the row's apriltag_id, and
the tag scaled to apriltag_sz_mm if that differs from the asset's. Border vegetation (rocks, shrubs, trees) within
CLEAR_RADIUS of a charger is deactivated so it doesn't sit in the charger or its approach.

Pose convention (as in the table's column comments): x/y is the charger's footprint centre on the ground, yaw is
the direction its front, the side with the AprilTag, faces, CCW from the frame's +X. The map frame is the sim's
world frame (X/Y = East/North, see build_farm_scene.py).
"""

import json
import math
import os
import sys

from pxr import Gf, Sdf, Usd, UsdGeom

ASSETS = "/sim/assets"
REL_ASSETS = "../assets"  # as seen from sim/scene/<file>

# charger_model -> its asset. front_deg: the direction the asset's front (tag side) faces at no rotation, CCW from
# +X; tag_prim: the tag's Xform below the asset's default prim; tag_mm: the tag's black-square edge in the asset
# (the tag texture is cropped to the black square, the edge apriltag_ros calls its size).
CHARGER_MODELS = {
    "TR-302": {  # WiBotic TR-302 Edge: 0.37 x 0.21 x 0.44 m, origin at the footprint centre, tag on the -Y face
        "asset": "wibotic_tr302_edge/wibotic_tr302_edge.usd",
        "front_deg": -90.0,
        "tag_variant_set": "AprilTag_ID",
        "tag_variant": "id_{:03d}",
        "tag_prim": "TR302_Edge/AprilTag",
        "tag_mm": 80,
    },
}

# frame -> its pose in the sim's world frame (x, y, yaw_deg). Only static frames make sense here.
FRAMES = {"map": (0.0, 0.0, 0.0)}

CLEAR_RADIUS = 3.0  # m from a charger, border items closer than this (their origin) are deactivated
BORDER_GROUPS = ("rocks", "shrubs", "trees")


def to_world(c):
    if c["frame"] not in FRAMES:
        raise ValueError(f"charger {c['id']}: frame {c['frame']!r} is not one of {sorted(FRAMES)}")
    fx, fy, fyaw = FRAMES[c["frame"]]
    a = math.radians(fyaw)
    return (fx + c["x"] * math.cos(a) - c["y"] * math.sin(a),
            fy + c["x"] * math.sin(a) + c["y"] * math.cos(a),
            fyaw + c["yaw_deg"])


def add_charger(stage, c, x, y, yaw):
    m = CHARGER_MODELS[c["charger_model"]]
    root = stage.DefinePrim(f"/World/charging_stations/charger_{c['id']}", "Xform")
    root.SetCustomDataByKey("charging_station_id", int(c["id"]))
    root.SetCustomDataByKey("apriltag_id", int(c["apriltag_id"]))
    xf = UsdGeom.Xformable(root)
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
    xf.AddRotateZOp().Set(yaw - m["front_deg"])

    # the asset's root may carry its own xform ops, so it goes on a child of the placement Xform
    asset = stage.DefinePrim(f"{root.GetPath()}/asset", "Xform")
    asset.GetReferences().AddReference(f"{REL_ASSETS}/{m['asset']}")
    vset = asset.GetVariantSets().GetVariantSet(m["tag_variant_set"])
    variant = m["tag_variant"].format(c["apriltag_id"])
    if variant not in vset.GetVariantNames():
        raise ValueError(f"charger {c['id']}: {c['charger_model']} has no AprilTag variant {variant}")
    vset.SetVariantSelection(variant)

    size = c.get("apriltag_sz_mm")
    if size and size != m["tag_mm"]:
        # scale the tag in its own plane about its centre (its normal axis is the one of zero extent)
        tag = stage.GetPrimAtPath(f"{asset.GetPath()}/{m['tag_prim']}")
        r = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeLocalBound(tag).ComputeAlignedRange()
        k = size / m["tag_mm"]
        scale = Gf.Vec3d(*(k if r.GetSize()[i] > 1e-6 else 1.0 for i in range(3)))
        mid = r.GetMidpoint()
        txf = UsdGeom.Xformable(tag)
        txf.AddTranslateOp(opSuffix="tag_pivot").Set(Gf.Vec3d(mid))
        txf.AddScaleOp(opSuffix="tag_size").Set(scale)
        txf.AddTranslateOp(opSuffix="tag_pivot_inv").Set(-Gf.Vec3d(mid))
        print(f"charger {c['id']}: tag scaled {m['tag_mm']} -> {size} mm")


def clear_border(stage, spots):
    """Deactivate the farm's border items within CLEAR_RADIUS of any (x, y) in spots; returns their paths."""
    cleared = []
    for g in BORDER_GROUPS:
        group = stage.GetPrimAtPath(f"/World/{g}")
        for p in (group.GetChildren() if group else []):
            t = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
            if any(math.hypot(t[0] - x, t[1] - y) < CLEAR_RADIUS for x, y in spots):
                stage.OverridePrim(p.GetPath()).SetActive(False)
                cleared.append(str(p.GetPath()))
    return cleared


def main(chargers_json, farm, out):
    with open(chargers_json) as f:
        data = json.load(f)
    chargers = data["chargers"]
    for c in chargers:
        if c["charger_model"] not in CHARGER_MODELS:
            sys.exit(f"charger {c['id']}: no asset for charger_model {c['charger_model']!r} "
                     f"(known: {', '.join(CHARGER_MODELS)}); add it to CHARGER_MODELS in {__file__}")

    farm_layer = Sdf.Layer.FindOrOpen(farm)
    if farm_layer is None:
        sys.exit(f"could not open {farm}")
    if os.path.exists(out):
        os.remove(out)
    layer = Sdf.Layer.CreateNew(out)
    layer.subLayerPaths.append("./" + os.path.relpath(farm, os.path.dirname(out)))
    stage = Usd.Stage.Open(layer)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
    UsdGeom.Xform.Define(stage, "/World/charging_stations")

    spots, placed = [], []
    for c in sorted(chargers, key=lambda c: c["id"]):
        x, y, yaw = to_world(c)
        add_charger(stage, c, x, y, yaw)
        spots.append((x, y))
        placed.append([c["id"], x, y, yaw, c["apriltag_id"]])
        print(f"charger {c['id']} ({c['charger_model']}): world ({x:.3f}, {y:.3f}) yaw {yaw:.1f} deg, "
              f"AprilTag tag36h11 id {c['apriltag_id']}")
    cleared = clear_border(stage, spots)
    if cleared:
        print(f"deactivated {len(cleared)} border items within {CLEAR_RADIUS} m: {', '.join(cleared)}")

    # build_file_world reads only this layer's customLayerData: keep the farm's (the web UI's spawn map rows)
    meta = dict(farm_layer.customLayerData)
    meta.update({
        "charger_source": data.get("source", ""),
        "farm_file": os.path.basename(farm),
        # per charger, in the world frame
        "charging_stations": {f"charger_{i}": {"x": float(x), "y": float(y), "yaw_deg": float(yaw),
                                               "apriltag_id": int(tag)} for i, x, y, yaw, tag in placed},
    })
    layer.customLayerData = meta
    layer.documentation = (f"{os.path.basename(farm)} plus its charging stations, written by "
                           "scripts/make_charger_scene.py from public.charging_stations; don't edit, rerun it.")
    layer.Save()
    os.chmod(out, 0o666)  # written as the image's user; the host user may replace it
    print(f"wrote {out}: {len(placed)} chargers on {os.path.basename(farm)}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3])
