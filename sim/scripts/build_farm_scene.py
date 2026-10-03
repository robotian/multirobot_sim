"""Write a scene USD of the real lavender field: one plant per object_data row at its map x/y.

Run by scripts/make_farm_scene.py inside the Isaac Sim image with only Isaac's USD libraries on the path (no Kit),
with ./sim mounted at /sim:  build_farm_scene.py <plants.json> <out.usd>
plants.json: {"source": ..., "plants": [{"object_id", "row_id", "x", "y"}, ...]} in the map frame, which is the
sim's world frame (the sim's GPS datum is its world origin, X/Y = East/North; sim_swift_nav_dual.launch.py).

The file is meant for SIM_SCENE=<file> (build_file_world in setup_scene.py: used as a sublayer, z=0 ground, the
fleet's physics settings authored over its physics scene), and also opens on its own in Isaac Sim. It only refers
to ../assets/... (relative to sim/scene/), nothing in sim/generated/: the colliders the lidars need are authored
in the file itself, on one `class` prototype per asset that the placed copies reference (the same meshes
setup_scene.collider_asset() would give colliders). The look follows the built-in `lavender` scene: soil-coloured
ground box, the ground-cover grass patch, HDR sky + sun, and a tree/shrub/rock border, here all around the field.
"""

import json
import math
import os
import random
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

ASSETS = "/sim/assets"
REL_ASSETS = "../assets"  # as seen from sim/scene/<file>

LAVENDER = "lavender/SM_Lavender_Nanite_01.usd"
# Plant size. The map (status_server) treats a row as 0.45 m thick and the plants are ~0.44 m apart in a row, so
# a 0.5 m clump just touches its neighbours into a hedge and leaves the mapped lane width free. The asset is a
# ~2 m wide, 1.34 m tall clump in cm, scaled uniformly (its own proportions; a taller-than-wide scale looked
# squeezed) to this width, so it ends up ~0.33 m tall.
PLANT_DIAMETER = 0.5
PLANT_SINK = 0.02  # lowest point below z=0, so no plant floats

GROUND_COVER = "Ground_cover/ground_cover.usd"  # 100 x 100 m of grass blades, geometry really in metres
GROUND_COVER_SCALE_Z = 0.6
GROUND_SIZE = 100.0
GROUND_SOIL_COLOR = (0.16, 0.10, 0.06)
SKY_HDR = "sky/farm_field_puresky_2k.hdr"
SKY_INTENSITY = 400
SUN_INTENSITY = 10000

# Border vegetation (setup_scene.py's add_horizon_vegetation, here a band around the field instead of an arc ahead
# of the robots): (assets, spacing along the band m, distance band outside the field's plants m, height m, seed).
# The rocks start 7 m out: the fleet's default spawn poses (x=0, y up to +1.6) are ~5.5 m north of row 1.
BORDER = {
    "trees": ([f"trees/{n}.usd" for n in ("Douglas_Fir", "Black_Oak", "Douglas_Fir")], 3.0, (13.0, 17.0), (7.0, 11.0), 7),
    "shrubs": ([f"shrubs/{n}.usd" for n in ("Rhododendron", "Lilac", "Goldflame_Spirea", "Barberry")], 1.8, (9.0, 14.0), (1.0, 2.2), 11),
    "rocks": ([f"rocks/rock_small_{i:02d}.usda" for i in range(1, 7)], 4.5, (7.0, 11.0), (0.4, 1.0), 13),
}

_bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])


def asset_bounds(asset):
    st = Usd.Stage.Open(f"{ASSETS}/{asset}")
    return _bbox.ComputeWorldBound(st.GetDefaultPrim() or st.GetPseudoRoot()).ComputeAlignedRange()


def add_prototype(stage, name, asset):
    """class /World/Prototypes/<name> referencing the asset, with a static exact triangle-mesh collider on each of
    its meshes (setup_scene.collider_asset, in this file). PointInstancer prototypes (leaves) can't be colliders."""
    path = f"/World/Prototypes/{name}"
    prim = stage.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(f"{REL_ASSETS}/{asset}")
    # opinions below an instanceable prim are ignored (the rocks have one)
    while True:
        inst = [q for q in Usd.PrimRange(prim, Usd.PrimAllPrimsPredicate) if q.IsInstanceable()]
        if not inst:
            break
        for q in inst:
            q.SetInstanceable(False)
    it = iter(Usd.PrimRange(prim, Usd.PrimAllPrimsPredicate))
    for q in it:
        if q.IsA(UsdGeom.PointInstancer):
            it.PruneChildren()
        elif q.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(q)
            UsdPhysics.MeshCollisionAPI.Apply(q).CreateApproximationAttr(UsdPhysics.Tokens.none)
    return path


def add_ground(stage, center):
    """GROUND_SIZE box centred on the field, top face at z=0, grippy physics material, matte soil colour."""
    mat = UsdShade.Material.Define(stage, "/World/Materials/ground_physics")
    pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr(1.0)
    pm.CreateDynamicFrictionAttr(0.9)
    pm.CreateRestitutionAttr(0.0)
    cube = UsdGeom.Cube.Define(stage, "/World/ground")
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(center[0], center[1], -0.5))
    xf.AddScaleOp().Set(Gf.Vec3d(GROUND_SIZE, GROUND_SIZE, 1.0))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*GROUND_SOIL_COLOR)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")
    surface = UsdShade.Material.Define(stage, "/World/Materials/ground_soil")
    shader = UsdShade.Shader.Define(stage, "/World/Materials/ground_soil/shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*GROUND_SOIL_COLOR))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
    surface.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(surface)

    # the grass patch (visual only, traction comes from the box), centred on the field like the box
    gc = stage.DefinePrim("/World/GroundCover", "Xform")
    gc.GetReferences().AddReference(f"{REL_ASSETS}/{GROUND_COVER}")
    gxf = UsdGeom.Xformable(gc)
    gxf.AddTranslateOp().Set(Gf.Vec3d(center[0], center[1], 0.0))
    gxf.AddScaleOp().Set(Gf.Vec3d(1.0, 1.0, GROUND_COVER_SCALE_Z))


def add_lights(stage):
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/dome")
    dome.CreateIntensityAttr(SKY_INTENSITY)
    dome.CreateTextureFileAttr(f"{REL_ASSETS}/{SKY_HDR}")
    dome.CreateTextureFormatAttr("latlong")
    sun = UsdLux.DistantLight.Define(stage, "/World/Lights/sun")
    sun.CreateIntensityAttr(SUN_INTENSITY)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-60, 33, -30))


def add_plants(stage, plants):
    """One instanceable copy of the lavender prototype per plant, under /World/lavender/row_<row_id>, scaled per axis
    uniformly to PLANT_DIAMETER wide and centred on its x/y (the asset's bbox is off its origin). Returns the rows'
    extents [x_min, x_max, y_mean, width] for the web UI's spawn map."""
    proto = add_prototype(stage, "lavender", LAVENDER)
    r = asset_bounds(LAVENDER)
    size, mid = r.GetSize(), r.GetMidpoint()
    k = PLANT_DIAMETER / max(size[0], size[1])
    pivot = Gf.Vec3d(-mid[0], -mid[1], -r.GetMin()[2])
    UsdGeom.Xform.Define(stage, "/World/lavender")
    rows = {}
    for p in sorted(plants, key=lambda p: (p["row_id"], p["object_id"])):
        row = f"/World/lavender/row_{p['row_id']}"
        if p["row_id"] not in rows:
            UsdGeom.Xform.Define(stage, row)
            rows[p["row_id"]] = []
        rows[p["row_id"]].append(p)
        prim = stage.DefinePrim(f"{row}/plant_{p['object_id']}", "Xform")
        prim.GetReferences().AddInternalReference(proto)
        prim.SetInstanceable(True)
        prim.SetCustomDataByKey("object_id", int(p["object_id"]))
        xf = UsdGeom.Xformable(prim)
        xf.AddTranslateOp().Set(Gf.Vec3d(p["x"], p["y"], -PLANT_SINK))
        xf.AddRotateZOp().Set(random.Random(p["object_id"]).uniform(0.0, 360.0))  # same look on every rebuild
        xf.AddScaleOp().Set(Gf.Vec3d(k, k, k))
        xf.AddTranslateOp(opSuffix="pivot").Set(pivot)
    half = PLANT_DIAMETER / 2
    return [[min(q["x"] for q in ps) - half, max(q["x"] for q in ps) + half, sum(q["y"] for q in ps) / len(ps),
             PLANT_DIAMETER]
            for _, ps in sorted(rows.items())]


def band_points(box, dist, spacing, rng):
    """Points spaced ~`spacing` apart along a rounded rectangle `dist` m outside box (x0, y0, x1, y1), jittered
    across the band (dist = (min, max))."""
    x0, y0, x1, y1 = box
    d_mid = sum(dist) / 2
    w, h = x1 - x0, y1 - y0
    arc = math.pi / 2 * d_mid
    # perimeter pieces, counter-clockwise from the south-east corner: (length, point(t in 0..1, d))
    pieces = [
        (h, lambda t, d: (x1 + d, y0 + t * h)),
        (arc, lambda t, d: (x1 + d * math.cos(t * math.pi / 2), y1 + d * math.sin(t * math.pi / 2))),
        (w, lambda t, d: (x1 - t * w, y1 + d)),
        (arc, lambda t, d: (x0 - d * math.sin(t * math.pi / 2), y1 + d * math.cos(t * math.pi / 2))),
        (h, lambda t, d: (x0 - d, y1 - t * h)),
        (arc, lambda t, d: (x0 - d * math.cos(t * math.pi / 2), y0 - d * math.sin(t * math.pi / 2))),
        (w, lambda t, d: (x0 + t * w, y0 - d)),
        (arc, lambda t, d: (x1 + d * math.sin(t * math.pi / 2), y0 - d * math.cos(t * math.pi / 2))),
    ]
    total = sum(length for length, _ in pieces)
    n = max(1, round(total / spacing))
    out = []
    for i in range(n):
        s = (i + rng.uniform(-0.3, 0.3)) * total / n % total
        for length, f in pieces:
            if s <= length:
                out.append(f(s / length if length else 0.0, rng.uniform(*dist)))
                break
            s -= length
    return out


def add_border(stage, box):
    for group, (assets, spacing, dist, height, seed) in BORDER.items():
        rng = random.Random(seed)
        protos = {}
        for a in dict.fromkeys(assets):
            name = f"{group}_{os.path.splitext(os.path.basename(a))[0]}"
            protos[a] = (add_prototype(stage, name, a), asset_bounds(a).GetSize()[2])
        UsdGeom.Xform.Define(stage, f"/World/{group}")
        for t, (x, y) in enumerate(band_points(box, dist, spacing, rng)):
            proto, z_size = protos[assets[rng.randrange(len(assets))]]
            k = rng.uniform(*height) / max(z_size, 1e-6)
            # the asset's root carries its own xform ops, so it goes on a child of the placement Xform
            prim = stage.DefinePrim(f"/World/{group}/item_{t}", "Xform")
            stage.DefinePrim(f"/World/{group}/item_{t}/asset", "Xform").GetReferences().AddInternalReference(proto)
            xf = UsdGeom.Xformable(prim)
            xf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
            xf.AddRotateZOp().Set(rng.uniform(0, 360))
            xf.AddScaleOp().Set(Gf.Vec3d(k, k, k))


def main(plants_json, out):
    with open(plants_json) as f:
        data = json.load(f)
    plants = data["plants"]
    if not plants:
        sys.exit("no plants")
    half = PLANT_DIAMETER / 2
    box = (min(p["x"] for p in plants) - half, min(p["y"] for p in plants) - half,
           max(p["x"] for p in plants) + half, max(p["y"] for p in plants) + half)
    center = ((box[0] + box[2]) / 2, (box[1] + box[3]) / 2)

    layer = Sdf.Layer.CreateNew(out)
    stage = Usd.Stage.Open(layer)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    stage.CreateClassPrim("/World/Prototypes")
    scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")  # setup_scene.py sets the fleet's settings on it
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr().Set(9.81)

    add_ground(stage, center)
    add_lights(stage)
    rows = add_plants(stage, plants)
    add_border(stage, box)

    # read by setup_scene.build_file_world: the web UI's spawn map draws these rows
    layer.customLayerData = {
        "lavender_rows": Vt.Vec4dArray([Gf.Vec4d(*r) for r in rows]),  # [x_min, x_max, y, width] per row
        "farm_source": data.get("source", ""),
        "plant_count": len(plants),
    }
    layer.Save()
    os.chmod(out, 0o666)  # written as the image's user; the host user may replace it
    print(f"wrote {out}: {len(plants)} plants in {len(rows)} rows, field box "
          f"x {box[0]:.2f}..{box[2]:.2f} y {box[1]:.2f}..{box[3]:.2f}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
