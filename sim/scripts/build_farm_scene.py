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

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

ASSETS = "/sim/assets"
REL_ASSETS = "../assets"  # as seen from sim/scene/<file>

LAVENDER = "lavender/SM_Lavender_Nanite_01.usd"
# Plant size. The asset is a ~2 m wide, 1.34 m tall clump in cm, scaled uniformly (its own proportions; a
# taller-than-wide scale looked squeezed) to this width, so it ends up ~0.55 m tall. Rows are ~1.85 m apart and
# plants ~0.44 m apart in a row, so the clumps overlap into a hedge and leave a ~1.03 m lane (the map,
# status_server, treats a row as 0.45 m thick; 0.5 m wide plants were only ~0.33 m tall; 0.75 m was made 10%
# bigger at the user's request).
PLANT_DIAMETER = 0.825
PLANT_SINK = 0.02  # lowest point below z=0, so no plant floats

# Weed barrier: the black woven landscape fabric laid under each real row (farm photo: a dark strip just past the
# foliage, no grass on it). One flat strip per row along the row's fitted line, a common 3 ft (0.9 m) roll (~4 cm
# past the 0.825 m plants each side; 1.1 m looked too wide to the user), running BARRIER_END_M past the end plants'
# centres. Visual only (no collider: driving and the lidars are unchanged); the grass blades rooted on it are hidden
# (hide_grass_under). Textured like a photo of the fabric, sparsely dusted with sand/dirt (more towards its edges):
# sim/assets/weed_barrier/, written by make_farm_textures.py; one tile spans the strip's width (BARRIER_TILE_M =
# BARRIER_WIDTH) with green guide lines on its centre line and 0.3 m either side; each row starts the tile at its own
# random offset along the row, so neighbouring rows don't show the same dirt.
BARRIER_WIDTH = 0.9
BARRIER_END_M = 0.5
BARRIER_Z = 0.0015  # above the ground (z=0); 4 mm showed as a thick edge
# Mounds: the asset's stems converge ~3.5 cm above its lowest (drooping) leaves, so on flat ground the crown floated
# over the fabric. Each plant gets a soil mound, MOUND_HEIGHT at its centre falling to 0 at MOUND_RADIUS (cos^2),
# that buries the stem bases; neighbours (0.44 m apart) merge into a wavy ridge (the higher of the two, ~2 cm
# between plants). The fabric is laid over them: each strip is a grid (MOUND_GRID_M cells) following the mounds,
# flat on the ground at its edges (MOUND_RADIUS < BARRIER_WIDTH / 2). Visual only, like the flat strip.
MOUND_HEIGHT = 0.06
MOUND_RADIUS = 0.35
MOUND_GRID_M = 0.03
BARRIER_TEX = "weed_barrier"  # folder in sim/assets: albedo.jpg, normal.png, roughness.jpg
BARRIER_TILE_M = 0.9  # make_farm_textures.BARRIER_TILE_M
BARRIER_COLOR = (0.03, 0.03, 0.032)  # displayColor only (the textures give the look)
# RTX renders the OmniPBR (MDL) version; UsdPreviewSurface stays as the fallback for other renderers. Seen from a
# Jackal's camera (~25 cm up, grazing) a UsdPreviewSurface strip mirrored the sky white even at specular 0.01 /
# roughness 0.95 (its Fresnel always reaches 1 at grazing); OmniPBR's specular_level weights the whole reflection.
BARRIER_SPECULAR_LEVEL = 0.1  # OmniPBR, default 0.5

GROUND_COVER = "Ground_cover/ground_cover.usd"  # 100 x 100 m of grass blades, geometry really in metres
GROUND_COVER_SCALE_Z = 0.6
GROUND_SIZE = 100.0
GROUND_SOIL_COLOR = (0.07, 0.05, 0.035)  # displayColor only: the soil textures give the look
# The visible ground: a textured quad at z=0 (dark grainy sandy soil, sim/assets/soil/ from make_farm_textures.py,
# one tile per SOIL_TILE_M); the ground box under it is guide purpose (not rendered, still the collider and the
# physics material), since a Cube has no UVs. The flat brown box looked artificial.
SOIL_TEX = "soil"
SOIL_TILE_M = 3.0  # make_farm_textures.SOIL_TILE_M
SOIL_SPECULAR_LEVEL = 0.15
SKY_HDR = "sky/farm_field_puresky_2k.hdr"
SKY_INTENSITY = 400
SUN_INTENSITY = 5000
SUN_ROTATE_XYZ = (20, 2.0, -50)  # degrees, the distant light's rotateXYZ (user's choice; was (-60, 33, -30) at 10000)

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


def textured_material(stage, path, folder, specular_level):
    """Material at `path` with sim/assets/<folder>/{albedo.jpg, normal.png, roughness.jpg} on the mesh's `st`:
    OmniPBR (MDL, what RTX renders) plus a UsdPreviewSurface fallback for other renderers."""
    files = {"albedo": "albedo.jpg", "normal": "normal.png", "roughness": "roughness.jpg"}
    surface = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("useSpecularWorkflow", Sdf.ValueTypeNames.Int).Set(1)
    shader.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.03))
    surface.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    st = UsdShade.Shader.Define(stage, f"{path}/st")
    st.CreateIdAttr("UsdPrimvarReader_float2")
    st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    st_out = st.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    def texture(name, color_space, output, out_type, **inputs):
        tex = UsdShade.Shader.Define(stage, f"{path}/{name}")
        tex.CreateIdAttr("UsdUVTexture")
        tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(f"{REL_ASSETS}/{folder}/{files[name]}")
        tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set(color_space)
        tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st_out)
        for wrap in ("wrapS", "wrapT"):
            tex.CreateInput(wrap, Sdf.ValueTypeNames.Token).Set("repeat")
        for k, v in inputs.items():
            tex.CreateInput(k, Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*v))
        return tex.CreateOutput(output, out_type)

    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        texture("albedo", "sRGB", "rgb", Sdf.ValueTypeNames.Float3))
    shader.CreateInput("normal", Sdf.ValueTypeNames.Normal3f).ConnectToSource(
        texture("normal", "raw", "rgb", Sdf.ValueTypeNames.Float3, scale=(2, 2, 2, 1), bias=(-1, -1, -1, 0)))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).ConnectToSource(
        texture("roughness", "raw", "r", Sdf.ValueTypeNames.Float))

    mdl = UsdShade.Shader.Define(stage, f"{path}/omnipbr")
    mdl.SetSourceAsset("OmniPBR.mdl", "mdl")
    mdl.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    mdl.CreateIdAttr("")
    mdl.GetPrim().GetAttribute("info:implementationSource").Set("sourceAsset")
    for name, key, space in (("albedo", "diffuse_texture", "sRGB"), ("roughness", "reflectionroughness_texture", "raw"),
                             ("normal", "normalmap_texture", "raw")):
        mdl.CreateInput(key, Sdf.ValueTypeNames.Asset).Set(f"{REL_ASSETS}/{folder}/{files[name]}")
        mdl.GetInput(key).GetAttr().SetColorSpace(space)
    mdl.CreateInput("reflection_roughness_texture_influence", Sdf.ValueTypeNames.Float).Set(1.0)
    mdl.CreateInput("specular_level", Sdf.ValueTypeNames.Float).Set(specular_level)
    mdl.CreateInput("uv_space_index", Sdf.ValueTypeNames.Int).Set(0)  # the mesh's st
    surface.CreateSurfaceOutput("mdl").ConnectToSource(mdl.ConnectableAPI(), "out")
    return surface


def add_ground(stage, center):
    """GROUND_SIZE box centred on the field, top face at z=0, grippy physics material, not rendered (guide); the
    visible ground is a soil-textured quad on its top face."""
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
    UsdGeom.Imageable(cube).CreatePurposeAttr(UsdGeom.Tokens.guide)  # still collides (cf. setup_scene's lavender cores)
    surf = UsdGeom.Mesh.Define(stage, "/World/ground_surface")
    h = GROUND_SIZE / 2
    corners = [(-h, -h), (h, -h), (h, h), (-h, h)]
    surf.CreatePointsAttr([Gf.Vec3f(center[0] + x, center[1] + y, 0.0) for x, y in corners])
    surf.CreateFaceVertexCountsAttr([4])
    surf.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    surf.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    surf.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    surf.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    surf.CreateExtentAttr([Gf.Vec3f(center[0] - h, center[1] - h, 0), Gf.Vec3f(center[0] + h, center[1] + h, 0)])
    UsdGeom.PrimvarsAPI(surf).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex).Set(
        [Gf.Vec2f((center[0] + x) / SOIL_TILE_M, (center[1] + y) / SOIL_TILE_M) for x, y in corners])
    surf.CreateDisplayColorAttr([Gf.Vec3f(*GROUND_SOIL_COLOR)])
    UsdShade.MaterialBindingAPI.Apply(surf.GetPrim()).Bind(
        textured_material(stage, "/World/Materials/ground_soil", SOIL_TEX, SOIL_SPECULAR_LEVEL))

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
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(*SUN_ROTATE_XYZ))


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


def row_strips(plants):
    """Per row: (row_id, start xy, end xy, unit direction xy, plant xys) of its weed barrier, on the least-squares
    line through the row's plants (the DB rows are straight to < 1 mm), BARRIER_END_M past the outermost plants."""
    rows = {}
    for p in plants:
        rows.setdefault(p["row_id"], []).append((p["x"], p["y"]))
    strips = []
    for row_id, pts in sorted(rows.items()):
        n = len(pts)
        mx, my = sum(x for x, _ in pts) / n, sum(y for _, y in pts) / n
        sxx = sum((x - mx) ** 2 for x, _ in pts)
        syy = sum((y - my) ** 2 for _, y in pts)
        sxy = sum((x - mx) * (y - my) for x, y in pts)
        a = 0.5 * math.atan2(2 * sxy, sxx - syy) if n > 1 else 0.0  # principal axis
        d = (math.cos(a), math.sin(a))
        ts = [(x - mx) * d[0] + (y - my) * d[1] for x, y in pts]
        t0, t1 = min(ts) - BARRIER_END_M, max(ts) + BARRIER_END_M
        strips.append((row_id, (mx + t0 * d[0], my + t0 * d[1]), (mx + t1 * d[0], my + t1 * d[1]), d, pts))
    return strips


def add_weed_barrier(stage, strips):
    """One mesh per row under /World/weed_barrier/row_<row_id>, laid over the plants' mounds, the woven-fabric
    textures, no collider."""
    surface = textured_material(stage, "/World/Materials/weed_barrier", BARRIER_TEX, BARRIER_SPECULAR_LEVEL)
    UsdGeom.Xform.Define(stage, "/World/weed_barrier")
    h = BARRIER_WIDTH / 2
    for row_id, (x0, y0), (x1, y1), (dx, dy), plant_xy in strips:
        length = math.hypot(x1 - x0, y1 - y0)
        # strip-local grid: t along the row from its start, s across (+s = left of the direction)
        t = np.linspace(0.0, length, max(2, round(length / MOUND_GRID_M) + 1))
        s_ = np.linspace(-h, h, max(2, round(BARRIER_WIDTH / MOUND_GRID_M) + 1))
        T, S = np.meshgrid(t, s_, indexing="ij")
        pxy = np.array(plant_xy) - (x0, y0)
        pt, ps = pxy @ (dx, dy), pxy @ (-dy, dx)
        r = np.hypot(T[..., None] - pt, S[..., None] - ps)
        Z = BARRIER_Z + MOUND_HEIGHT * (np.cos(np.minimum(r / MOUND_RADIUS, 1.0) * np.pi / 2) ** 2).max(-1)
        # smooth normals from the height field
        gz_t, gz_s = np.gradient(Z, t, s_)
        nt, ns = -gz_t, -gz_s
        nrm = np.stack([nt * dx - ns * dy, nt * dy + ns * dx, np.ones_like(Z)], -1)
        nrm /= np.linalg.norm(nrm, axis=-1, keepdims=True)
        X, Y = x0 + T * dx - S * dy, y0 + T * dy + S * dx
        nt_, ns_ = len(t), len(s_)
        idx = np.arange(nt_ * ns_).reshape(nt_, ns_)
        # quads counter-clockwise seen from above: (t, s) -> (t+1, s) -> (t+1, s+1) -> (t, s+1)
        quads = np.stack([idx[:-1, :-1], idx[1:, :-1], idx[1:, 1:], idx[:-1, 1:]], -1).reshape(-1)
        mesh = UsdGeom.Mesh.Define(stage, f"/World/weed_barrier/row_{row_id}")
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.stack([X, Y, Z], -1).reshape(-1, 3).astype(np.float32)))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(quads) // 4, 4, np.int32)))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(quads.astype(np.int32)))
        mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(nrm.reshape(-1, 3).astype(np.float32)))
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        # u: one tile per BARRIER_TILE_M along the row from a per-row offset; v: the tile across the strip's width
        u0 = random.Random(f"barrier{row_id}").random()
        uv = np.stack([u0 + T / BARRIER_TILE_M, 0.5 + S / BARRIER_TILE_M], -1).reshape(-1, 2).astype(np.float32)
        UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                                UsdGeom.Tokens.vertex).Set(Vt.Vec2fArray.FromNumpy(uv))
        mesh.CreateExtentAttr([Gf.Vec3f(float(X.min()), float(Y.min()), float(Z.min())),
                               Gf.Vec3f(float(X.max()), float(Y.max()), float(Z.max()))])
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*BARRIER_COLOR)])
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(surface)


def hide_grass_under(stage, strips, center):
    """invisibleIds on the ground cover's PointInstancers for every blade rooted on a weed barrier. The patch is
    at /World/GroundCover translated to `center`, unscaled in x/y, its positions in metres. Returns the count."""
    hidden = 0
    h = BARRIER_WIDTH / 2
    for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/GroundCover")):
        if not prim.IsA(UsdGeom.PointInstancer):
            continue
        pi = UsdGeom.PointInstancer(prim)
        ids = []
        for i, p in enumerate(pi.GetPositionsAttr().Get()):
            x, y = p[0] + center[0], p[1] + center[1]
            for _, (x0, y0), (x1, y1), (dx, dy), _ in strips:
                t = (x - x0) * dx + (y - y0) * dy
                if 0.0 <= t <= math.hypot(x1 - x0, y1 - y0) and abs(-(x - x0) * dy + (y - y0) * dx) <= h:
                    ids.append(i)
                    break
        pi.CreateInvisibleIdsAttr().Set(Vt.Int64Array(ids))
        hidden += len(ids)
    return hidden


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
    strips = row_strips(plants)
    add_weed_barrier(stage, strips)
    hidden = hide_grass_under(stage, strips, center)
    add_border(stage, box)

    # read by setup_scene.build_file_world: the web UI's spawn map draws these rows
    layer.customLayerData = {
        "lavender_rows": Vt.Vec4dArray([Gf.Vec4d(*r) for r in rows]),  # [x_min, x_max, y, width] per row
        "farm_source": data.get("source", ""),
        "plant_count": len(plants),
    }
    layer.Save()
    os.chmod(out, 0o666)  # written as the image's user; the host user may replace it
    print(f"wrote {out}: {len(plants)} plants in {len(rows)} rows, weed barrier under each "
          f"({hidden} grass blades hidden), field box "
          f"x {box[0]:.2f}..{box[2]:.2f} y {box[1]:.2f}..{box[3]:.2f}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
