"""Photo-realistic robot materials (ROBOT_LOOKS).

The URDF importer gives every visual a UsdPreviewSurface with one flat colour, roughness 0.5 and metallic 0, so tyres,
paint, aluminium and the Kinova arm all look like the same smooth, spotless plastic. This module rebinds every robot
visual to a "look": an NVIDIA OmniPBR / OmniPBR_ClearCoat MDL material with tileable, procedurally generated colour,
roughness and normal textures (powder-coat grain, orange-peel paint, brushed aluminium, rubber, scratches, dust and
mud), modelled on the photos in robot_data/pictures/.

ROBOT_LOOKS (env, default "full"):
  full   textured looks (dust, scratches, surface relief) + rounded-edge shading
  basic  the same materials as constants, no textures, no rounded edges (cheaper: no texture memory or lookups)
  0/off  the importer's own materials, untouched

How (see apply()): the robots' visuals are instanced, so their bindings can't be authored on the stage (instance
proxies). They live in each model's payloads/instances.usda, a layer the stage references; apply() edits that layer
*in memory* (never saved, so the import cache in sim/generated/<model>/ stays as imported) before the robots are
spawned: each `material:binding` there is retargeted to a new Material prim that references
/sim/generated/looks/looks_<mode>.usda</Looks/<look>_<dust>>. Physics-purpose bindings and colliders are not touched.
The parts' meshes have no UVs (STL), so the textures use OmniPBR's object-space cubic projection; texture_scale is
set per part from the mesh's own unit size, so a texture tile is TILE_M metres on every part.
"""
import colorsys
import json
import math
import os
import re

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

MODE = {"1": "full", "on": "full", "true": "full", "0": "off", "false": "off", "none": "off"}.get(
    os.environ.get("ROBOT_LOOKS", "full").strip().lower(), os.environ.get("ROBOT_LOOKS", "full").strip().lower())
OUT_DIR = "/sim/generated/looks"
TEX_SIZE = 1024
TILE_M = 0.5  # one texture tile covers 0.5 m x 0.5 m of a part
LOW_PART_Z = 0.18  # m above the robot's lowest point: parts centred lower get the heavy dust/mud variant
ROUND_EDGES_M = 0.003  # shading-only edge bevel radius (OmniPBR round_edges_radius), "full" mode only
VERSION = 7  # bump to regenerate the cached textures / library

# Dust/mud colours (linear). Dry dust on a dark part reads lighter, mud on the tyres darker -- from the photos.
DUST = (0.27, 0.22, 0.165)
MUD = (0.10, 0.065, 0.04)

# Looks. Colours are linear RGB (sRGB from the photos converted). surface = height-field kind used for the normal
# map; scratch = (density, colour, roughness); clearcoat = (weight, roughness) -> OmniPBR_ClearCoat.
LOOKS = {
    # Clearpath yellow: glossy, slightly orange-peel paint (A200/A300 front panels, Jackal body). sRGB ~(235,175,0)
    "paint_yellow": dict(color=(0.83, 0.43, 0.0), rough=0.36, surface="peel", bump=0.08, clearcoat=(0.5, 0.08),
                         scratch=(0.15, (0.55, 0.40, 0.20), 0.55), dust=0.8),
    "paint_orange": dict(color=(0.85, 0.33, 0.0), rough=0.36, surface="peel", bump=0.08, clearcoat=(0.5, 0.08),
                         scratch=(0.15, (0.55, 0.35, 0.18), 0.55), dust=0.8),
    # Black powder-coated steel: top plates, chassis, arches. The A200's plate is visibly scuffed.
    "powder_black": dict(color=(0.017, 0.017, 0.018), rough=0.62, surface="grain", bump=0.45,
                         scratch=(0.6, (0.09, 0.09, 0.09), 0.4), dust=0.6),
    # Semi-gloss black tube bumpers.
    "bumper_black": dict(color=(0.014, 0.014, 0.015), rough=0.33, surface="grain", bump=0.25,
                         scratch=(0.8, (0.10, 0.10, 0.10), 0.45), dust=0.5),
    # Knobby black rubber tyres.
    "rubber": dict(color=(0.021, 0.020, 0.019), rough=0.86, surface="rubber", bump=0.6, scratch=None, dust=1.3,
                   dust_color=MUD),
    # Silver aluminium extrusion / plates.
    "aluminium": dict(color=(0.80, 0.80, 0.80), rough=0.30, metallic=1.0, surface="brushed", bump=0.12,
                      scratch=(0.3, (0.95, 0.95, 0.95), 0.18), dust=0.5),
    # Dark anodised aluminium sensor housings (RealSense, VLP16, Robotiq knuckles).
    "anodized_grey": dict(color=(0.20, 0.20, 0.21), rough=0.38, metallic=0.7, surface="grain", bump=0.2,
                          scratch=(0.2, (0.5, 0.5, 0.5), 0.25), dust=0.4),
    # Kinova arm links, glossy white plastic.
    "plastic_white_gloss": dict(color=(0.76, 0.76, 0.75), rough=0.22, surface="smooth", bump=0.15,
                                clearcoat=(0.3, 0.12), scratch=(0.15, (0.45, 0.44, 0.42), 0.4), dust=0.35),
    # GNSS domes, enclosures: matte white plastic.
    "plastic_white_matte": dict(color=(0.72, 0.72, 0.70), rough=0.55, surface="grain", bump=0.2, scratch=None,
                                dust=0.5),
    "plastic_black": dict(color=(0.025, 0.025, 0.026), rough=0.45, surface="grain", bump=0.3,
                          scratch=(0.2, (0.07, 0.07, 0.07), 0.35), dust=0.45),
    "plastic_red": dict(color=(0.55, 0.012, 0.01), rough=0.28, surface="smooth", bump=0.15, scratch=None, dust=0.3),
    # Any other coloured part keeps its importer colour (tinted neutral texture).
    "plastic_tinted": dict(color=None, rough=0.45, surface="grain", bump=0.25, scratch=None, dust=0.5),
    "glass_black": dict(color=(0.002, 0.002, 0.002), rough=0.04, surface=None, bump=0.0, scratch=None, dust=0.0),
}

# Part rules, first match wins. Keys: regex on the instance (mesh) name, the material name, or a colour class
# ("yellow", "orange", "red", "blue", "white", "black", "grey"). Model-specific rules (MODEL_RULES) go first.
RULES = [
    (dict(inst=r"outdoor|wheel|tire|tyre|mecanum"), "rubber"),
    (dict(inst=r"bumper"), "bumper_black"),
    (dict(mat=r"^lense?"), "glass_black"),
    (dict(inst=r"gnss|gps|antenna"), "plastic_white_matte"),
    (dict(inst=r"finger_(prox|dist)_link"), "plastic_black"),  # Kinova 2F Lite fingers
    (dict(inst=r"finger", cls="grey"), "anodized_grey"),  # Robotiq 2F-85 finger links
    (dict(inst=r"finger"), "plastic_black"),
    (dict(inst=r"knuckle", cls="grey"), "anodized_grey"),
    (dict(inst=r"knuckle|robotiq", cls="black"), "plastic_black"),
    (dict(inst=r"robotiq", cls="grey"), "anodized_grey"),
    # Kinova Gen3 / Gen3 Lite links: their importer materials are "material_<n>", grey
    (dict(mat=r"(?-i:^material_\d+$)", cls="grey"), "plastic_white_gloss"),
    (dict(mat=r"(?-i:^material_\d+$)", cls="white"), "plastic_white_gloss"),
    (dict(inst=r"vlp16_scan", cls="black"), "glass_black"),  # VLP16 scan window band
    (dict(inst=r"d435|d405|vlp16|velodyne"), "anodized_grey"),
    (dict(inst=r"mount|bracket", cls="grey"), "anodized_grey"),
    (dict(inst=r"zed|hokuyo"), "plastic_black"),
    (dict(cls="yellow"), "paint_yellow"),
    (dict(cls="orange"), "paint_orange"),
    (dict(cls="red"), "plastic_red"),
    (dict(cls="white"), "plastic_white_matte"),
    (dict(cls="black"), "powder_black"),
    (dict(cls="blue"), "plastic_tinted"),
    (dict(cls="grey"), "plastic_tinted"),
]
# From the photos, where the URDF colour is wrong for the real robot.
MODEL_RULES = {
    "a300_00036": [(dict(inst=r"sensor_arch"), "aluminium")],  # silver extrusion posts
    "j100_0921": [(dict(inst=r"top_assy"), "aluminium")],  # MTU top assembly: brushed aluminium plate
    "j100_0922": [(dict(inst=r"top_assy"), "aluminium")],
    "j100_0936": [(dict(inst=r"top_assy"), "aluminium")],
}

_held_layers = []  # in-memory edits live only as long as the layer object does


def enabled():
    return MODE in ("full", "basic")


def _log(msg):
    print(f"[fleet] looks: {msg}", flush=True)


# --------------------------------------------------------------------------------------------------------------
# Procedural textures (numpy). All fields are periodic (FFT synthesis, wrap-around scratches), so they tile.

def _np():
    import numpy as np

    return np


def _field(n, rng, f_lo, f_hi, beta=1.0, aniso=1.0):
    """Zero-mean, unit-variance periodic noise with energy between f_lo..f_hi cycles per tile (1/f^beta)."""
    np = _np()
    fx = np.fft.fftfreq(n)[None, :] * n * aniso
    fy = np.fft.fftfreq(n)[:, None] * n
    f = np.sqrt(fx * fx + fy * fy)
    f[0, 0] = 1.0
    band = np.exp(-(f / f_hi) ** 2) * (1.0 - np.exp(-(f / max(f_lo, 1e-3)) ** 2))
    spec = (rng.normal(size=(n, n)) + 1j * rng.normal(size=(n, n))) * band / f ** beta
    spec[0, 0] = 0.0
    x = np.fft.ifft2(spec).real
    return (x - x.mean()) / (x.std() + 1e-9)


def _smoothstep(a, b, x):
    np = _np()
    t = np.clip((x - a) / (b - a), 0.0, 1.0)
    return t * t * (3 - 2 * t)


def _scratches(n, rng, count, px_per_m):
    """Thin wrap-around line segments, 2-30 mm long; returns a 0..0.7 mask."""
    np = _np()
    m = np.zeros((n, n), np.float32)
    for _ in range(int(count)):
        length = rng.uniform(0.002, 0.03) * px_per_m
        ang = rng.normal(0.4, 0.5) if rng.random() < 0.7 else rng.uniform(0, math.pi)  # scuffs share a direction
        x0, y0 = rng.uniform(0, n, 2)
        k = max(2, int(length * 1.5))
        t = np.linspace(0, 1, k)
        xs = (x0 + t * length * math.cos(ang) + rng.normal(0, 0.3, k).cumsum() * 0.2).astype(int) % n
        ys = (y0 + t * length * math.sin(ang)).astype(int) % n
        np.maximum.at(m, (ys, xs), rng.uniform(0.25, 0.7) * np.sin(t * math.pi) ** 0.3)
    # 1 px lines are aliased away at a distance; widen slightly
    return np.maximum(m, 0.5 * (np.roll(m, 1, 0) + np.roll(m, 1, 1)))


def _height(kind, n, rng):
    np = _np()
    if kind == "peel":  # orange-peel paint: soft bumps ~2-3 mm
        return _field(n, rng, 120, 260, beta=0.5)
    if kind == "grain":  # powder coat / textured plastic: fine stipple
        return 0.6 * _field(n, rng, 300, 900, beta=0.2) + 0.4 * _field(n, rng, 80, 200, beta=0.5)
    if kind == "brushed":  # streaks along x
        return _field(n, rng, 40, 600, beta=0.4, aniso=40.0)
    if kind == "rubber":  # moulded rubber: mottled
        return 0.6 * _field(n, rng, 150, 500, beta=0.3) + 0.4 * _field(n, rng, 20, 80, beta=0.8)
    if kind == "smooth":  # moulded plastic, faint waviness
        return _field(n, rng, 10, 60, beta=1.0)
    return np.zeros((n, n))


def _normal_map(h, strength):
    np = _np()
    dx = (np.roll(h, -1, 1) - np.roll(h, 1, 1)) * 0.5 * strength
    dy = (np.roll(h, -1, 0) - np.roll(h, 1, 0)) * 0.5 * strength
    nz = np.ones_like(h)
    inv = 1.0 / np.sqrt(dx * dx + dy * dy + nz)
    rgb = np.stack([-dx * inv, -dy * inv, nz * inv], -1)
    return ((rgb * 0.5 + 0.5) * 255 + 0.5).clip(0, 255).astype(np.uint8)


def _srgb8(lin):
    np = _np()
    lin = np.clip(lin, 0.0, 1.0)
    s = np.where(lin <= 0.0031308, lin * 12.92, 1.055 * np.power(lin, 1 / 2.4) - 0.055)
    return (s * 255 + 0.5).astype(np.uint8)


def _make_textures(name, look, heavy, out_dir):
    """Writes <name>_{albedo,rough,normal}.png; returns their paths."""
    np = _np()
    from PIL import Image

    n = TEX_SIZE
    seed = sum(map(ord, name)) * 7919 + (1 if heavy else 0)
    rng = np.random.default_rng(seed)
    px_per_m = n / TILE_M
    base = np.array(look["color"] if look["color"] else (1.0, 1.0, 1.0), np.float32)
    rough = np.full((n, n), look["rough"], np.float32)

    # base colour: slight low-frequency mottling (real paint/plastic is never one flat value)
    mott = _field(n, rng, 2, 30, beta=1.0)
    albedo = base[None, None, :] * (1.0 + 0.025 * mott[..., None])
    rough += 0.04 * _field(n, rng, 4, 60, beta=0.8)

    height = _height(look["surface"], n, rng) * 0.6

    # finger smudges / water spots on glossy parts: rougher, faint
    if look["rough"] < 0.45:
        smudge = _smoothstep(0.8, 2.2, _field(n, rng, 3, 25, beta=0.8))
        rough += smudge * 0.12

    if look.get("scratch"):
        density, scol, srough = look["scratch"]
        s = _scratches(n, rng, density * 120 * (1.5 if heavy else 1.0), px_per_m)
        albedo = albedo * (1 - s[..., None]) + np.array(scol, np.float32)[None, None, :] * s[..., None]
        rough = rough * (1 - s) + srough * s
        height -= 1.5 * s

    amount = look.get("dust", 0.0) * (1.0 if heavy else 0.45)
    if amount > 0:
        dcol = np.array(look.get("dust_color", DUST), np.float32)
        lowf = _field(n, rng, 1.5, 12, beta=1.0) + 0.35 * _field(n, rng, 10, 60, beta=0.6)
        thr = 1.2 - 0.8 * min(amount, 1.3)  # more dust: lower threshold, larger patches
        peak = min(0.7, 0.2 + 0.3 * amount)  # dust is a film, never fully opaque
        grain = np.clip(0.75 + 0.35 * _field(n, rng, 150, 500, beta=0.2), 0.3, 1.2)
        mask = _smoothstep(thr - 0.9, thr + 1.6, lowf) * peak * grain
        mask += 0.05 * amount * (1.0 + 0.3 * _field(n, rng, 2, 20, beta=1.0))  # haze
        speck = _smoothstep(2.4, 3.4, _field(n, rng, 250, 700, beta=0.1)) * 0.6 * amount
        mask = np.clip(mask + speck, 0.0, 0.85)
        albedo = albedo * (1 - mask[..., None]) + dcol[None, None, :] * mask[..., None]
        rough = rough * (1 - mask) + 0.9 * mask
        height += 0.6 * mask * _field(n, rng, 200, 600, beta=0.2)

    paths = {k: f"{out_dir}/{name}_{k}.png" for k in ("albedo", "rough", "normal")}
    Image.fromarray(_srgb8(albedo)).save(paths["albedo"])
    Image.fromarray((np.clip(rough, 0.02, 1.0) * 255 + 0.5).astype(np.uint8)).save(paths["rough"])
    Image.fromarray(_normal_map(height, look["bump"] * 3.0)).save(paths["normal"])
    return paths


# --------------------------------------------------------------------------------------------------------------
# Material library: /sim/generated/looks/looks_<mode>.usda, /Looks/<look>_<light|heavy>

def _variant_names():
    for name in LOOKS:
        for dust in ("light", "heavy"):
            yield name, dust, f"{name}_{dust}"


def build_library():
    """(Re)writes the textures (cached by a stamp) and the material library layer for MODE; returns its path."""
    os.makedirs(f"{OUT_DIR}/textures", exist_ok=True)
    lib_path = f"{OUT_DIR}/looks_{MODE}.usda"
    stamp_path = f"{OUT_DIR}/.stamp_{MODE}"
    stamp = json.dumps({"v": VERSION, "looks": LOOKS, "tex": TEX_SIZE, "tile": TILE_M, "dust": [DUST, MUD]},
                       sort_keys=True)
    try:
        if open(stamp_path).read() == stamp and os.path.exists(lib_path):
            return lib_path
    except OSError:
        pass
    import time

    t0 = time.time()
    layer = Sdf.Layer.CreateAnonymous()
    stage = Usd.Stage.Open(layer)
    UsdGeom.Scope.Define(stage, "/Looks")
    for name, dust, vname in _variant_names():
        look = LOOKS[name]
        cc = look.get("clearcoat")
        mdl = "OmniPBR_ClearCoat" if cc else "OmniPBR"
        mat = UsdShade.Material.Define(stage, f"/Looks/{vname}")
        sh = UsdShade.Shader.Define(stage, f"/Looks/{vname}/Shader")
        sh.SetSourceAsset(f"{mdl}.mdl", "mdl")
        sh.SetSourceAssetSubIdentifier(mdl, "mdl")
        sh.CreateIdAttr("")
        sh.GetPrim().GetAttribute("info:implementationSource").Set("sourceAsset")
        f = Sdf.ValueTypeNames

        def inp(k, t, v):
            sh.CreateInput(k, t).Set(v)

        col = look["color"] or (1.0, 1.0, 1.0)
        inp("diffuse_color_constant", f.Color3f, Gf.Vec3f(*col))
        inp("reflection_roughness_constant", f.Float, float(look["rough"]))
        inp("metallic_constant", f.Float, float(look.get("metallic", 0.0)))
        if cc:
            inp("clearcoat_weight", f.Float, float(cc[0]))
            inp("clearcoat_reflection_roughness", f.Float, float(cc[1]))
        if MODE == "full" and look["surface"]:
            tex = _make_textures(vname, look, dust == "heavy", f"{OUT_DIR}/textures")
            inp("diffuse_texture", f.Asset, Sdf.AssetPath(tex["albedo"]))
            sh.GetInput("diffuse_texture").GetAttr().SetColorSpace("sRGB")
            inp("reflectionroughness_texture", f.Asset, Sdf.AssetPath(tex["rough"]))
            sh.GetInput("reflectionroughness_texture").GetAttr().SetColorSpace("raw")
            inp("reflection_roughness_texture_influence", f.Float, 1.0)
            inp("normalmap_texture", f.Asset, Sdf.AssetPath(tex["normal"]))
            sh.GetInput("normalmap_texture").GetAttr().SetColorSpace("raw")
            inp("bump_factor", f.Float, 1.0)
            inp("project_uvw", f.Bool, True)
            inp("world_or_object", f.Bool, False)
            inp("texture_scale", f.Float2, Gf.Vec2f(1.0, 1.0))
            inp("round_edges_radius", f.Float, ROUND_EDGES_M)
        out = sh.CreateOutput("out", Sdf.ValueTypeNames.Token)
        for o in ("surface", "displacement", "volume"):
            mat.CreateOutput(f"mdl:{o}", Sdf.ValueTypeNames.Token).ConnectToSource(out)
    layer.Export(lib_path)
    with open(stamp_path, "w") as fh:
        fh.write(stamp)
    _log(f"built {lib_path} ({len(LOOKS) * 2} materials) in {time.time() - t0:.1f} s")
    return lib_path


# --------------------------------------------------------------------------------------------------------------
# Classification and rebinding

def _color_class(c):
    if c is None:
        return "grey"
    r, g, b = c
    v = max(r, g, b)  # linear
    srgb = [1.055 * max(x, 0) ** (1 / 2.4) - 0.055 if x > 0.0031308 else 12.92 * max(x, 0) for x in c]
    h, s, _ = colorsys.rgb_to_hsv(*srgb)  # hue/saturation as a person would name the colour
    if v < 0.07:
        return "black"
    if s < 0.25:
        return "white" if v > 0.8 else "grey"
    deg = h * 360
    if deg < 15 or deg >= 330:
        return "red"
    if deg < 40:  # A300 livery amber (sRGB ~(255,153,0)) is 36 deg, Clearpath yellow (255,184,0) 43 deg
        return "orange"
    if deg < 75:
        return "yellow"
    if 180 <= deg < 270:
        return "blue"
    return "grey"


def _material_info(mat_prim):
    """(diffuse colour, emissive?) of an importer material (UsdPreviewSurface)."""
    # The importer puts the values on the Material's interface inputs (the shader connects to them); fall back to
    # the surface shader's own inputs.
    mat = UsdShade.Material(mat_prim)
    out = mat.GetSurfaceOutput()
    src = out.GetConnectedSources()[0] if out and out.HasConnectedSource() else []
    shader = UsdShade.Shader(src[0].source.GetPrim()) if src else None

    def value(name):
        for owner in (mat, shader):
            i = owner.GetInput(name) if owner else None
            if i and i.Get() is not None:
                return tuple(i.Get())
        return None

    e = value("emissiveColor")
    return value("diffuseColor"), bool(e and max(e) > 0)


def classify(model, inst, mat_name, color):
    cls = _color_class(color)
    for cond, look in MODEL_RULES.get(model, []) + RULES:
        if "inst" in cond and not re.search(cond["inst"], inst, re.I):
            continue
        if "mat" in cond and not re.search(cond["mat"], mat_name, re.I):
            continue
        if "cls" in cond and cond["cls"] != cls:
            continue
        return look
    return "plastic_tinted"


def _gprim_info(stage):
    """{key: (unit size in m of the part's object space, lowest centre z above the robot's lowest point)} for every
    visual gprim of a model stage (rest pose). key = instance name (instances.usda) for instanced meshes, else the
    prim path of each ancestor (base.usda primitives such as URDF boxes)."""
    cache = UsdGeom.XformCache()
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    info, zmin = {}, float("inf")
    for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Gprim) or prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        r = bbox.ComputeWorldBound(prim).ComputeAlignedRange()
        if r.IsEmpty():
            continue
        m = cache.GetLocalToWorldTransform(prim)
        scale = [Gf.Vec3d(m[i][0], m[i][1], m[i][2]).GetLength() for i in range(3)]
        unit = (scale[0] * scale[1] * scale[2]) ** (1 / 3)  # non-uniform (URDF boxes): the geometric mean
        zmin = min(zmin, r.GetMin()[2])
        cz = (r.GetMin()[2] + r.GetMax()[2]) / 2
        keys = []
        anc = prim
        while anc and not anc.IsInstance() and not anc.IsPseudoRoot():
            keys.append(str(anc.GetPath()))
            anc = anc.GetParent()
        if anc and anc.IsInstance():
            refs = [r for spec in anc.GetPrimStack() for r in spec.referenceList.GetAddedOrExplicitItems()]
            keys = [str(r.primPath).rsplit("/", 1)[-1] for r in refs if "instances" in r.assetPath][:1]
        for k in keys:
            u0, z0 = info.get(k, (unit, cz))
            info[k] = (u0, min(z0, cz))
    return {k: (u, z - zmin) for k, (u, z) in info.items()}


def _bindings(stage, layer, under, name_of):
    """(name, binding rel path, target, material name, colour, emissive) for each visual binding authored in
    `layer` on prims under `under` of `stage`."""
    found = []
    root = stage.GetPrimAtPath(under)
    for prim in Usd.PrimRange(root) if root else []:
        rel = prim.GetRelationship("material:binding")
        if not rel or not rel.GetTargets() or layer.GetPropertyAtPath(rel.GetPath()) is None:
            continue
        target = rel.GetTargets()[0]
        mp = stage.GetPrimAtPath(target)
        if mp and "look_" not in mp.GetName():
            found.append((name_of(prim), rel.GetPath(), target, mp.GetName()) + _material_info(mp))
    return found


def apply(model, usd_path, lib_path):
    """Retarget the visual bindings of one imported model (in memory) to the looks library. Call after the import,
    before the robots are spawned. Instanced meshes are bound in payloads/instances.usda, URDF primitives (boxes,
    cylinders) in payloads/base.usda; both layers are edited."""
    payloads = os.path.join(os.path.dirname(usd_path), "payloads")
    inst_layer = Sdf.Layer.FindOrOpen(f"{payloads}/instances.usda")
    base_layer = Sdf.Layer.FindOrOpen(f"{payloads}/base.usda")
    if inst_layer is None or base_layer is None:
        _log(f"{model}: no instances.usda/base.usda in {payloads}, skipped")
        return
    _held_layers.extend([inst_layer, base_layer])
    # collect everything first, edit after (editing a layer recomposes the stages reading it)
    model_stage = Usd.Stage.Open(usd_path)
    parts = _gprim_info(model_stage)
    inst_stage = Usd.Stage.Open(inst_layer)  # instances.usda alone: its prims are the instance prototypes
    jobs = [(inst_layer, b) for b in _bindings(inst_stage, inst_layer, "/Instances",
                                               lambda p: str(p.GetPath()).split("/")[2])]
    root = model_stage.GetDefaultPrim()
    jobs += [(base_layer, b) for b in _bindings(model_stage, base_layer, root.GetPath(),
                                                lambda p: f"{p.GetParent().GetName()}/{p.GetName()}")]
    keyed = {id(b): str(b[1].GetPrimPath()) for _, b in jobs}
    del inst_stage, model_stage
    counts, changed = {}, 0
    for layer, b in jobs:
        inst, rel_path, target, mat_name, color, emissive = b
        if emissive:
            continue  # status lights keep their glow
        unit, zrel = parts.get(inst) or parts.get(keyed[id(b)]) or (1.0, 1.0)
        look = classify(model, inst, mat_name, color)
        heavy = look == "rubber" or "bumper" in look or zrel < LOW_PART_Z
        vname = f"{look}_{'heavy' if heavy else 'light'}"
        suffix = vname
        if look == "plastic_tinted" and color:  # one prim per colour (an instance can have several)
            suffix += "_" + "".join(f"{int(min(max(c, 0), 1) * 255):02x}" for c in color)
        if MODE == "full":  # texture scale differs per part
            suffix += f"_u{unit:.4g}".replace(".", "p")
        new_path = target.GetParentPath().AppendChild(f"look_{suffix}")
        if not layer.GetPrimAtPath(new_path):
            spec = Sdf.CreatePrimInLayer(layer, new_path)
            spec.specifier = Sdf.SpecifierDef
            spec.typeName = "Material"
            spec.referenceList.Prepend(Sdf.Reference(lib_path, f"/Looks/{vname}"))
            sh = Sdf.CreatePrimInLayer(layer, new_path.AppendChild("Shader"))
            sh.specifier = Sdf.SpecifierOver
            if MODE == "full":
                s = unit / TILE_M  # object units -> tiles
                a = Sdf.AttributeSpec(sh, "inputs:texture_scale", Sdf.ValueTypeNames.Float2)
                a.default = Gf.Vec2f(s, s)
                if look == "plastic_tinted" and color:
                    t = Sdf.AttributeSpec(sh, "inputs:diffuse_tint", Sdf.ValueTypeNames.Color3f)
                    t.default = Gf.Vec3f(*color)
            if look == "plastic_tinted" and color:
                c = Sdf.AttributeSpec(sh, "inputs:diffuse_color_constant", Sdf.ValueTypeNames.Color3f)
                c.default = Gf.Vec3f(*color)
        rspec = layer.GetPropertyAtPath(rel_path)
        rspec.targetPathList.ClearEditsAndMakeExplicit()
        rspec.targetPathList.explicitItems = [new_path]
        counts[look] = counts.get(look, 0) + 1
        changed += 1
    _log(f"{model}: {changed} bindings -> " + ", ".join(f"{k} {v}" for k, v in sorted(counts.items())))
