"""Farm workers (optional, FARM_WORKERS=1): two people sitting still on wooden crates either side of a lavender row,
facing each other across it.

`add(stage, rows)` (called from setup_scene.main after build_world) references sim/assets/farm_workers/<name>/<name>.usda
under /World/FarmWorkers -- an NVIDIA digital human in a static seated pose (FarmPose, retargeted NVIDIA mocap) with
an invisible capsule collider on a kinematic body, so lidars and robots see it -- and a crate under each one
(sim/assets/farm_workers/crate/crate.usd, scaled to the worker's farm:seatHeight, static box collider).
Nothing runs per frame. Built in gen_3d_model/scripts/farm_workers/.
The bodies are referenced from NVIDIA's S3 content server; the first load downloads them and compiles their MDL
shaders (minutes, cached afterwards), and they cost frame rate and memory, hence off by default.
"""
import os

from pxr import Gf, UsdGeom, UsdPhysics

ENABLED = os.environ.get("FARM_WORKERS", "0") == "1"
ROOT = "/World/FarmWorkers"
ASSET_DIR = "/sim/assets/farm_workers"
WORKERS = ("farm_worker_01", "farm_worker_02")
SPOT_ROW = 2              # lavender row they sit at, 0 = the northernmost
SPOT_FROM_END_M = 2.5     # distance from the row's west end
FEET_FROM_ROW_M = 0.5     # each worker's feet from the row's centre line (plants are ~0.75 m wide)
CRATE_SIZE = (0.40, 0.32) # crate.usd footprint (x, y); it is 1 m tall and scaled to the seat height


def log(msg):
    print(f"[farm_workers] {msg}", flush=True)


def add(stage, rows):
    """Seat the workers either side of row SPOT_ROW, facing each other across it."""
    rows = sorted(rows or [], key=lambda r: r[2], reverse=True)
    if rows:
        r = rows[min(SPOT_ROW, len(rows) - 1)]
        x, y = r[0] + SPOT_FROM_END_M, r[2]
    else:
        x, y = 2.0, 2.0
    # the characters face -Y in their own frame: rotateZ 0 faces south, 180 north
    spots = [(x, y + FEET_FROM_ROW_M, 0.0), (x + 0.5, y - FEET_FROM_ROW_M, 180.0)]
    UsdGeom.Xform.Define(stage, ROOT)
    for name, (px, py, rz) in zip(WORKERS, spots):
        path = f"{ROOT}/{name}"
        prim = stage.DefinePrim(path, "Xform")
        prim.GetReferences().AddReference(f"{ASSET_DIR}/{name}/{name}.usda")
        ops = {op.GetOpName(): op for op in UsdGeom.Xformable(prim).GetOrderedXformOps()}
        ops["xformOp:translate"].Set(Gf.Vec3d(px, py, 0.0))
        ops["xformOp:rotateZ"].Set(rz)
        seat_h = prim.GetAttribute("farm:seatHeight").Get() or 0.5
        seat_y = prim.GetAttribute("farm:seatOffsetY").Get() or 0.45
        # crate in the worker's frame (its back is +Y), placed as a sibling so the worker's body stays kinematic
        crate = UsdGeom.Xform.Define(stage, f"{ROOT}/{name}_crate")
        crate.AddTranslateOp().Set(Gf.Vec3d(px, py, 0.0))
        crate.AddRotateZOp().Set(rz)
        model = UsdGeom.Xform.Define(stage, f"{ROOT}/{name}_crate/model")
        model.GetPrim().GetReferences().AddReference(f"{ASSET_DIR}/crate/crate.usd")
        model.AddTranslateOp().Set(Gf.Vec3d(0.0, seat_y, 0.0))
        model.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, seat_h))
        box = UsdGeom.Cube.Define(stage, f"{ROOT}/{name}_crate/collider")
        box.CreateSizeAttr(1.0)
        box.AddTranslateOp().Set(Gf.Vec3d(0.0, seat_y, seat_h / 2))
        box.AddScaleOp().Set(Gf.Vec3f(CRATE_SIZE[0], CRATE_SIZE[1], seat_h))
        box.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
        UsdPhysics.CollisionAPI.Apply(box.GetPrim())
    log(f"{len(WORKERS)} workers seated at row {SPOT_ROW} (y={y:.2f}), x={x:.2f}")
