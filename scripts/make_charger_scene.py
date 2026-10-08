#!/usr/bin/env python3
"""Build sim/scene/lavender_farm_chargers.usda: an existing farm scene plus the farm database's charging stations.

  scripts/make_charger_scene.py [--farm lavender_farm.usd] [--out lavender_farm_chargers.usda]
                                [--dbname NAME] [--host H] [--port P]

Reads every row of public.charging_stations (id, charger_model, x_coord, y_coord, yaw_coord_deg, apriltag_id,
apriltag_sz_mm, frame) with the same connection as scripts/make_farm_scene.py, writes them to
sim/generated/farm/chargers.json and runs sim/scripts/add_chargers.py in the a300-isaac-sim image (no GPU, no Kit;
the running sim is not touched). The result has the farm file as its sublayer, so rebuilding the farm with
make_farm_scene.py keeps the chargers; rerun this when the charging_stations rows change. Use it with
SIM_SCENE=lavender_farm_chargers.usda in .env or the web UI's Scene picker.
x/y is the charger's footprint centre, yaw the direction its front (AprilTag side) faces, CCW from the frame's +X.
"""
import argparse
import datetime
import json
import sys

from make_farm_scene import ROOT, add_db_args, connect, run_usd_script


def fetch_chargers(args):
    conn, dbname = connect(args)
    with conn:
        rows = conn.execute("SELECT id, charger_model, x_coord, y_coord, yaw_coord_deg, apriltag_id, apriltag_sz_mm, "
                            "frame FROM public.charging_stations ORDER BY id").fetchall()
    chargers, problems = [], []
    for i, model, x, y, yaw, tag, size, frame in rows:
        missing = [n for n, v in (("charger_model", model), ("x_coord", x), ("y_coord", y),
                                  ("yaw_coord_deg", yaw), ("apriltag_id", tag), ("frame", frame)) if v is None]
        if missing:
            problems.append(f"charger {i}: no {', '.join(missing)}")
            continue
        chargers.append({"id": i, "charger_model": model, "x": x, "y": y, "yaw_deg": yaw, "apriltag_id": tag,
                         "apriltag_sz_mm": size, "frame": frame})
    stamp = datetime.datetime.now().isoformat(timespec="seconds")
    return {"source": f"{dbname}.public.charging_stations @ {stamp}", "chargers": chargers}, problems


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--farm", default="lavender_farm.usd", help="farm scene in sim/scene/ (default %(default)s)")
    ap.add_argument("--out", default="lavender_farm_chargers.usda",
                    help="file name in sim/scene/ (default %(default)s)")
    add_db_args(ap)
    args = ap.parse_args()

    data, problems = fetch_chargers(args)
    if problems:
        sys.exit("charging_stations rows left incomplete:\n  " + "\n  ".join(problems))
    if not data["chargers"]:
        sys.exit("charging_stations has no rows")
    sim = args.sim_dir.resolve()
    if not (sim / "scene" / args.farm).exists():
        sys.exit(f"no farm scene {sim / 'scene' / args.farm}: build it with scripts/make_farm_scene.py")
    farm_dir = sim / "generated/farm"
    farm_dir.mkdir(parents=True, exist_ok=True)
    (farm_dir / "chargers.json").write_text(json.dumps(data, indent=1))
    print(f"{len(data['chargers'])} chargers from {data['source']}")

    out = sim / "scene" / args.out
    if out.exists():
        out.unlink()  # may belong to the sim's user (uid 1234), sim/scene/ is world-writable
    run_usd_script(sim, "add_chargers.py", "/sim/generated/farm/chargers.json",
                   f"/sim/scene/{args.farm}", f"/sim/scene/{args.out}")


if __name__ == "__main__":
    main()
