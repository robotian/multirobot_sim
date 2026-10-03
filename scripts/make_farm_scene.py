#!/usr/bin/env python3
"""Build sim/scene/lavender_farm.usd, the real lavender field, from the farm database.

  scripts/make_farm_scene.py [--out lavender_farm.usd] [--dbname NAME] [--host H] [--port P]

Reads every plant of public.object_data (object_id, row_id, x_coord, y_coord: map frame = the sim's world frame)
with the connection settings of colcon_ws/src/status_server/config/config.yaml (host.docker.internal is the host,
so it is replaced by localhost here), writes them to sim/generated/farm/plants.json and runs
sim/scripts/build_farm_scene.py with Isaac Sim's USD libraries in the a300-isaac-sim image (no GPU, no Kit; the
running sim is not touched). Use the result with SIM_SCENE=lavender_farm.usd in .env or the web UI's Scene picker.
Needs psycopg and pyyaml on the host.
"""
import argparse
import datetime
import json
import subprocess
import sys
from pathlib import Path

import psycopg
import yaml

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "colcon_ws/src/status_server/config/config.yaml"
IMAGE = "a300-isaac-sim:6.0.0"
# Isaac's USD python libs live in extension folders whose names carry a build hash
USD_ENV = ('L=$(ls -d /isaac-sim/extscache/omni.usd.libs-*/ | head -1); '
           'export PYTHONPATH=$L LD_LIBRARY_PATH=$L/bin:$LD_LIBRARY_PATH; ')


def fetch_plants(args):
    db = yaml.safe_load(CONFIG.read_text())["database"]
    host = args.host or ("localhost" if db["host"] == "host.docker.internal" else db["host"])
    dbname = args.dbname or db["dbname"]
    with psycopg.connect(host=host, port=args.port or db["port"], dbname=dbname, user=db["user"],
                         password=db["password"], connect_timeout=db.get("connect_timeout", 5)) as conn:
        rows = conn.execute("SELECT object_id, row_id, x_coord, y_coord FROM public.object_data "
                            "WHERE x_coord IS NOT NULL AND y_coord IS NOT NULL "
                            "ORDER BY row_id, object_id").fetchall()
    plants = [{"object_id": r[0], "row_id": r[1] if r[1] is not None else 0, "x": r[2], "y": r[3]} for r in rows]
    stamp = datetime.datetime.now().isoformat(timespec="seconds")
    return {"source": f"{dbname}.public.object_data @ {stamp}", "plants": plants}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="lavender_farm.usd", help="file name in sim/scene/ (default %(default)s)")
    ap.add_argument("--dbname", help="database (default: status_server's config.yaml)")
    ap.add_argument("--host", help="database host (default: localhost)")
    ap.add_argument("--port", type=int, help="database port (default: status_server's config.yaml)")
    ap.add_argument("--sim-dir", type=Path, default=ROOT / "sim",
                    help="the sim folder: assets in, scene out (default: this checkout's, %(default)s)")
    args = ap.parse_args()

    data = fetch_plants(args)
    if not data["plants"]:
        sys.exit("object_data has no plants")
    sim = args.sim_dir.resolve()
    farm_dir = sim / "generated/farm"
    farm_dir.mkdir(parents=True, exist_ok=True)
    (farm_dir / "plants.json").write_text(json.dumps(data, indent=1))
    print(f"{len(data['plants'])} plants from {data['source']}")

    out = sim / "scene" / args.out
    if out.exists():
        out.unlink()  # may belong to the sim's user (uid 1234), sim/scene/ is world-writable
    # as the image's user (uid 1234, like the sim): /isaac-sim is not readable by others
    cmd = ["docker", "run", "--rm", "--entrypoint", "bash", "-v", f"{sim}:/sim",
           "-v", f"{ROOT / 'sim/scripts/build_farm_scene.py'}:/build_farm_scene.py:ro", IMAGE, "-c",
           USD_ENV + f"exec /isaac-sim/python.sh /build_farm_scene.py "
                     f"/sim/generated/farm/plants.json /sim/scene/{args.out}"]
    subprocess.run(cmd, check=True)


if __name__ == "__main__":
    main()
