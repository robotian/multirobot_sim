#!/usr/bin/env python3
"""Write the weed barrier's tiling textures into sim/assets/weed_barrier/ (used by build_farm_scene.py).

  sim/scripts/make_weed_barrier_textures.py [out_dir]

Modelled on a photo of woven polypropylene landscape fabric: a plain weave of flat dark tapes (~2.3 mm), and a
green zig-zag guide line along the roll. One tile is TILE_M square, the roll direction along the image's x (u);
the guide line runs along the tile's middle row, so build_farm_scene.py puts it at v = 0.5 for a line every
TILE_M across the strip. Deterministic (fixed seed). Needs numpy and Pillow (host python is fine).
albedo.png (sRGB), normal.png (tangent space, +Y up = +v), roughness.png (linear, grey).
"""
import os
import sys

import numpy as np
from PIL import Image

TILE_M = 0.3  # also BARRIER_TILE_M in build_farm_scene.py
PX = 1024
TAPES = 128  # tapes per tile and direction: 0.3 m / 128 = 2.3 mm
TAPE_RGB = (52, 52, 54)  # sRGB, mid-tone of a lit tape in the photo
GAP_DARK = 0.45  # brightness in the gaps between tapes
LINE_RGB = (110, 205, 70)
LINE_WIDTH_M = 0.0018
LINE_AMPLITUDE_M = 0.0016
LINE_PERIOD_M = 0.006


def main(out):
    os.makedirs(out, exist_ok=True)
    rng = np.random.default_rng(7)
    cell = PX / TAPES
    y, x = np.mgrid[0:PX, 0:PX].astype(np.float32) + 0.5
    i, j = (x // cell).astype(int), (y // cell).astype(int)  # tape across x (weft) / along x (warp)
    fx, fy = (x % cell) / cell, (y % cell) / cell  # position inside the cell, 0..1
    warp_on_top = (i + j) % 2 == 0  # plain weave
    # the visible tape runs along x (warp) or along y (weft); its cross-section profile is a flattened arch
    across = np.where(warp_on_top, fy, fx)
    along = np.where(warp_on_top, fx, fy)
    arch = np.clip(np.sin(np.pi * across), 0, 1) ** 0.35
    dip = 0.75 + 0.25 * np.sin(np.pi * along)  # tape dives under its neighbour at the cell's ends
    height = arch * dip
    # per-tape brightness (extruded tapes differ a little) and fine noise
    warp_var = rng.normal(1.0, 0.06, TAPES)[j]
    weft_var = rng.normal(1.0, 0.06, TAPES)[i]
    tape_var = np.where(warp_on_top, warp_var, weft_var)
    noise = 1.0 + rng.normal(0.0, 0.03, (PX, PX)).astype(np.float32)
    shade = (GAP_DARK + (1 - GAP_DARK) * height) * tape_var * noise
    rgb = np.array(TAPE_RGB, np.float32)[None, None, :] * shade[..., None]

    # green guide line along x at the tile's middle row, a zig-zag stitch
    m_per_px = TILE_M / PX
    zig = LINE_AMPLITUDE_M * (2 * np.abs(2 * ((x * m_per_px / LINE_PERIOD_M) % 1.0) - 1) - 1)
    d = np.abs((y - PX / 2) * m_per_px - zig)
    line = np.clip(1.0 - (d - LINE_WIDTH_M / 2) / m_per_px, 0, 1)  # 1 px anti-aliased edge
    line_rgb = np.array(LINE_RGB, np.float32)[None, None, :] * (0.7 + 0.3 * height)[..., None]
    rgb = rgb * (1 - line[..., None]) + line_rgb * line[..., None]
    Image.fromarray(np.clip(rgb, 0, 255).astype(np.uint8), "RGB").save(f"{out}/albedo.png", optimize=True)

    # normal map from the height field (tapes ~0.3 mm proud), tangent space: x = +u, y = +v (image rows go -v)
    h_m = 0.0003 * height
    gx = (np.roll(h_m, -1, 1) - np.roll(h_m, 1, 1)) / (2 * m_per_px)
    gy = -(np.roll(h_m, -1, 0) - np.roll(h_m, 1, 0)) / (2 * m_per_px)
    n = np.stack([-gx, -gy, np.ones_like(gx)], -1)
    n /= np.linalg.norm(n, axis=-1, keepdims=True)
    Image.fromarray(((n * 0.5 + 0.5) * 255).round().astype(np.uint8), "RGB").save(f"{out}/normal.png", optimize=True)

    # tape tops a little glossier than the gaps; the guide line is matte thread
    rough = 0.9 - 0.25 * height + rng.normal(0.0, 0.02, (PX, PX))
    rough = rough * (1 - line) + 0.85 * line
    Image.fromarray((np.clip(rough, 0, 1) * 255).astype(np.uint8), "L").save(f"{out}/roughness.png", optimize=True)
    print(f"wrote {out}/albedo.png, normal.png, roughness.png ({PX} px = {TILE_M} m)")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else
         os.path.join(os.path.dirname(os.path.abspath(__file__)), "../assets/weed_barrier"))
