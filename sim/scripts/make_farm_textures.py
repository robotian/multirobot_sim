#!/usr/bin/env python3
"""Write the farm scene's tiling textures into sim/assets/ (used by build_farm_scene.py).

  sim/scripts/make_farm_textures.py [assets_dir]

weed_barrier/: woven polypropylene landscape fabric, modelled on a photo of it: a plain weave of flat dark tapes
  (~2.3 mm) with green zig-zag guide lines along the roll, sparsely dusted with sand/dirt, more of it towards the
  strip's edges (where the soil blows on). One tile is BARRIER_TILE_M square = the strip's whole width across
  (v), so the dirt can depend on the distance to the edge; the roll direction is the image's x (u). Guide lines at
  v = 1/6, 1/2, 5/6, i.e. on the strip's centre line and 0.3 m either side.
soil/: dark, grainy sandy soil for the ground: sand grains, small pebbles, low-frequency mottling (damp/dry).
Each folder: albedo.jpg (sRGB), normal.png (tangent space, +Y = +v), roughness.jpg (linear, grey).
Every noise is periodic (filtered in the Fourier domain), so the tiles have no seams. Deterministic (fixed
seeds). Needs numpy and Pillow (host python is fine).
"""
import os
import sys

import numpy as np
from PIL import Image

BARRIER_TILE_M = 0.9  # build_farm_scene.BARRIER_WIDTH and BARRIER_TILE_M
BARRIER_PX = 2048  # 0.44 mm/px
TAPE_M = 0.0023
TAPE_RGB = (52, 52, 54)  # sRGB, mid-tone of a lit tape in the photo
GAP_DARK = 0.45  # brightness in the gaps between tapes
LINE_RGB = (110, 205, 70)
LINE_V = (1 / 6, 1 / 2, 5 / 6)
LINE_WIDTH_M = 0.0018
LINE_AMPLITUDE_M = 0.0016
LINE_PERIOD_M = 0.006
DIRT_RGB = (104, 94, 82)  # dry sandy soil on the fabric, greyer and a bit lighter than the damp ground
DIRT_COVER = 0.05  # fraction of the strip's centre under solid dirt
DIRT_EDGE_COVER = 0.22  # ... and at its edges
DUST = 0.22  # opacity of the thin dust film at its thickest

SOIL_TILE_M = 3.0  # build_farm_scene.SOIL_TILE_M
SOIL_PX = 2048  # 1.5 mm/px
SOIL_RGB = (74, 62, 50)  # sRGB, dark sandy soil


def periodic_noise(rng, n, lo, hi, power=1.0):
    """Tileable n x n noise (zero mean, unit std) keeping spatial frequencies lo..hi cycles per tile, amplitude
    ~ 1/f^power inside the band."""
    f = np.fft.fftfreq(n) * n
    fr = np.hypot(*np.meshgrid(f, f, indexing="ij"))
    spec = np.fft.fft2(rng.normal(size=(n, n)))
    band = ((fr >= lo) & (fr <= hi)) / np.maximum(fr, 1.0) ** power
    out = np.real(np.fft.ifft2(spec * band))
    return (out - out.mean()) / (out.std() + 1e-12)


def cover_mask(noise, fraction, softness=0.15):
    """Soft 0..1 mask covering about `fraction` of the pixels where `noise` is highest."""
    th = np.quantile(noise, 1 - np.clip(fraction, 1e-3, 1 - 1e-3))
    return np.clip((noise - th) / softness + 0.5, 0, 1)


def normal_map(height_m, m_per_px):
    gx = (np.roll(height_m, -1, 1) - np.roll(height_m, 1, 1)) / (2 * m_per_px)
    gy = -(np.roll(height_m, -1, 0) - np.roll(height_m, 1, 0)) / (2 * m_per_px)  # image rows go -v
    n = np.stack([-gx, -gy, np.ones_like(gx)], -1)
    n /= np.linalg.norm(n, axis=-1, keepdims=True)
    return ((n * 0.5 + 0.5) * 255).round().astype(np.uint8)


def save(out, albedo, normal, rough):
    os.makedirs(out, exist_ok=True)
    for old in ("albedo.png", "roughness.png"):  # earlier versions' names
        if os.path.exists(f"{out}/{old}"):
            os.remove(f"{out}/{old}")
    # JPEG for the noisy colour/roughness (PNG was ~4x the size), PNG for the normals
    Image.fromarray(np.clip(albedo, 0, 255).astype(np.uint8), "RGB").save(f"{out}/albedo.jpg", quality=92)
    Image.fromarray(normal, "RGB").save(f"{out}/normal.png", optimize=True)
    Image.fromarray((np.clip(rough, 0, 1) * 255).astype(np.uint8), "L").save(f"{out}/roughness.jpg", quality=92)
    print(f"wrote {out}/albedo.jpg, normal.png, roughness.jpg")


def sand(rng, n, m_per_px, rgb, grain_amp=0.22):
    """Grainy sand colour field (n x n x 3) and its height (m): per-grain brightness (~2-6 px grains plus per-pixel
    noise), a few dark and light grains, faint hue variation."""
    grain = periodic_noise(rng, n, n / 6, n / 2, 0.0)
    pixel = rng.normal(0.0, 1.0, (n, n))
    speck = rng.random((n, n))
    hue = periodic_noise(rng, n, 2, 24, 1.0)
    shade = 1.0 + grain_amp * grain + 0.08 * pixel
    shade = np.where(speck < 0.03, shade * 0.5, shade)  # dark grains / organic bits
    shade = np.where(speck > 0.98, shade * 1.5, shade)  # light quartz grains
    tint = np.stack([1.0 + 0.03 * hue, np.ones_like(hue), 1.0 - 0.04 * hue], -1)
    col = np.array(rgb, np.float32) * shade[..., None] * tint
    height = 0.0004 * grain
    return col, height


def weed_barrier(out):
    rng = np.random.default_rng(7)
    n = BARRIER_PX
    m_per_px = BARRIER_TILE_M / n
    tapes = round(BARRIER_TILE_M / TAPE_M)
    cell = n / tapes
    y, x = np.mgrid[0:n, 0:n].astype(np.float32) + 0.5
    i, j = (x // cell).astype(int), (y // cell).astype(int)
    fx, fy = (x % cell) / cell, (y % cell) / cell
    warp_on_top = (i + j) % 2 == 0  # plain weave
    across = np.where(warp_on_top, fy, fx)
    along = np.where(warp_on_top, fx, fy)
    height = np.clip(np.sin(np.pi * across), 0, 1) ** 0.35 * (0.75 + 0.25 * np.sin(np.pi * along))
    tape_var = np.where(warp_on_top, rng.normal(1.0, 0.06, tapes + 1)[j], rng.normal(1.0, 0.06, tapes + 1)[i])
    shade = (GAP_DARK + (1 - GAP_DARK) * height) * tape_var * (1.0 + rng.normal(0.0, 0.03, (n, n)))
    rgb = np.array(TAPE_RGB, np.float32)[None, None, :] * shade[..., None]

    # green guide lines along x (u)
    line = np.zeros((n, n), np.float32)
    zig = LINE_AMPLITUDE_M * (2 * np.abs(2 * ((x * m_per_px / LINE_PERIOD_M) % 1.0) - 1) - 1)
    for v in LINE_V:
        d = np.abs((y - v * n) * m_per_px - zig)
        line = np.maximum(line, np.clip(1.0 - (d - LINE_WIDTH_M / 2) / m_per_px, 0, 1))
    line_rgb = np.array(LINE_RGB, np.float32)[None, None, :] * (0.7 + 0.3 * height)[..., None]
    rgb = rgb * (1 - line[..., None]) + line_rgb * line[..., None]

    # sand/dirt, sparse: solid patches (a few cm, ragged grainy rims) more frequent towards the strip's edges
    # (v = 0 and 1), a thin translucent dust film in larger areas, and loose grains everywhere
    edge = np.abs(y / n - 0.5) * 2  # 0 centre .. 1 edge
    patches = 0.6 * periodic_noise(rng, n, 6, 60, 1.0) + 0.4 * periodic_noise(rng, n, 60, 400, 0.3)
    th = np.quantile(patches, 1 - DIRT_COVER)
    th = th + (np.quantile(patches, 1 - DIRT_EDGE_COVER) - th) * edge ** 2  # lower threshold at the edges
    solid = np.clip((patches - th) / 0.35, 0, 1) ** 0.5
    film = DUST * np.clip(periodic_noise(rng, n, 2, 20, 1.0) * 0.5 + 0.2 + 0.3 * edge, 0, 1)
    loose = (rng.random((n, n)) < 0.02 + 0.04 * edge).astype(np.float32) * 0.85
    dirt = np.clip(np.maximum(np.maximum(solid, film), loose), 0, 1)
    dcol, dheight = sand(rng, n, m_per_px, DIRT_RGB)
    rgb = rgb * (1 - dirt[..., None]) + dcol * dirt[..., None]

    h = 0.0003 * height * (1 - dirt) + (0.0003 + dheight) * dirt
    rough = (0.9 - 0.25 * height + rng.normal(0.0, 0.02, (n, n))) * (1 - line) + 0.85 * line
    rough = rough * (1 - dirt) + 0.97 * dirt
    save(out, rgb, normal_map(h, m_per_px), rough)
    print(f"  weed barrier: {BARRIER_PX} px = {BARRIER_TILE_M} m, solid dirt {np.mean(solid > 0.5):.0%} "
          f"(centre third {np.mean(solid[n // 3: 2 * n // 3] > 0.5):.0%}), mean dirt opacity {dirt.mean():.0%}")


def soil(out):
    rng = np.random.default_rng(11)
    n = SOIL_PX
    m_per_px = SOIL_TILE_M / n
    col, height = sand(rng, n, m_per_px, SOIL_RGB, grain_amp=0.32)
    # subtle damp (darker) areas, and small pebbles (~5-15 mm), clustered a little
    damp = np.clip(periodic_noise(rng, n, 2, 16, 1.0) * 0.5 + 0.5, 0, 1)
    col *= (1 - 0.12 * damp)[..., None]
    peb = periodic_noise(rng, n, n / 10, n / 4, 0.0) + 0.5 * periodic_noise(rng, n, 4, 40, 0.5)
    peb_mask = np.clip((peb - np.quantile(peb, 0.985)) / 0.4, 0, 1) ** 0.5
    peb_shade = 0.8 + 0.5 * np.clip(periodic_noise(rng, n, n / 10, n / 4, 0.0) * 0.5 + 0.5, 0, 1)
    peb_col = np.array((100, 92, 84), np.float32)[None, None, :] * peb_shade[..., None]
    col = col * (1 - peb_mask[..., None]) + peb_col * peb_mask[..., None]
    height = height + 0.003 * peb_mask
    rough = 0.93 - 0.06 * damp + 0.02 * rng.normal(size=(n, n)) - 0.15 * peb_mask
    save(out, col, normal_map(height, m_per_px), rough)
    print(f"  soil: {SOIL_PX} px = {SOIL_TILE_M} m, pebbles {np.mean(peb_mask > 0.5):.1%}")


if __name__ == "__main__":
    assets = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                 "../assets")
    weed_barrier(os.path.join(assets, "weed_barrier"))
    soil(os.path.join(assets, "soil"))
