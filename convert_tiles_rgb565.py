#!/usr/bin/env python3
"""Convert a z/x/y PNG tile tree to raw little-endian RGB565 files.

Input layout:
    <source>/1/0/0.png
    <source>/17/67234/...?png

Output layout is identical, with ``.rgb565`` replacing ``.png``. The
firmware reads these files directly and only falls back to PNG when a raw
file is absent.

Usage:
    python convert_tiles_rgb565.py E:\\map E:\\map_rgb565
    python convert_tiles_rgb565.py E:\\map E:\\map --overwrite

Requires Pillow (``python -m pip install pillow``).
"""
from __future__ import annotations

import argparse
from pathlib import Path

from PIL import Image


TILE_SIZE = 256


def convert_one(source: Path, target: Path, overwrite: bool) -> bool:
    if target.exists() and not overwrite:
        return False

    with Image.open(source) as image:
        image = image.convert("RGB")
        if image.size != (TILE_SIZE, TILE_SIZE):
            image = image.resize((TILE_SIZE, TILE_SIZE), Image.Resampling.NEAREST)

        pixels = image.load()
        raw = bytearray(TILE_SIZE * TILE_SIZE * 2)
        offset = 0
        for y in range(TILE_SIZE):
            for x in range(TILE_SIZE):
                red, green, blue = pixels[x, y]
                value = ((red >> 3) << 11) | ((green >> 2) << 5) | (blue >> 3)
                raw[offset] = value & 0xFF
                raw[offset + 1] = value >> 8
                offset += 2

    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(raw)
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="PNG tile root containing z/x/y.png")
    parser.add_argument("destination", type=Path, help="Output root for z/x/y.rgb565")
    parser.add_argument("--overwrite", action="store_true", help="replace existing raw tiles")
    args = parser.parse_args()

    if not args.source.is_dir():
        parser.error(f"source directory does not exist: {args.source}")

    png_files = sorted(args.source.glob("*/*/*.png"))
    if not png_files:
        parser.error(f"no z/x/y.png files found below {args.source}")

    converted = skipped = failed = 0
    for source in png_files:
        relative = source.relative_to(args.source)
        target = args.destination / relative.with_suffix(".rgb565")
        try:
            if convert_one(source, target, args.overwrite):
                converted += 1
            else:
                skipped += 1
        except Exception as exc:  # keep processing the rest of the card
            failed += 1
            print(f"ERROR {source}: {exc}")

    print(f"Converted {converted}, skipped {skipped}, failed {failed}")
    print(f"Copy the contents of {args.destination} into the SD card /map directory.")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
