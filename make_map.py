#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将 GeoDownloader 下载的多级 GeoTIFF 地图转换为 ESP32 屏幕可用的 RGB565 RAW 文件

档位（同一地理范围）：
    map1.raw  240x320    由 z16 图裁剪（总览）
    map2.raw  480x640    由 z17 图裁剪（街道级）
    map4.raw  960x1280   由 z18 图裁剪（建筑级）
    map8.raw  1920x2560  由可选 z19 图裁剪；未提供时由 z16 插值放大（会糊）

裁剪中心：默认各图中心；可用 --cx --cy 指定（tif 像素坐标，对应 z16 图）。
每次运行会输出 preview.png：z16 缩略图 + 裁剪窗口红框，方便肉眼确认位置。

用法：
    python make_map.py <z16.tif> <z17.tif> <z18.tif> [z19.tif] [--cx X --cy Y] [out_dir]

示例（想看图中某位置，先跑一次看 preview.png 再调坐标）：
    python make_map.py a.tif b.tif c.tif map_out
    python make_map.py a.tif b.tif c.tif --cx 1500 --cy 1200 map_out

依赖：pip install pillow
"""
import sys
import os
from PIL import Image, ImageDraw

def rgb565_le(r, g, b):
    v = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
    return v.to_bytes(2, "little")

def crop_centered(src_img, out_path, tw, th, cx, cy):
    W, H = src_img.size
    if W < tw or H < th:
        print(f"WARN: source {W}x{H} smaller than {tw}x{th}, resizing instead")
        win = src_img.resize((tw, th), Image.BILINEAR)
    else:
        left = max(0, min(cx - tw // 2, W - tw))
        top = max(0, min(cy - th // 2, H - th))
        win = src_img.crop((left, top, left + tw, top + th))
        print(f"Crop {tw}x{th} at ({left},{top}) center=({cx},{cy})")
    raw = bytearray()
    for r, g, b in win.getdata():
        raw += rgb565_le(r, g, b)
    with open(out_path, "wb") as f:
        f.write(raw)
    print(f"Wrote {out_path} ({tw}x{th}, {len(raw) / 1024:.0f} KB)")

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("usage: python make_map.py <z16.tif> <z17.tif> <z18.tif> [z19.tif] [--cx X --cy Y] [out_dir]")
        sys.exit(1)

    args = sys.argv[1:]
    cx = cy = None
    out_dir = "map_out"

    # 解析 --cx/--cy 和输出目录
    rest = []
    i = 0
    while i < len(args):
        if args[i] == "--cx" and i + 1 < len(args):
            cx = int(args[i + 1]); i += 2
        elif args[i] == "--cy" and i + 1 < len(args):
            cy = int(args[i + 1]); i += 2
        else:
            rest.append(args[i]); i += 1
    if rest and os.path.isdir(rest[-1]):
        out_dir = rest.pop()
    os.makedirs(out_dir, exist_ok=True)

    if len(rest) < 3:
        print("usage: python make_map.py <z16.tif> <z17.tif> <z18.tif> [z19.tif] [--cx X --cy Y] [out_dir]")
        sys.exit(1)

    tif16 = Image.open(rest[0]).convert("RGB")
    tif17 = Image.open(rest[1]).convert("RGB")
    tif18 = Image.open(rest[2]).convert("RGB")
    tif19 = Image.open(rest[3]).convert("RGB") if len(rest) >= 4 else None

    W, H = tif16.size
    if cx is None: cx = W // 2
    if cy is None: cy = H // 2
    print(f"z16 size: {W}x{H}  crop center: ({cx},{cy})")

    # 预览图：z16 缩略图 + 裁剪窗口红框
    scale = min(1.0, 600.0 / max(W, H))
    prev = tif16.resize((int(W * scale), int(H * scale)), Image.BILINEAR)
    draw = ImageDraw.Draw(prev)
    l = int((cx - 120) * scale); t = int((cy - 160) * scale)
    draw.rectangle([l, t, l + int(240 * scale), t + int(320 * scale)], outline="red", width=2)
    prev_path = os.path.join(out_dir, "preview.png")
    prev.save(prev_path)
    print(f"Preview saved: {prev_path}  (red box = 240x320 window on z16)")

    crop_centered(tif16, os.path.join(out_dir, "map1.raw"), 240, 320, cx, cy)
    crop_centered(tif17, os.path.join(out_dir, "map2.raw"), 480, 640,
                  int(cx / 2), int(cy / 2))      # z17 像素尺寸是 z16 的 2 倍
    crop_centered(tif18, os.path.join(out_dir, "map4.raw"), 960, 1280,
                  int(cx / 4), int(cy / 4))      # z18 是 z16 的 4 倍

    if tif19 is not None:
        print("map8.raw: from z19 (real resolution)")
        crop_centered(tif19, os.path.join(out_dir, "map8.raw"), 1920, 2560,
                      int(cx / 8), int(cy / 8))
    else:
        print("map8.raw: no z19, interpolating from z16 (will be blurry)")
        crop_centered(tif16, os.path.join(out_dir, "map8.raw"), 1920, 2560, cx, cy)
