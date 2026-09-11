"""
make_icons.py - 生成 resources/ 目录下的 16x16, 32x32, 64x64 PNG 图标
使用纯 Python 标准库 (struct + zlib)，无需依赖第三方图像库。
"""

import os
import struct
import zlib
import math


def create_png(width: int, height: int, pixels: list) -> bytes:
    """
    pixels: list of rows, each row is a list of (r, g, b, a) tuples.
    """
    def chunk(tag: bytes, data: bytes) -> bytes:
        crc = zlib.crc32(tag + data) & 0xFFFFFFFF
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", crc)

    # PNG Signature
    png_sig = b"\x89PNG\r\n\x1a\n"

    # IHDR: width, height, bit_depth=8, color_type=6 (RGBA), comp=0, filter=0, interlace=0
    ihdr_data = struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0)
    ihdr = chunk(b"IHDR", ihdr_data)

    # IDAT
    raw_data = bytearray()
    for row in pixels:
        raw_data.append(0)  # Filter byte: None
        for r, g, b, a in row:
            raw_data.extend([r, g, b, a])

    compressed = zlib.compress(bytes(raw_data), level=9)
    idat = chunk(b"IDAT", compressed)

    # IEND
    iend = chunk(b"IEND", b"")

    return png_sig + ihdr + idat + iend


def draw_worm_gear_icon(size: int) -> list:
    """绘制齿轮与蜗杆符号图标"""
    pixels = []
    cx = size * 0.45
    cy = size * 0.55
    r_outer = size * 0.36
    r_root = size * 0.26
    r_hole = size * 0.10

    # 齿数
    n_teeth = 8

    # 蜗杆参数 (右上方倾斜柱与螺旋纹)
    wx0, wy0 = size * 0.70, size * 0.15
    wx1, wy1 = size * 0.90, size * 0.75
    w_width = size * 0.16

    for y in range(size):
        row = []
        for x in range(size):
            dx = x - cx
            dy = y - cy
            dist = math.hypot(dx, dy)
            angle = math.atan2(dy, dx)

            # 背景透明
            r, g, b, a = 0, 0, 0, 0

            # 1. 齿轮绘制 (外齿圆 + 轮芯)
            tooth_mod = (math.cos(angle * n_teeth) + 1.0) / 2.0
            r_limit = r_root + (r_outer - r_root) * tooth_mod

            if r_hole <= dist <= r_limit:
                # 齿轮青色/机械灰渐变
                t = (x + y) / (2.0 * size)
                r = int(40 + 60 * t)
                g = int(120 + 70 * t)
                b = int(190 + 50 * t)
                a = 255
            elif dist < r_hole:
                # 轴孔周围深灰内圈
                if dist > r_hole * 0.6:
                    r, g, b, a = 20, 60, 100, 200

            # 2. 蜗杆绘制 (黄色/铜色柱体与螺旋刻线)
            # 点到线段 (wx0, wy0) -> (wx1, wy1) 的距离
            seg_dx = wx1 - wx0
            seg_dy = wy1 - wy0
            seg_len = math.hypot(seg_dx, seg_dy)
            if seg_len > 1e-4:
                nx = seg_dx / seg_len
                ny = seg_dy / seg_len
                # 投影长度
                proj = (x - wx0) * nx + (y - wy0) * ny
                # 垂向距离
                perp = abs(-(x - wx0) * ny + (y - wy0) * nx)

                if 0 <= proj <= seg_len and perp <= w_width / 2.0:
                    # 铜色/金属色
                    t_worm = (proj / seg_len) * 4.0 * math.pi
                    stripe = (math.sin(t_worm) + 1.0) / 2.0
                    r = int(210 + 40 * stripe)
                    g = int(140 + 50 * stripe)
                    b = int(40 + 20 * stripe)
                    a = 255

            row.append((r, g, b, a))
        pixels.append(row)

    return pixels


def main():
    res_dir = os.path.join(os.path.dirname(__file__), "resources")
    os.makedirs(res_dir, exist_ok=True)

    for sz in [16, 32, 64]:
        px = draw_worm_gear_icon(sz)
        png_bytes = create_png(sz, sz, px)
        filepath = os.path.join(res_dir, f"{sz}x{sz}.png")
        with open(filepath, "wb") as f:
            f.write(png_bytes)
        print(f"Generated: {filepath} ({len(png_bytes)} bytes)")


if __name__ == "__main__":
    main()
