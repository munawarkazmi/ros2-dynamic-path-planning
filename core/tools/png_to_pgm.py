#!/usr/bin/env python3
"""Convert a map PNG to a binary PGM (P5) for the benchmark loader.

Keeps the benchmark free of image-library dependencies: PGM is trivial
to parse in C++. Pixels are written as 8-bit grayscale luminance.

Usage: png_to_pgm.py <in.png> <out.pgm>
"""
import sys

from PIL import Image


def main() -> None:
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    src, dst = sys.argv[1], sys.argv[2]
    im = Image.open(src).convert("L")
    w, h = im.size
    with open(dst, "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode())
        f.write(im.tobytes())
    print(f"{src} ({w}x{h}) -> {dst}")


if __name__ == "__main__":
    main()
