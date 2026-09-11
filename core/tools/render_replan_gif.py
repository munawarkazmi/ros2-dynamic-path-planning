#!/usr/bin/env python3
"""Animate the replan figure from the same committed scenario dump.

Usage: render_replan_gif.py <map.png> <scenario_dump.txt> <out.gif>

Every element is read from the dump written by core/tools/dump_scenario,
which is one real run: the original 919-cell path, the position the robot
had reached when the obstacle appeared, the 317 obstacle cells, and the
613-cell path D* Lite repaired from that position to the same goal.

What the animation adds is the sweep of a marker along those paths, which
is a way of reading committed coordinates in order and not a simulated
controller. No pose here was integrated, and no timing is claimed: the
marker advances a fixed number of cells per frame. The claim the figure
makes is the one the still makes, that the repair reaches the same goal
around the obstacle, with the moment of the repair shown rather than
described.
"""
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image

sys.path.insert(0, str(__file__.rsplit("/", 1)[0]))
from render_figures import BLUE, GOLD, GRAY, NAVY, RED, load_dump, scale_bar

# One shared palette, written out rather than chosen. Letting the quantiser
# pick from a frame spends every slot on the floor plan's greys, because the
# coloured pixels are a fraction of a per cent of the image, and the robot,
# the goal and the repaired path all come back grey.
def shared_palette():
    colours = [(255, 255, 255), (0, 0, 0)]
    colours += [tuple(int(c[i:i + 2], 16) for i in (1, 3, 5))
                for c in (NAVY, BLUE, GOLD, RED, GRAY)]
    colours += [(v, v, v) for v in range(8, 256, 10)]
    flat = [channel for colour in colours for channel in colour]
    flat += [0] * (768 - len(flat))
    palette = Image.new("P", (1, 1))
    palette.putpalette(flat)
    return palette


def map_axes(ax, im):
    # render_figures draws the map at print size with nearest-neighbour, which
    # is right for a PNG at 110 dpi. These frames are rendered at their final
    # width instead, where nearest turns the wall hatching into moire, so the
    # map is resampled properly on the way down.
    ax.imshow(im, cmap="gray", interpolation="antialiased")
    ax.set_xticks([])
    ax.set_yticks([])

FRAMES_BEFORE = 16   # robot walking the original route
FRAMES_HELD = 6      # the obstacle is there and the route has not changed yet
FRAMES_AFTER = 30    # robot walking the repaired route


def frame(im, s, travelled, ahead, ahead_style, show_obstacle, caption):
    # Rendered at the width it is published at, so nothing is resized later.
    fig, ax = plt.subplots(figsize=(4.8, 6.9), dpi=100)
    map_axes(ax, im)
    if ahead:
        ax.plot([p[0] for p in ahead], [p[1] for p in ahead],
                c=ahead_style[0], lw=ahead_style[1], ls=ahead_style[2])
    if travelled:
        ax.plot([p[0] for p in travelled], [p[1] for p in travelled],
                c=NAVY, lw=2.4)
    if show_obstacle:
        ob = s["obstacle"]
        ax.scatter([p[0] for p in ob], [p[1] for p in ob], s=1.5, c=RED,
                   linewidths=0)
    rx, ry = travelled[-1] if travelled else s["start"][0]
    gx, gy = s["goal"][0]
    ax.scatter([gx], [gy], c=RED, s=90, zorder=5)
    ax.scatter([rx], [ry], c=GOLD, s=110, zorder=6, edgecolors=NAVY,
               linewidths=1.2)
    ax.set_title(caption, fontsize=11, color=NAVY)
    scale_bar(ax, im.width, im.height)
    fig.tight_layout()
    fig.canvas.draw()
    # buffer_rgba rather than tostring_rgb, which matplotlib 3.10 removed.
    out = Image.frombytes("RGBA", fig.canvas.get_width_height(),
                          fig.canvas.buffer_rgba()).convert("RGB")
    plt.close(fig)
    return out


def main():
    if len(sys.argv) != 4:
        raise SystemExit(__doc__)
    map_png, dump_path, out_path = sys.argv[1:]
    im = Image.open(map_png).convert("L")
    s = load_dump(dump_path)

    old, new, pos = s["old_path"], s["repaired_path"], s["pos"][0]
    split = old.index(pos)

    frames = []
    for i in range(1, FRAMES_BEFORE + 1):
        k = max(2, round(split * i / FRAMES_BEFORE))
        frames.append(frame(im, s, old[:k], old[k:], (GRAY, 1.8, "--"), False,
                            "A* planned this route across the building"))
    for _ in range(FRAMES_HELD):
        frames.append(frame(im, s, old[:split + 1], old[split:],
                            (GRAY, 1.8, "--"), True,
                            "Someone steps into the route"))
    for i in range(1, FRAMES_AFTER + 1):
        k = max(2, round(len(new) * i / FRAMES_AFTER))
        frames.append(frame(im, s, old[:split + 1] + new[:k], new[k:],
                            (NAVY, 2.2, "-"), True,
                            "D* Lite repairs the same journey, reusing its search"))

    palette = shared_palette()
    frames = [f.quantize(palette=palette, dither=Image.NONE) for f in frames]
    frames[0].save(out_path, save_all=True, append_images=frames[1:],
                   duration=[110] * FRAMES_BEFORE + [500] * FRAMES_HELD
                   + [110] * FRAMES_AFTER, loop=0, optimize=True)
    print(f"{out_path}: {len(frames)} frames, {frames[0].size[0]}x{frames[0].size[1]}")


if __name__ == "__main__":
    main()
