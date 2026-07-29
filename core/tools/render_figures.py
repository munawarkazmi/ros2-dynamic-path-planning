#!/usr/bin/env python3
"""Render the README figures from real algorithm output.

Inputs:
  - the map PNG,
  - the scenario dump written by core/tools/dump_scenario (real A* and
    D* Lite runs),
  - the committed benchmark CSV (real 200-trial run).

Usage: render_figures.py <map.png> <scenario_dump.txt> <benchmark.csv> <outdir>
"""
import csv
import statistics
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image

NAVY = "#123a5f"
BLUE = "#9db8d2"
GOLD = "#d9a441"
RED = "#b03a2e"
GRAY = "#8a8f98"


def load_dump(path):
    sections = {}
    name = None
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("SECTION "):
                name = line.split(" ", 1)[1]
                sections[name] = []
            else:
                x, y = line.split()
                sections[name].append((int(x), int(y)))
    return sections


def map_axes(ax, im):
    ax.imshow(im, cmap="gray", interpolation="nearest")
    ax.set_xticks([])
    ax.set_yticks([])


def scale_bar(ax, w, h):
    # 5 m = 100 cells at 0.05 m/cell
    x0, y0 = w * 0.06, h * 0.96
    ax.plot([x0, x0 + 100], [y0, y0], color=NAVY, lw=3)
    ax.text(x0 + 50, y0 - 12, "5 m", color=NAVY, ha="center", fontsize=10)


def fig_astar(im, s, out):
    fig, ax = plt.subplots(figsize=(7, 10))
    map_axes(ax, im)
    ex = s["expanded"]
    ax.scatter([p[0] for p in ex], [p[1] for p in ex], s=0.3, c=BLUE,
               alpha=0.35, linewidths=0, label="expanded nodes", rasterized=True)
    path = s["astar_path"]
    ax.plot([p[0] for p in path], [p[1] for p in path], c=NAVY, lw=2.2,
            label="planned path")
    sx, sy = s["start"][0]
    gx, gy = s["goal"][0]
    ax.scatter([sx], [sy], c=GOLD, s=90, zorder=5)
    ax.scatter([gx], [gy], c=RED, s=90, zorder=5)
    ax.annotate("start", (sx, sy), textcoords="offset points", xytext=(10, 4),
                color=NAVY, fontsize=11, fontweight="bold")
    ax.annotate("goal", (gx, gy), textcoords="offset points", xytext=(-38, 4),
                color=NAVY, fontsize=11, fontweight="bold")
    ax.legend(loc="upper right", fontsize=9)
    ax.set_title(f"A* on the repository map - real run "
                 f"({len(ex):,} nodes expanded)", fontsize=11, color=NAVY)
    scale_bar(ax, im.width, im.height)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


def fig_replan(im, s, out):
    fig, ax = plt.subplots(figsize=(7, 10))
    map_axes(ax, im)
    old = s["old_path"]
    new = s["repaired_path"]
    ax.plot([p[0] for p in old], [p[1] for p in old], c=GRAY, lw=1.8, ls="--",
            label="original path")
    ax.plot([p[0] for p in new], [p[1] for p in new], c=NAVY, lw=2.2,
            label="repaired path")
    ob = s["obstacle"]
    ax.scatter([p[0] for p in ob], [p[1] for p in ob], s=1.5, c=RED,
               linewidths=0, label="new obstacle")
    px, py = s["pos"][0]
    gx, gy = s["goal"][0]
    ax.scatter([px], [py], c=GOLD, s=90, zorder=5)
    ax.scatter([gx], [gy], c=RED, s=90, zorder=5)
    ax.annotate("robot", (px, py), textcoords="offset points", xytext=(10, 4),
                color=NAVY, fontsize=11, fontweight="bold")
    ax.legend(loc="upper right", fontsize=9)
    ax.set_title("D* Lite incremental repair after a path-blocking obstacle - real run",
                 fontsize=11, color=NAVY)
    scale_bar(ax, im.width, im.height)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


def fig_benchmark(csv_path, out):
    a_ms, d_ms = [], []
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            if row["event"] == "0" or row["cost_match"] != "1":
                continue
            if float(row["astar_cost_cells"]) < 0 or float(row["dstar_cost_cells"]) < 0:
                continue
            a_ms.append(float(row["astar_ms"]))
            d_ms.append(float(row["dstar_ms"]))

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 5))

    bins = [0.03, 0.1, 0.3, 1, 3, 10, 30, 100, 300]
    ax1.hist(a_ms, bins=bins, alpha=0.6, color=GRAY, label="A* (from scratch)")
    ax1.hist(d_ms, bins=bins, alpha=0.6, color=NAVY, label="D* Lite (incremental)")
    ax1.set_xscale("log")
    ax1.set_xlabel("replan time (ms, log scale)")
    ax1.set_ylabel("replan events")
    ax1.legend(fontsize=9)
    ax1.set_title(f"Replan time distribution ({len(a_ms)} events)", fontsize=11)

    labels = ["mean", "median"]
    a_stats = [statistics.mean(a_ms), statistics.median(a_ms)]
    d_stats = [statistics.mean(d_ms), statistics.median(d_ms)]
    xpos = range(len(labels))
    width = 0.38
    ax2.bar([x - width / 2 for x in xpos], a_stats, width, color=GRAY,
            label="A* (from scratch)")
    ax2.bar([x + width / 2 for x in xpos], d_stats, width, color=NAVY,
            label="D* Lite (incremental)")
    for x, v in zip(xpos, a_stats):
        ax2.text(x - width / 2, v, f"{v:.2f}", ha="center", va="bottom", fontsize=9)
    for x, v in zip(xpos, d_stats):
        ax2.text(x + width / 2, v, f"{v:.2f}", ha="center", va="bottom", fontsize=9)
    ax2.set_xticks(list(xpos), labels)
    ax2.set_ylabel("replan time (ms)")
    ax2.legend(fontsize=9)
    speedup = a_stats[0] / d_stats[0]
    wins = sum(1 for a, d in zip(a_ms, d_ms) if d < a)
    ax2.set_title(f"Seed 42, 200 trials: D* Lite {speedup:.1f}x faster on average,\n"
                  f"faster on {wins}/{len(a_ms)} events", fontsize=11)

    fig.suptitle("Replanning after a path-blocking obstacle - measured, not modeled",
                 fontsize=12, color=NAVY)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


def main():
    if len(sys.argv) != 5:
        sys.exit(__doc__)
    map_png, dump_txt, bench_csv, outdir = sys.argv[1:]
    im = Image.open(map_png).convert("L").point(lambda v: 255 if v > 127 else 40)
    s = load_dump(dump_txt)
    fig_astar(im, s, f"{outdir}/astar_plan.png")
    fig_replan(im, s, f"{outdir}/replan_obstacle.png")
    fig_benchmark(bench_csv, f"{outdir}/benchmark_summary.png")
    print("figures written")


if __name__ == "__main__":
    main()
