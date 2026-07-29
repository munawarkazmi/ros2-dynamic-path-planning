// core/tools/dump_scenario.cpp
//
// Runs one real planning scenario on the repository map and dumps the
// data the documentation figures are rendered from, so every figure in
// the README is actual algorithm output:
//   1. A* plan start -> goal: expanded cells + path.
//   2. A path-blocking obstacle appears; D* Lite (which planned the
//      same route) repairs incrementally: obstacle cells + both paths.
//
// Output is plain text blocks consumed by core/tools/render_figures.py:
//   SECTION <name>
//   x y
//   ...
#include <cstdio>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

#include "planning/astar.hpp"
#include "planning/dstar_lite.hpp"
#include "planning/grid.hpp"

using planning::AStarPlanner;
using planning::Cell;
using planning::DStarLitePlanner;
using planning::Grid;
using planning::Path;

namespace {

// Same binarizing loader as the benchmark (see benchmark_main.cpp).
std::optional<Grid> loadPgm(const std::string& path, double occupied_thresh) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return std::nullopt;
  std::string magic;
  f >> magic;
  if (magic != "P5") return std::nullopt;
  auto skipWs = [&] {
    while (true) {
      int ch = f.peek();
      if (ch == '#') {
        std::string line;
        std::getline(f, line);
      } else if (std::isspace(ch)) {
        f.get();
      } else {
        break;
      }
    }
  };
  std::size_t w = 0, h = 0;
  int maxval = 0;
  skipWs(); f >> w;
  skipWs(); f >> h;
  skipWs(); f >> maxval;
  f.get();
  if (w == 0 || h == 0 || maxval != 255) return std::nullopt;
  std::vector<std::uint8_t> pixels(w * h);
  f.read(reinterpret_cast<char*>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
  if (!f) return std::nullopt;
  Grid grid(w, h, 0);
  for (std::size_t y = 0; y < h; ++y) {
    for (std::size_t x = 0; x < w; ++x) {
      const double p = (255.0 - pixels[y * w + x]) / 255.0;
      grid.setCost(x, y, p > occupied_thresh ? planning::kLethal : std::uint8_t{0});
    }
  }
  return grid;
}

void dumpCells(std::FILE* f, const char* name, const std::vector<Cell>& cells) {
  std::fprintf(f, "SECTION %s\n", name);
  for (const auto& [x, y] : cells) std::fprintf(f, "%zu %zu\n", x, y);
}

}  // namespace

int main(int argc, char** argv) {
  const std::string map = argc > 1 ? argv[1] : "maps/indoor_grid.pgm";
  const std::string out = argc > 2 ? argv[2] : "docs/figures/scenario_dump.txt";

  auto grid_opt = loadPgm(map, 0.5);
  if (!grid_opt) {
    std::fprintf(stderr, "failed to load %s\n", map.c_str());
    return 2;
  }
  Grid grid = *grid_opt;

  // A long diagonal route across the building; chosen from free space
  // on the committed map.
  const Cell start{120, 140};
  const Cell goal{600, 1000};
  if (!grid.traversable(start.first, start.second) ||
      !grid.traversable(goal.first, goal.second)) {
    std::fprintf(stderr, "start/goal not free on this map\n");
    return 2;
  }

  AStarPlanner astar;
  const auto a_path = astar.plan(grid, start.first, start.second, goal.first, goal.second);
  if (!a_path) {
    std::fprintf(stderr, "no initial path\n");
    return 2;
  }
  const auto expanded = astar.expandedCells(grid);

  DStarLitePlanner dstar(grid);
  dstar.setGoal(goal.first, goal.second);
  const auto d_path0 = dstar.plan(start.first, start.second);
  if (!d_path0) {
    std::fprintf(stderr, "D* found no initial path\n");
    return 2;
  }

  // Robot advances along the D* path; an obstacle lands ahead on it.
  const std::size_t pos_idx = d_path0->size() / 3;
  const Cell pos = (*d_path0)[pos_idx];
  const Cell center = (*d_path0)[pos_idx + 60 < d_path0->size() ? pos_idx + 60
                                                                : d_path0->size() / 2];
  const int radius = 10;
  std::vector<Cell> obstacle;
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      if (dx * dx + dy * dy > radius * radius) continue;
      const auto x = static_cast<std::size_t>(static_cast<std::int64_t>(center.first) + dx);
      const auto y = static_cast<std::size_t>(static_cast<std::int64_t>(center.second) + dy);
      if (!grid.inBounds(x, y) || Cell{x, y} == pos || Cell{x, y} == goal) continue;
      if (grid.cost(x, y) == planning::kLethal) continue;
      grid.setCost(x, y, planning::kLethal);
      dstar.updateCell(x, y, planning::kLethal);
      obstacle.emplace_back(x, y);
    }
  }

  const auto d_repaired = dstar.plan(pos.first, pos.second);
  if (!d_repaired) {
    std::fprintf(stderr, "D* repair found no path\n");
    return 2;
  }
  std::printf("initial path %zu cells, expanded %zu; repair expanded %zu\n",
              a_path->size(), expanded.size(), dstar.nodesExpanded());

  std::FILE* f = std::fopen(out.c_str(), "w");
  if (!f) {
    std::fprintf(stderr, "cannot write %s\n", out.c_str());
    return 2;
  }
  std::fprintf(f, "SECTION meta\n%zu %zu\n", grid.width(), grid.height());
  dumpCells(f, "start", {start});
  dumpCells(f, "goal", {goal});
  dumpCells(f, "pos", {pos});
  dumpCells(f, "expanded", expanded);
  dumpCells(f, "astar_path", *a_path);
  dumpCells(f, "old_path", *d_path0);
  dumpCells(f, "obstacle", obstacle);
  dumpCells(f, "repaired_path", *d_repaired);
  std::fclose(f);
  std::printf("wrote %s\n", out.c_str());
  return 0;
}
