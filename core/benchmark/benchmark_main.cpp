// core/benchmark/benchmark_main.cpp
//
// Fair replanning benchmark: A* (from scratch) vs D* Lite (incremental)
// on the repository's occupancy map.
//
// Both planners face byte-identical conditions. Per trial:
//   - a random reachable start/goal pair is sampled (seeded RNG),
//   - event 0: both planners produce the initial plan (timed),
//   - each later event: the simulated robot advances along the current
//     reference path, a circular obstacle appears further ahead ON that
//     path (guaranteeing the environment changes where it matters), and
//     both planners replan from the same position on the same map.
// A* replans from scratch; D* Lite absorbs the changed cells through
// updateCell() and repairs its previous search. The D* Lite timing
// includes updateCell(), so its full incremental workload is measured.
//
// The reference path used for robot advancement and obstacle placement
// is A*'s latest plan. Both planners optimize the same exact integer
// metric, so their path costs must agree on every event; the benchmark
// verifies this (cost_match column) - a built-in validity check that
// the comparison is between two optimal planners, not a fast-but-wrong
// one and a slow-but-right one.
//
// Measurement order between the two planners alternates per event to
// cancel cache-warmth effects.
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "planning/astar.hpp"
#include "planning/cost.hpp"
#include "planning/dstar_lite.hpp"
#include "planning/grid.hpp"

using planning::AStarPlanner;
using planning::Cell;
using planning::CostT;
using planning::costToCells;
using planning::DStarLitePlanner;
using planning::Grid;
using planning::kInfCost;
using planning::octile;
using planning::Path;

namespace {

// Loads a P5 PGM and binarizes it at the given occupancy threshold:
// occupancy p = (255 - pixel) / 255; p > thresh -> lethal, else free.
//
// The repository's map is a textured render rather than a SLAM-grade
// trinary map - nearly half its pixels sit between the standard
// free/occupied thresholds - so the nav2-style trinary conversion
// would shred the free space. Binarizing at 50% keeps 98.6% of the
// free space in one connected component.
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
  f.get();  // single whitespace after maxval
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

struct Args {
  std::string map = "maps/indoor_grid.pgm";
  std::string out = "reports/results/replan_benchmark.csv";
  int trials = 200;
  int events = 8;
  unsigned seed = 42;
  int radius = 6;       // obstacle radius in cells (0.30 m at 0.05 m/cell)
  int advance = 20;     // robot advancement per event, in path cells
  CostT min_dist_cells = 300;  // min start-goal octile separation
};

double toMs(std::chrono::steady_clock::duration d) {
  return std::chrono::duration<double, std::milli>(d).count();
}

Cell randomFreeCell(std::mt19937& rng, const Grid& grid) {
  std::uniform_int_distribution<std::size_t> dx(0, grid.width() - 1);
  std::uniform_int_distribution<std::size_t> dy(0, grid.height() - 1);
  while (true) {
    const std::size_t x = dx(rng);
    const std::size_t y = dy(rng);
    if (grid.traversable(x, y)) return {x, y};
  }
}

// Applies a filled circle of lethal cost; returns the changed cells.
std::vector<Cell> stampObstacle(Grid& grid, Cell center, int radius,
                                Cell protect_a, Cell protect_b) {
  std::vector<Cell> changed;
  const auto cx = static_cast<std::int64_t>(center.first);
  const auto cy = static_cast<std::int64_t>(center.second);
  for (std::int64_t dy = -radius; dy <= radius; ++dy) {
    for (std::int64_t dx = -radius; dx <= radius; ++dx) {
      if (dx * dx + dy * dy > static_cast<std::int64_t>(radius) * radius) continue;
      const auto x = static_cast<std::size_t>(cx + dx);
      const auto y = static_cast<std::size_t>(cy + dy);
      if (!grid.inBounds(x, y)) continue;
      if (Cell{x, y} == protect_a || Cell{x, y} == protect_b) continue;
      if (grid.cost(x, y) == planning::kLethal) continue;
      grid.setCost(x, y, planning::kLethal);
      changed.emplace_back(x, y);
    }
  }
  return changed;
}

double median(std::vector<double> v) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const std::size_t n = v.size();
  return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

CostT pathCost(const Grid& grid, const Path& path) {
  CostT total = 0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const bool diagonal = path[i].first != path[i - 1].first &&
                          path[i].second != path[i - 1].second;
    total += planning::stepCost(grid.cost(path[i].first, path[i].second), diagonal);
  }
  return total;
}

}  // namespace

int main(int argc, char** argv) {
  Args args;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string k = argv[i];
    const std::string v = argv[i + 1];
    if (k == "--map") args.map = v;
    else if (k == "--out") args.out = v;
    else if (k == "--trials") args.trials = std::stoi(v);
    else if (k == "--events") args.events = std::stoi(v);
    else if (k == "--seed") args.seed = static_cast<unsigned>(std::stoul(v));
    else if (k == "--radius") args.radius = std::stoi(v);
    else if (k == "--advance") args.advance = std::stoi(v);
    else { std::fprintf(stderr, "unknown arg %s\n", k.c_str()); return 2; }
  }

  auto base_grid = loadPgm(args.map, 0.5);
  if (!base_grid) {
    std::fprintf(stderr, "failed to load %s\n", args.map.c_str());
    return 2;
  }
  std::printf("map: %s  %zux%zu cells\n", args.map.c_str(),
              base_grid->width(), base_grid->height());
  std::printf("trials=%d events=%d seed=%u radius=%d advance=%d\n",
              args.trials, args.events, args.seed, args.radius, args.advance);

  std::ofstream csv(args.out);
  if (!csv) {
    std::fprintf(stderr, "cannot write %s\n", args.out.c_str());
    return 2;
  }
  csv << "trial,event,seed,pos_x,pos_y,obst_x,obst_y,radius,changed_cells,"
         "astar_ms,astar_expanded,astar_cost_cells,"
         "dstar_ms,dstar_expanded,dstar_cost_cells,cost_match\n";

  std::mt19937 rng(args.seed);
  AStarPlanner astar;

  std::vector<double> astar_replan_ms, dstar_replan_ms;
  std::vector<double> astar_initial_ms, dstar_initial_ms;
  long long astar_replan_exp = 0, dstar_replan_exp = 0;
  int replan_events = 0, mismatches = 0, dstar_wins = 0;

  for (int trial = 0; trial < args.trials; ++trial) {
    Grid grid = *base_grid;

    // Sample a reachable start/goal pair with enough separation.
    Cell start{}, goal{};
    Path ref_path;
    for (int attempt = 0; ; ++attempt) {
      if (attempt >= 1000) {
        std::fprintf(stderr, "trial %d: no reachable start/goal found\n", trial);
        return 2;
      }
      start = randomFreeCell(rng, grid);
      goal = randomFreeCell(rng, grid);
      if (octile(start.first, start.second, goal.first, goal.second) <
          args.min_dist_cells * planning::kUnitCost) {
        continue;
      }
      auto p = astar.plan(grid, start.first, start.second, goal.first, goal.second);
      if (p) { ref_path = std::move(*p); break; }
    }

    DStarLitePlanner dstar(grid);
    dstar.setGoal(goal.first, goal.second);
    Cell pos = start;

    // Event 0: initial plans, timed.
    auto t0 = std::chrono::steady_clock::now();
    auto ap = astar.plan(grid, pos.first, pos.second, goal.first, goal.second);
    auto t1 = std::chrono::steady_clock::now();
    auto dp = dstar.plan(pos.first, pos.second);
    auto t2 = std::chrono::steady_clock::now();

    const double a_init = toMs(t1 - t0);
    const double d_init = toMs(t2 - t1);
    astar_initial_ms.push_back(a_init);
    dstar_initial_ms.push_back(d_init);

    csv << trial << ",0," << args.seed << ',' << pos.first << ',' << pos.second
        << ",-1,-1,0,0,"
        << a_init << ',' << astar.nodesExpanded() << ','
        << (ap ? costToCells(pathCost(grid, *ap)) : -1.0) << ','
        << d_init << ',' << dstar.nodesExpanded() << ','
        << (dp ? costToCells(pathCost(grid, *dp)) : -1.0) << ','
        << ((ap && dp && pathCost(grid, *ap) == pathCost(grid, *dp)) || (!ap && !dp) ? 1 : 0)
        << '\n';
    if (!ap || !dp) continue;
    ref_path = *ap;

    for (int event = 1; event <= args.events; ++event) {
      // Advance the robot along the reference path.
      const std::size_t pos_idx = std::min<std::size_t>(
          static_cast<std::size_t>(args.advance), ref_path.size() - 1);
      pos = ref_path[pos_idx];
      if (pos == goal) break;

      // Obstacle ahead of the robot on the reference path.
      const std::size_t lo = std::min(pos_idx + args.radius + 4, ref_path.size() - 1);
      const std::size_t hi = std::min(lo + 40, ref_path.size() - 1);
      if (lo >= hi) break;
      std::uniform_int_distribution<std::size_t> pick(lo, hi);
      const Cell center = ref_path[pick(rng)];
      const auto changed = stampObstacle(grid, center, args.radius, pos, goal);

      // Both planners now replan on the identical modified grid.
      double a_ms = 0.0, d_ms = 0.0;
      std::optional<Path> a_path, d_path;
      const bool astar_first = (event % 2) == 1;
      for (int which = 0; which < 2; ++which) {
        const bool run_astar = (which == 0) == astar_first;
        if (run_astar) {
          const auto s = std::chrono::steady_clock::now();
          a_path = astar.plan(grid, pos.first, pos.second, goal.first, goal.second);
          a_ms = toMs(std::chrono::steady_clock::now() - s);
        } else {
          const auto s = std::chrono::steady_clock::now();
          for (const auto& [cx, cy] : changed) {
            dstar.updateCell(cx, cy, planning::kLethal);
          }
          d_path = dstar.plan(pos.first, pos.second);
          d_ms = toMs(std::chrono::steady_clock::now() - s);
        }
      }

      const CostT a_cost = a_path ? pathCost(grid, *a_path) : kInfCost;
      const CostT d_cost = d_path ? pathCost(grid, *d_path) : kInfCost;
      const bool match = (a_cost == d_cost);
      if (!match) ++mismatches;

      csv << trial << ',' << event << ',' << args.seed << ','
          << pos.first << ',' << pos.second << ','
          << center.first << ',' << center.second << ','
          << args.radius << ',' << changed.size() << ','
          << a_ms << ',' << astar.nodesExpanded() << ','
          << (a_path ? costToCells(a_cost) : -1.0) << ','
          << d_ms << ',' << dstar.nodesExpanded() << ','
          << (d_path ? costToCells(d_cost) : -1.0) << ','
          << (match ? 1 : 0) << '\n';

      if (!a_path || !d_path) break;  // goal cut off; next trial

      astar_replan_ms.push_back(a_ms);
      dstar_replan_ms.push_back(d_ms);
      astar_replan_exp += static_cast<long long>(astar.nodesExpanded());
      dstar_replan_exp += static_cast<long long>(dstar.nodesExpanded());
      ++replan_events;
      if (d_ms < a_ms) ++dstar_wins;

      ref_path = *a_path;
    }
  }

  auto mean = [](const std::vector<double>& v) {
    double s = 0;
    for (double x : v) s += x;
    return v.empty() ? 0.0 : s / static_cast<double>(v.size());
  };

  std::printf("\n=== initial plans (%zu trials) ===\n", astar_initial_ms.size());
  std::printf("A*      mean %.3f ms  median %.3f ms\n",
              mean(astar_initial_ms), median(astar_initial_ms));
  std::printf("D* Lite mean %.3f ms  median %.3f ms\n",
              mean(dstar_initial_ms), median(dstar_initial_ms));

  std::printf("\n=== replans after path-blocking obstacle (%d events) ===\n", replan_events);
  std::printf("A*      mean %.3f ms  median %.3f ms  expansions/event %.0f\n",
              mean(astar_replan_ms), median(astar_replan_ms),
              replan_events ? static_cast<double>(astar_replan_exp) / replan_events : 0.0);
  std::printf("D* Lite mean %.3f ms  median %.3f ms  expansions/event %.0f\n",
              mean(dstar_replan_ms), median(dstar_replan_ms),
              replan_events ? static_cast<double>(dstar_replan_exp) / replan_events : 0.0);
  std::printf("speedup (mean A* / mean D*): %.2fx\n",
              mean(dstar_replan_ms) > 0 ? mean(astar_replan_ms) / mean(dstar_replan_ms) : 0.0);
  std::printf("D* Lite faster on %d / %d replan events\n", dstar_wins, replan_events);
  std::printf("path cost mismatches (must be 0): %d\n", mismatches);

  return mismatches == 0 ? 0 : 1;
}
