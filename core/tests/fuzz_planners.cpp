// core/tests/fuzz_planners.cpp
//
// Randomized fuzz of D* Lite's incremental replanning, validated
// against a reference Dijkstra with exact integer equality.
//
// Eight scenario modes (every combination of three ingredients) x 3000
// seeds; each scenario runs up to 8 edit/replan rounds on a small grid:
//   bit 0: cost repricing allowed (not just block/unblock)
//   bit 1: batched edits (8 per round instead of 1)
//   bit 2: moving start (the robot advances along its path, so km grows)
// Every replan must exactly match Dijkstra on the mutated map - both
// the reachability verdict and the optimal cost - and the returned
// path must be well-formed.
//
// This is the harness that originally exposed the floating-point
// priority-key tie bug fixed by the integer cost metric (see
// cost.hpp); it is committed so the scenario count it reports stays
// reproducible. Scale with FUZZ_SEEDS (default 3000 per mode).
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <queue>
#include <random>
#include <vector>

#include "planning/cost.hpp"
#include "planning/dstar_lite.hpp"
#include "planning/grid.hpp"

namespace {

using planning::Cell;
using planning::CostT;
using planning::DStarLitePlanner;
using planning::Grid;
using planning::kInfCost;
using planning::Path;
using planning::stepCost;

constexpr std::size_t W = 10;
constexpr std::size_t H = 8;

std::vector<CostT> dijkstra(const Grid& grid, std::size_t sx, std::size_t sy) {
  std::vector<CostT> dist(grid.size(), kInfCost);
  if (!grid.traversable(sx, sy)) return dist;
  using QE = std::pair<CostT, std::size_t>;
  std::priority_queue<QE, std::vector<QE>, std::greater<>> q;
  dist[grid.index(sx, sy)] = 0;
  q.push({0, grid.index(sx, sy)});
  while (!q.empty()) {
    const auto [d, idx] = q.top();
    q.pop();
    if (d > dist[idx]) continue;
    const std::size_t x = idx % W, y = idx / W;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) continue;
        const auto nx = static_cast<std::size_t>(static_cast<std::int64_t>(x) + dx);
        const auto ny = static_cast<std::size_t>(static_cast<std::int64_t>(y) + dy);
        if (nx >= W || ny >= H || !grid.traversable(nx, ny)) continue;
        const CostT nd = d + stepCost(grid.cost(nx, ny), dx != 0 && dy != 0);
        if (nd < dist[grid.index(nx, ny)]) {
          dist[grid.index(nx, ny)] = nd;
          q.push({nd, grid.index(nx, ny)});
        }
      }
    }
  }
  return dist;
}

// Path validity + exact cost, or nullopt if malformed.
std::optional<CostT> pathCost(const Grid& grid, const Path& path,
                              Cell start, Cell goal) {
  if (path.empty() || path.front() != start || path.back() != goal) return std::nullopt;
  CostT total = 0;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const auto [x, y] = path[i];
    if (!grid.inBounds(x, y) || !grid.traversable(x, y)) return std::nullopt;
    if (i == 0) continue;
    const auto dx = static_cast<std::int64_t>(x) - static_cast<std::int64_t>(path[i - 1].first);
    const auto dy = static_cast<std::int64_t>(y) - static_cast<std::int64_t>(path[i - 1].second);
    if ((dx == 0 && dy == 0) || std::abs(dx) > 1 || std::abs(dy) > 1) return std::nullopt;
    total += stepCost(grid.cost(x, y), dx != 0 && dy != 0);
  }
  return total;
}

struct Stats {
  long long scenarios = 0;
  long long replans = 0;
  long long edits = 0;
  int failures = 0;
};

void runMode(int mode, unsigned seeds, Stats& st) {
  const bool reprice = mode & 1;
  const bool batch = mode & 2;
  const bool moving = mode & 4;

  for (unsigned seed = 1; seed <= seeds; ++seed) {
    std::mt19937 rng(seed);
    Grid grid(W, H, 0);
    std::uniform_real_distribution<double> coin(0, 1);
    for (std::size_t y = 0; y < H; ++y)
      for (std::size_t x = 0; x < W; ++x)
        if (coin(rng) < 0.2) grid.setCost(x, y, planning::kLethal);
    const Cell goal{W - 1, H - 1};
    grid.setCost(0, 0, 0);
    grid.setCost(goal.first, goal.second, 0);

    DStarLitePlanner dstar(grid);
    dstar.setGoal(goal.first, goal.second);
    Cell pos{0, 0};
    auto path = dstar.plan(0, 0);
    if (!path && moving) continue;  // nowhere to walk; skip like the original harness
    ++st.scenarios;

    std::uniform_int_distribution<std::size_t> dx(0, W - 1), dy(0, H - 1);
    std::uniform_int_distribution<int> act(0, 2);
    std::uniform_int_distribution<int> price(1, 200);

    auto randomCost = [&]() -> std::uint8_t {
      const int a = reprice ? act(rng) : act(rng) % 2;
      if (a == 0) return planning::kLethal;
      if (a == 1) return 0;
      return static_cast<std::uint8_t>(price(rng));
    };

    for (int round = 0; round < 8; ++round) {
      const std::size_t x = dx(rng), y = dy(rng);
      if (Cell{x, y} == pos || Cell{x, y} == goal) continue;
      const std::uint8_t c = randomCost();
      grid.setCost(x, y, c);
      dstar.updateCell(x, y, c);
      ++st.edits;
      if (batch) {
        for (int e = 0; e < 7; ++e) {
          const std::size_t ex = dx(rng), ey = dy(rng);
          if (Cell{ex, ey} == pos || Cell{ex, ey} == goal) continue;
          const std::uint8_t c2 = randomCost();
          grid.setCost(ex, ey, c2);
          dstar.updateCell(ex, ey, c2);
          ++st.edits;
        }
      }

      path = dstar.plan(pos.first, pos.second);
      ++st.replans;
      const auto dist = dijkstra(grid, pos.first, pos.second);
      const CostT truth = dist[grid.index(goal.first, goal.second)];

      bool bad;
      if (truth == kInfCost) {
        bad = path.has_value();
      } else if (!path) {
        bad = true;
      } else {
        const auto cost = pathCost(grid, *path, pos, goal);
        bad = !cost || *cost != truth;
      }
      if (bad) {
        ++st.failures;
        std::printf("FAIL mode=%d seed=%u round=%d edit(%zu,%zu)->%u truth=%s path=%s\n",
                    mode, seed, round, x, y, c,
                    truth == kInfCost ? "unreachable" : "reachable",
                    path ? "found" : "none");
        break;
      }

      if (moving && path && path->size() > 1 &&
          grid.traversable((*path)[1].first, (*path)[1].second)) {
        pos = (*path)[1];
      }
    }
  }
}

}  // namespace

int main() {
  const char* env = std::getenv("FUZZ_SEEDS");
  const unsigned seeds = env ? static_cast<unsigned>(std::atoi(env)) : 3000;

  Stats st;
  for (int mode = 0; mode < 8; ++mode) runMode(mode, seeds, st);

  std::printf("fuzz: %lld scenarios (8 modes x %u seeds), %lld validated replans, "
              "%lld edits, %d failures\n",
              st.scenarios, seeds, st.replans, st.edits, st.failures);
  return st.failures == 0 ? 0 : 1;
}
