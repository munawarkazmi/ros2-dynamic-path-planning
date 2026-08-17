// core/tests/planning_test_support.hpp
//
// Shared reference oracle and random-grid generators for the planner test
// suites. Kept separate from any one test binary so that the oracle has a
// single definition: if the oracle and a planner ever disagree, exactly one
// of them is wrong, and it must not be possible for two copies of the oracle
// to drift apart.
//
// The oracle is an independent Dijkstra over the same edge model as the
// planners. Because the cost metric is integral (see planning/cost.hpp),
// every comparison against it is exact rather than a tolerance.

#ifndef PLANNING_TEST_SUPPORT_HPP_
#define PLANNING_TEST_SUPPORT_HPP_

#include <cstdint>
#include <cstdlib>
#include <functional>
#include <optional>
#include <queue>
#include <random>
#include <utility>
#include <vector>

#include "planning/cost.hpp"
#include "planning/grid.hpp"

namespace planning::testsupport {

// Reference Dijkstra from (sx, sy); returns per-cell distance, kInfCost for
// cells that are unreachable.
inline std::vector<CostT> dijkstra(const Grid& grid, std::size_t sx, std::size_t sy) {
  const std::size_t w = grid.width();
  const std::size_t h = grid.height();
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
    const std::size_t x = idx % w;
    const std::size_t y = idx / w;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) continue;
        const auto nx = static_cast<std::size_t>(static_cast<std::int64_t>(x) + dx);
        const auto ny = static_cast<std::size_t>(static_cast<std::int64_t>(y) + dy);
        if (nx >= w || ny >= h || !grid.traversable(nx, ny)) continue;
        const CostT nd = d + stepCost(grid.cost(nx, ny), dx != 0 && dy != 0);
        const std::size_t nidx = grid.index(nx, ny);
        if (nd < dist[nidx]) {
          dist[nidx] = nd;
          q.push({nd, nidx});
        }
      }
    }
  }
  return dist;
}

// Validates a path (endpoints, bounds, traversability, 8-connected adjacency)
// and returns its total cost, or nullopt if the path is malformed. A planner
// returning a cheap but illegal path must fail, so the cost is recomputed here
// rather than trusted from the planner.
inline std::optional<CostT> pathCost(const Grid& grid, const Path& path,
                                     Cell start, Cell goal) {
  if (path.empty() || path.front() != start || path.back() != goal) return std::nullopt;
  CostT total = 0;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const auto [x, y] = path[i];
    if (!grid.inBounds(x, y) || !grid.traversable(x, y)) return std::nullopt;
    if (i == 0) continue;
    const auto [px, py] = path[i - 1];
    const auto dx = static_cast<std::int64_t>(x) - static_cast<std::int64_t>(px);
    const auto dy = static_cast<std::int64_t>(y) - static_cast<std::int64_t>(py);
    if (dx == 0 && dy == 0) return std::nullopt;
    if (std::abs(dx) > 1 || std::abs(dy) > 1) return std::nullopt;
    total += stepCost(grid.cost(x, y), dx != 0 && dy != 0);
  }
  return total;
}

// A grid with `obstacle_density` lethal cells and a scattering of non-zero
// traversal costs, so that optimality is about cost rather than only about
// reachability.
inline Grid randomGrid(std::mt19937& rng, std::size_t w, std::size_t h,
                       double obstacle_density) {
  Grid grid(w, h, 0);
  std::uniform_real_distribution<double> coin(0.0, 1.0);
  std::uniform_int_distribution<int> cost_dist(0, 200);
  for (std::size_t y = 0; y < h; ++y) {
    for (std::size_t x = 0; x < w; ++x) {
      if (coin(rng) < obstacle_density) {
        grid.setCost(x, y, kLethal);
      } else if (coin(rng) < 0.3) {
        grid.setCost(x, y, static_cast<std::uint8_t>(cost_dist(rng)));
      }
    }
  }
  return grid;
}

inline Cell randomFreeCell(std::mt19937& rng, const Grid& grid) {
  std::uniform_int_distribution<std::size_t> dx(0, grid.width() - 1);
  std::uniform_int_distribution<std::size_t> dy(0, grid.height() - 1);
  while (true) {
    const std::size_t x = dx(rng);
    const std::size_t y = dy(rng);
    if (grid.traversable(x, y)) return {x, y};
  }
}

}  // namespace planning::testsupport

#endif  // PLANNING_TEST_SUPPORT_HPP_
