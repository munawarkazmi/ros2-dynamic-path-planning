// core/tests/test_planners.cpp
//
// Correctness tests for the planning core. Ground truth is a reference
// Dijkstra over the same edge model; the integer cost metric makes all
// comparisons exact. The critical properties:
//   1. A* returns cost-optimal paths.
//   2. D* Lite's first plan returns cost-optimal paths.
//   3. After arbitrary map edits, D* Lite's incremental replan is
//      exactly as optimal as planning from scratch.
//   4. An incremental replan after a local change expands far fewer
//      vertices than a from-scratch search (the reason D* Lite exists).
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <queue>
#include <random>
#include <vector>

#include "planning/astar.hpp"
#include "planning/cost.hpp"
#include "planning/dstar_lite.hpp"
#include "planning/grid.hpp"

namespace {

using planning::AStarPlanner;
using planning::Cell;
using planning::CostT;
using planning::DStarLitePlanner;
using planning::Grid;
using planning::kInfCost;
using planning::Path;
using planning::stepCost;

int g_failures = 0;

#define CHECK(cond, ...)                                        \
  do {                                                          \
    if (!(cond)) {                                              \
      ++g_failures;                                             \
      std::printf("FAIL %s:%d  ", __FILE__, __LINE__);          \
      std::printf(__VA_ARGS__);                                 \
      std::printf("\n");                                        \
    }                                                           \
  } while (0)

// Reference Dijkstra from (sx, sy); returns per-cell distance.
std::vector<CostT> dijkstra(const Grid& grid, std::size_t sx, std::size_t sy) {
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

// Validates a path (adjacency, traversability, endpoints) and returns
// its total cost, or nullopt if the path is malformed.
std::optional<CostT> pathCost(const Grid& grid, const Path& path,
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

Grid randomGrid(std::mt19937& rng, std::size_t w, std::size_t h,
                double obstacle_density) {
  Grid grid(w, h, 0);
  std::uniform_real_distribution<double> coin(0.0, 1.0);
  std::uniform_int_distribution<int> cost_dist(0, 200);
  for (std::size_t y = 0; y < h; ++y) {
    for (std::size_t x = 0; x < w; ++x) {
      if (coin(rng) < obstacle_density) {
        grid.setCost(x, y, planning::kLethal);
      } else if (coin(rng) < 0.3) {
        grid.setCost(x, y, static_cast<std::uint8_t>(cost_dist(rng)));
      }
    }
  }
  return grid;
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

void testAStarMatchesDijkstra() {
  for (unsigned seed = 1; seed <= 25; ++seed) {
    std::mt19937 rng(seed);
    const Grid grid = randomGrid(rng, 60, 40, 0.25);
    const Cell start = randomFreeCell(rng, grid);
    const Cell goal = randomFreeCell(rng, grid);

    const auto dist = dijkstra(grid, start.first, start.second);
    const CostT truth = dist[grid.index(goal.first, goal.second)];

    AStarPlanner astar;
    const auto path = astar.plan(grid, start.first, start.second, goal.first, goal.second);

    if (truth == kInfCost) {
      CHECK(!path.has_value(), "seed %u: A* found a path where none exists", seed);
      continue;
    }
    CHECK(path.has_value(), "seed %u: A* found no path but one exists", seed);
    if (!path) continue;
    const auto cost = pathCost(grid, *path, start, goal);
    CHECK(cost.has_value(), "seed %u: A* path malformed", seed);
    if (cost) {
      CHECK(*cost == truth, "seed %u: A* cost %lld != optimal %lld", seed,
            static_cast<long long>(*cost), static_cast<long long>(truth));
    }
  }
}

void testDStarInitialMatchesDijkstra() {
  for (unsigned seed = 1; seed <= 25; ++seed) {
    std::mt19937 rng(seed + 1000);
    const Grid grid = randomGrid(rng, 60, 40, 0.25);
    const Cell start = randomFreeCell(rng, grid);
    const Cell goal = randomFreeCell(rng, grid);

    const auto dist = dijkstra(grid, start.first, start.second);
    const CostT truth = dist[grid.index(goal.first, goal.second)];

    DStarLitePlanner dstar(grid);
    dstar.setGoal(goal.first, goal.second);
    const auto path = dstar.plan(start.first, start.second);

    if (truth == kInfCost) {
      CHECK(!path.has_value(), "seed %u: D* found a path where none exists", seed);
      continue;
    }
    CHECK(path.has_value(), "seed %u: D* found no path but one exists", seed);
    if (!path) continue;
    const auto cost = pathCost(grid, *path, start, goal);
    CHECK(cost.has_value(), "seed %u: D* path malformed", seed);
    if (cost) {
      CHECK(*cost == truth, "seed %u: D* cost %lld != optimal %lld", seed,
            static_cast<long long>(*cost), static_cast<long long>(truth));
    }
  }
}

// The core guarantee: after any sequence of edits, an incremental
// replan must be exactly as optimal as a fresh search on the same map.
void testDStarIncrementalMatchesScratch() {
  for (unsigned seed = 1; seed <= 15; ++seed) {
    std::mt19937 rng(seed + 2000);
    Grid grid = randomGrid(rng, 80, 60, 0.2);
    const Cell start = randomFreeCell(rng, grid);
    Cell goal = randomFreeCell(rng, grid);

    DStarLitePlanner dstar(grid);
    dstar.setGoal(goal.first, goal.second);
    auto path = dstar.plan(start.first, start.second);

    std::uniform_int_distribution<std::size_t> dx(0, grid.width() - 1);
    std::uniform_int_distribution<std::size_t> dy(0, grid.height() - 1);
    std::uniform_int_distribution<int> action(0, 2);
    std::uniform_int_distribution<int> cost_dist(0, 200);

    for (int round = 0; round < 10; ++round) {
      // A batch of random edits: block, unblock, or reprice cells.
      for (int e = 0; e < 12; ++e) {
        const std::size_t x = dx(rng);
        const std::size_t y = dy(rng);
        if (Cell{x, y} == start || Cell{x, y} == goal) continue;
        const int a = action(rng);
        const std::uint8_t c = (a == 0)   ? planning::kLethal
                               : (a == 1) ? std::uint8_t{0}
                                          : static_cast<std::uint8_t>(cost_dist(rng));
        grid.setCost(x, y, c);
        dstar.updateCell(x, y, c);
      }
      // Also block a mid-path cell when we have one, to force repair.
      if (path && path->size() > 4) {
        const Cell mid = (*path)[path->size() / 2];
        if (mid != start && mid != goal) {
          grid.setCost(mid.first, mid.second, planning::kLethal);
          dstar.updateCell(mid.first, mid.second, planning::kLethal);
        }
      }

      path = dstar.plan(start.first, start.second);
      const auto dist = dijkstra(grid, start.first, start.second);
      const CostT truth = dist[grid.index(goal.first, goal.second)];

      if (truth == kInfCost) {
        CHECK(!path.has_value(),
              "seed %u round %d: incremental D* found impossible path", seed, round);
        continue;
      }
      CHECK(path.has_value(),
            "seed %u round %d: incremental D* missed existing path", seed, round);
      if (!path) continue;
      const auto cost = pathCost(grid, *path, start, goal);
      CHECK(cost.has_value(), "seed %u round %d: path malformed", seed, round);
      if (cost) {
        CHECK(*cost == truth,
              "seed %u round %d: incremental cost %lld != optimal %lld",
              seed, round, static_cast<long long>(*cost), static_cast<long long>(truth));
      }
    }
  }
}

// Robot walks the path while obstacles appear ahead of it; every replan
// from the moved start must stay optimal (exercises the km machinery).
void testDStarMovingStart() {
  for (unsigned seed = 1; seed <= 10; ++seed) {
    std::mt19937 rng(seed + 3000);
    Grid grid = randomGrid(rng, 80, 60, 0.15);
    Cell pos = randomFreeCell(rng, grid);
    const Cell goal = randomFreeCell(rng, grid);

    DStarLitePlanner dstar(grid);
    dstar.setGoal(goal.first, goal.second);

    for (int step = 0; step < 20 && pos != goal; ++step) {
      auto path = dstar.plan(pos.first, pos.second);
      const auto dist = dijkstra(grid, pos.first, pos.second);
      const CostT truth = dist[grid.index(goal.first, goal.second)];

      if (truth == kInfCost) {
        CHECK(!path.has_value(), "seed %u step %d: found impossible path", seed, step);
        break;
      }
      CHECK(path.has_value(), "seed %u step %d: missed existing path", seed, step);
      if (!path) break;
      const auto cost = pathCost(grid, *path, pos, goal);
      CHECK(cost.has_value(), "seed %u step %d: path malformed", seed, step);
      if (cost) {
        CHECK(*cost == truth, "seed %u step %d: cost %lld != optimal %lld", seed, step,
              static_cast<long long>(*cost), static_cast<long long>(truth));
      }

      // Advance along the path, then drop an obstacle further ahead.
      const std::size_t advance = std::min<std::size_t>(3, path->size() - 1);
      pos = (*path)[advance];
      if (path->size() > advance + 3) {
        const Cell ahead = (*path)[advance + 2];
        if (ahead != pos && ahead != goal) {
          grid.setCost(ahead.first, ahead.second, planning::kLethal);
          dstar.updateCell(ahead.first, ahead.second, planning::kLethal);
        }
      }
    }
  }
}

// The efficiency property that justifies the algorithm: repairing after
// a local change must expand far fewer vertices than a fresh search.
void testDStarIncrementalIsCheaper() {
  std::mt19937 rng(4000);
  Grid grid = randomGrid(rng, 300, 300, 0.10);
  grid.setCost(5, 5, 0);
  grid.setCost(295, 295, 0);

  DStarLitePlanner dstar(grid);
  dstar.setGoal(295, 295);
  auto path = dstar.plan(5, 5);
  CHECK(path.has_value(), "efficiency test: no initial path on sparse map");
  if (!path) return;
  const std::size_t initial_expansions = dstar.nodesExpanded();

  const Cell mid = (*path)[path->size() / 2];
  grid.setCost(mid.first, mid.second, planning::kLethal);
  dstar.updateCell(mid.first, mid.second, planning::kLethal);

  auto repaired = dstar.plan(5, 5);
  CHECK(repaired.has_value(), "efficiency test: repair lost the path");
  const std::size_t repair_expansions = dstar.nodesExpanded();

  // Fresh planner on the modified map = the from-scratch baseline.
  DStarLitePlanner fresh(grid);
  fresh.setGoal(295, 295);
  auto fresh_path = fresh.plan(5, 5);
  CHECK(fresh_path.has_value(), "efficiency test: fresh replan found no path");
  const std::size_t scratch_expansions = fresh.nodesExpanded();

  CHECK(repair_expansions * 2 < scratch_expansions,
        "repair expanded %zu vs %zu from scratch (initial %zu) - not incremental",
        repair_expansions, scratch_expansions, initial_expansions);

  if (repaired && fresh_path) {
    const auto rc = pathCost(grid, *repaired, {5, 5}, {295, 295});
    const auto fc = pathCost(grid, *fresh_path, {5, 5}, {295, 295});
    CHECK(rc && fc && *rc == *fc, "repair and scratch disagree on cost");
  }
}

void testEdgeCases() {
  Grid grid(20, 20, 0);

  // start == goal
  AStarPlanner astar;
  const auto p1 = astar.plan(grid, 4, 4, 4, 4);
  CHECK(p1 && p1->size() == 1 && p1->front() == Cell(4, 4),
        "A* start==goal should be a single-cell path");

  DStarLitePlanner dstar(grid);
  dstar.setGoal(4, 4);
  const auto p2 = dstar.plan(4, 4);
  CHECK(p2 && p2->size() == 1 && p2->front() == Cell(4, 4),
        "D* start==goal should be a single-cell path");

  // Full wall: no path.
  Grid walled(20, 20, 0);
  for (std::size_t y = 0; y < 20; ++y) walled.setCost(10, y, planning::kLethal);
  CHECK(!astar.plan(walled, 2, 2, 18, 18).has_value(), "A* crossed a solid wall");
  DStarLitePlanner dstar2(walled);
  dstar2.setGoal(18, 18);
  CHECK(!dstar2.plan(2, 2).has_value(), "D* crossed a solid wall");

  // Blocked endpoints.
  Grid blocked(20, 20, 0);
  blocked.setCost(2, 2, planning::kLethal);
  CHECK(!astar.plan(blocked, 2, 2, 18, 18).has_value(), "A* planned from lethal start");
  CHECK(!astar.plan(blocked, 18, 18, 2, 2).has_value(), "A* planned into lethal goal");

  // Wall opens up: D* must find the new route incrementally.
  DStarLitePlanner dstar3(walled);
  dstar3.setGoal(18, 18);
  CHECK(!dstar3.plan(2, 2).has_value(), "precondition: wall blocks");
  dstar3.updateCell(10, 7, 0);
  const auto reopened = dstar3.plan(2, 2);
  CHECK(reopened.has_value(), "D* did not find path after wall opened");
}

}  // namespace

int main() {
  testAStarMatchesDijkstra();
  testDStarInitialMatchesDijkstra();
  testDStarIncrementalMatchesScratch();
  testDStarMovingStart();
  testDStarIncrementalIsCheaper();
  testEdgeCases();

  if (g_failures == 0) {
    std::printf("all tests passed\n");
    return 0;
  }
  std::printf("%d check(s) failed\n", g_failures);
  return 1;
}
