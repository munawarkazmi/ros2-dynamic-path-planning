// core/tests/planners_gtest.cpp
//
// GoogleTest suite for the planning core. The properties checked here are the
// same ones tests/test_planners.cpp checks, against the same Dijkstra oracle
// and the same seeds, so coverage is unchanged. What the framework buys is:
//
//   1. One reported test case per seed, via TEST_P. A failure names the seed,
//      and a single scenario can be re-run in isolation with
//      --gtest_filter=*seed2007*  instead of rerunning the whole suite.
//   2. A real distinction between fatal and non-fatal checks. The hand-rolled
//      CHECK macro could only accumulate; here a malformed path stops that
//      scenario (ASSERT) while a merely suboptimal cost is recorded and the
//      run continues (EXPECT).
//   3. SCOPED_TRACE on the inner edit rounds and robot steps, so a failure
//      reports which round it happened in without threading that through
//      every message by hand.
//   4. The edge cases become six independently named invariants rather than
//      one function, so a red test says which invariant broke.
//
// The randomised stress binary (tests/fuzz_planners.cpp) is deliberately left
// alone. It is a long-running validation sweep whose job is to print a count
// for the record, not a unit test, and it should not be inside a unit test
// runner.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <optional>
#include <random>
#include <string>

#include "planning/astar.hpp"
#include "planning/cost.hpp"
#include "planning/dstar_lite.hpp"
#include "planning/grid.hpp"
#include "planning_test_support.hpp"

namespace {

using planning::AStarPlanner;
using planning::Cell;
using planning::CostT;
using planning::DStarLitePlanner;
using planning::Grid;
using planning::kInfCost;
using planning::kLethal;
using planning::Path;
using planning::testsupport::dijkstra;
using planning::testsupport::pathCost;
using planning::testsupport::randomFreeCell;
using planning::testsupport::randomGrid;

// Checks a planner result against the oracle. Fatal on a missing or malformed
// path, because every later assertion in the same scenario would be noise;
// non-fatal on the cost itself, so one suboptimal result does not hide the
// others in the same run.
void expectOptimal(const Grid& grid, const std::optional<Path>& path,
                   Cell start, Cell goal, CostT truth) {
  if (truth == kInfCost) {
    ASSERT_FALSE(path.has_value()) << "planner returned a path where none exists";
    return;
  }
  ASSERT_TRUE(path.has_value()) << "planner found no path although one exists";
  const auto cost = pathCost(grid, *path, start, goal);
  ASSERT_TRUE(cost.has_value())
      << "path is malformed: endpoints, bounds, traversability or adjacency";
  EXPECT_EQ(*cost, truth) << "path is legal but not cost-optimal";
}

// ASSERT_* inside a helper returns from the helper, not from the test, so a
// caller in a loop has to check for itself before continuing.
#define RETURN_IF_FATAL()                       \
  do {                                          \
    if (::testing::Test::HasFatalFailure()) return; \
  } while (0)

std::string seedName(const ::testing::TestParamInfo<unsigned>& info) {
  return "seed" + std::to_string(info.param);
}

// ---------------------------------------------------------------- A*

class AStarSeeded : public ::testing::TestWithParam<unsigned> {};

TEST_P(AStarSeeded, MatchesDijkstra) {
  std::mt19937 rng(GetParam());
  const Grid grid = randomGrid(rng, 60, 40, 0.25);
  const Cell start = randomFreeCell(rng, grid);
  const Cell goal = randomFreeCell(rng, grid);

  const auto dist = dijkstra(grid, start.first, start.second);
  const CostT truth = dist[grid.index(goal.first, goal.second)];

  AStarPlanner astar;
  const auto path = astar.plan(grid, start.first, start.second, goal.first, goal.second);
  expectOptimal(grid, path, start, goal, truth);
}

INSTANTIATE_TEST_SUITE_P(Seeds, AStarSeeded, ::testing::Range(1u, 26u), seedName);

// ------------------------------------------------- D* Lite, first plan

class DStarInitialSeeded : public ::testing::TestWithParam<unsigned> {};

TEST_P(DStarInitialSeeded, FirstPlanMatchesDijkstra) {
  std::mt19937 rng(GetParam());
  const Grid grid = randomGrid(rng, 60, 40, 0.25);
  const Cell start = randomFreeCell(rng, grid);
  const Cell goal = randomFreeCell(rng, grid);

  const auto dist = dijkstra(grid, start.first, start.second);
  const CostT truth = dist[grid.index(goal.first, goal.second)];

  DStarLitePlanner dstar(grid);
  dstar.setGoal(goal.first, goal.second);
  const auto path = dstar.plan(start.first, start.second);
  expectOptimal(grid, path, start, goal, truth);
}

INSTANTIATE_TEST_SUITE_P(Seeds, DStarInitialSeeded, ::testing::Range(1001u, 1026u),
                         seedName);

// ----------------------------- D* Lite, incremental repair after edits

// The core guarantee: after any sequence of edits, an incremental replan must
// be exactly as optimal as a fresh search on the same map.
class DStarIncrementalSeeded : public ::testing::TestWithParam<unsigned> {};

TEST_P(DStarIncrementalSeeded, RepairStaysOptimalAcrossEditRounds) {
  std::mt19937 rng(GetParam());
  Grid grid = randomGrid(rng, 80, 60, 0.2);
  const Cell start = randomFreeCell(rng, grid);
  const Cell goal = randomFreeCell(rng, grid);

  DStarLitePlanner dstar(grid);
  dstar.setGoal(goal.first, goal.second);
  auto path = dstar.plan(start.first, start.second);

  std::uniform_int_distribution<std::size_t> dx(0, grid.width() - 1);
  std::uniform_int_distribution<std::size_t> dy(0, grid.height() - 1);
  std::uniform_int_distribution<int> action(0, 2);
  std::uniform_int_distribution<int> cost_dist(0, 200);

  for (int round = 0; round < 10; ++round) {
    SCOPED_TRACE("edit round " + std::to_string(round));

    // A batch of random edits: block, unblock, or reprice cells.
    for (int e = 0; e < 12; ++e) {
      const std::size_t x = dx(rng);
      const std::size_t y = dy(rng);
      if (Cell{x, y} == start || Cell{x, y} == goal) continue;
      const int a = action(rng);
      const std::uint8_t c = (a == 0)   ? kLethal
                             : (a == 1) ? std::uint8_t{0}
                                        : static_cast<std::uint8_t>(cost_dist(rng));
      grid.setCost(x, y, c);
      dstar.updateCell(x, y, c);
    }
    // Also block a mid-path cell when we have one, to force repair.
    if (path && path->size() > 4) {
      const Cell mid = (*path)[path->size() / 2];
      if (mid != start && mid != goal) {
        grid.setCost(mid.first, mid.second, kLethal);
        dstar.updateCell(mid.first, mid.second, kLethal);
      }
    }

    path = dstar.plan(start.first, start.second);
    const auto dist = dijkstra(grid, start.first, start.second);
    const CostT truth = dist[grid.index(goal.first, goal.second)];

    expectOptimal(grid, path, start, goal, truth);
    RETURN_IF_FATAL();
  }
}

INSTANTIATE_TEST_SUITE_P(Seeds, DStarIncrementalSeeded, ::testing::Range(2001u, 2016u),
                         seedName);

// ------------------------------------------ D* Lite, start moves (km)

// Robot walks the path while obstacles appear ahead of it; every replan from
// the moved start must stay optimal, which exercises the km machinery.
class DStarMovingStartSeeded : public ::testing::TestWithParam<unsigned> {};

TEST_P(DStarMovingStartSeeded, ReplanFromMovedStartStaysOptimal) {
  std::mt19937 rng(GetParam());
  Grid grid = randomGrid(rng, 80, 60, 0.15);
  Cell pos = randomFreeCell(rng, grid);
  const Cell goal = randomFreeCell(rng, grid);

  DStarLitePlanner dstar(grid);
  dstar.setGoal(goal.first, goal.second);

  for (int step = 0; step < 20 && pos != goal; ++step) {
    SCOPED_TRACE("robot step " + std::to_string(step));

    auto path = dstar.plan(pos.first, pos.second);
    const auto dist = dijkstra(grid, pos.first, pos.second);
    const CostT truth = dist[grid.index(goal.first, goal.second)];

    if (truth == kInfCost) {
      ASSERT_FALSE(path.has_value()) << "planner returned a path where none exists";
      break;
    }
    expectOptimal(grid, path, pos, goal, truth);
    RETURN_IF_FATAL();

    // Advance along the path, then drop an obstacle further ahead.
    const std::size_t advance = std::min<std::size_t>(3, path->size() - 1);
    pos = (*path)[advance];
    if (path->size() > advance + 3) {
      const Cell ahead = (*path)[advance + 2];
      if (ahead != pos && ahead != goal) {
        grid.setCost(ahead.first, ahead.second, kLethal);
        dstar.updateCell(ahead.first, ahead.second, kLethal);
      }
    }
  }
}

INSTANTIATE_TEST_SUITE_P(Seeds, DStarMovingStartSeeded, ::testing::Range(3001u, 3011u),
                         seedName);

// ------------------------------------------------------- efficiency

// The property that justifies the algorithm existing: repairing after a local
// change must expand far fewer vertices than a fresh search. Without this,
// D* Lite is a slower A* with more state.
TEST(DStarLiteEfficiency, RepairExpandsFarFewerVerticesThanFreshSearch) {
  std::mt19937 rng(4000);
  Grid grid = randomGrid(rng, 300, 300, 0.10);
  grid.setCost(5, 5, 0);
  grid.setCost(295, 295, 0);

  DStarLitePlanner dstar(grid);
  dstar.setGoal(295, 295);
  const auto path = dstar.plan(5, 5);
  ASSERT_TRUE(path.has_value()) << "no initial path on a sparse 300x300 map";
  const std::size_t initial_expansions = dstar.nodesExpanded();

  const Cell mid = (*path)[path->size() / 2];
  grid.setCost(mid.first, mid.second, kLethal);
  dstar.updateCell(mid.first, mid.second, kLethal);

  const auto repaired = dstar.plan(5, 5);
  ASSERT_TRUE(repaired.has_value()) << "repair lost the path";
  const std::size_t repair_expansions = dstar.nodesExpanded();

  // A fresh planner on the modified map is the from-scratch baseline.
  DStarLitePlanner fresh(grid);
  fresh.setGoal(295, 295);
  const auto fresh_path = fresh.plan(5, 5);
  ASSERT_TRUE(fresh_path.has_value()) << "fresh replan found no path";
  const std::size_t scratch_expansions = fresh.nodesExpanded();

  EXPECT_LT(repair_expansions * 2, scratch_expansions)
      << "repair expanded " << repair_expansions << " vertices against "
      << scratch_expansions << " from scratch (initial plan " << initial_expansions
      << "): the repair is not behaving incrementally";

  const auto rc = pathCost(grid, *repaired, {5, 5}, {295, 295});
  const auto fc = pathCost(grid, *fresh_path, {5, 5}, {295, 295});
  ASSERT_TRUE(rc.has_value()) << "repaired path is malformed";
  ASSERT_TRUE(fc.has_value()) << "from-scratch path is malformed";
  EXPECT_EQ(*rc, *fc) << "repair and from-scratch disagree on cost";
}

// ------------------------------------------------------- edge cases

TEST(PlannerEdgeCases, AStarStartEqualsGoalIsASingleCellPath) {
  const Grid grid(20, 20, 0);
  AStarPlanner astar;
  const auto path = astar.plan(grid, 4, 4, 4, 4);
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(path->size(), 1u);
  EXPECT_EQ(path->front(), Cell(4, 4));
}

TEST(PlannerEdgeCases, DStarStartEqualsGoalIsASingleCellPath) {
  const Grid grid(20, 20, 0);
  DStarLitePlanner dstar(grid);
  dstar.setGoal(4, 4);
  const auto path = dstar.plan(4, 4);
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(path->size(), 1u);
  EXPECT_EQ(path->front(), Cell(4, 4));
}

TEST(PlannerEdgeCases, NeitherPlannerCrossesASolidWall) {
  Grid walled(20, 20, 0);
  for (std::size_t y = 0; y < 20; ++y) walled.setCost(10, y, kLethal);

  AStarPlanner astar;
  EXPECT_FALSE(astar.plan(walled, 2, 2, 18, 18).has_value()) << "A* crossed a solid wall";

  DStarLitePlanner dstar(walled);
  dstar.setGoal(18, 18);
  EXPECT_FALSE(dstar.plan(2, 2).has_value()) << "D* Lite crossed a solid wall";
}

TEST(PlannerEdgeCases, AStarRejectsLethalEndpoints) {
  Grid blocked(20, 20, 0);
  blocked.setCost(2, 2, kLethal);
  AStarPlanner astar;
  EXPECT_FALSE(astar.plan(blocked, 2, 2, 18, 18).has_value())
      << "A* planned out of a lethal start";
  EXPECT_FALSE(astar.plan(blocked, 18, 18, 2, 2).has_value())
      << "A* planned into a lethal goal";
}

TEST(PlannerEdgeCases, DStarFindsARouteOnceTheWallOpens) {
  Grid walled(20, 20, 0);
  for (std::size_t y = 0; y < 20; ++y) walled.setCost(10, y, kLethal);

  DStarLitePlanner dstar(walled);
  dstar.setGoal(18, 18);
  ASSERT_FALSE(dstar.plan(2, 2).has_value()) << "precondition: the wall should block";

  dstar.updateCell(10, 7, 0);
  EXPECT_TRUE(dstar.plan(2, 2).has_value())
      << "D* Lite did not find the route after the wall opened";
}

}  // namespace
