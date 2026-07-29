// core/include/planning/dstar_lite.hpp
//
// D* Lite (optimized version) after Koenig & Likhachev,
// "D* Lite", AAAI 2002. The search runs backward from the goal:
// g(s) estimates the cost from s to the goal, rhs(s) is the one-step
// lookahead value min over successors s' of c(s, s') + g(s'). Search
// state persists across replans; when cells change cost or the start
// moves along the path, only the affected portion of the search is
// repaired, which is the entire point of the algorithm.
//
// All costs use the exact integer metric from cost.hpp: priority-key
// ties between vertices are common and must compare exactly, which
// floating point cannot guarantee.
//
// Usage:
//   DStarLitePlanner planner(grid);
//   planner.setGoal(gx, gy);
//   auto path = planner.plan(sx, sy);       // initial plan
//   planner.updateCell(x, y, new_cost);     // map changed
//   auto repaired = planner.plan(sx2, sy2); // incremental repair
#pragma once

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "planning/cost.hpp"
#include "planning/grid.hpp"

namespace planning {

using Cell = std::pair<std::size_t, std::size_t>;
using Path = std::vector<Cell>;

class DStarLitePlanner {
public:
  // The planner keeps its own copy of the grid; apply map changes
  // through updateCell() so the search state stays consistent with it.
  explicit DStarLitePlanner(const Grid& grid, bool allow_diagonal = true);

  // Fixes a new goal; all search state is rebuilt on the next plan().
  void setGoal(std::size_t goal_x, std::size_t goal_y);

  // Plans from (start_x, start_y) to the current goal. The first call
  // after setGoal() computes a full solution; later calls reuse the
  // previous search and only repair what changed. Returns std::nullopt
  // when no path exists.
  [[nodiscard]] std::optional<Path> plan(std::size_t start_x, std::size_t start_y);

  // Applies a cost change to the planner's grid copy and updates the
  // affected vertices. Cheap; the real repair work happens lazily in
  // the next plan() call.
  void updateCell(std::size_t x, std::size_t y, std::uint8_t new_cost);

  [[nodiscard]] const Grid& grid() const noexcept { return grid_; }

  // Vertices expanded by the most recent plan() call.
  [[nodiscard]] std::size_t nodesExpanded() const noexcept { return nodes_expanded_; }

private:
  struct Key {
    CostT k1;
    CostT k2;
    [[nodiscard]] bool operator<(const Key& o) const noexcept {
      return k1 < o.k1 || (k1 == o.k1 && k2 < o.k2);
    }
    [[nodiscard]] bool operator==(const Key& o) const noexcept {
      return k1 == o.k1 && k2 == o.k2;
    }
  };

  struct OpenEntry {
    Key key;
    std::uint32_t idx;
  };

  Grid grid_;
  bool allow_diagonal_;

  bool has_goal_{false};
  bool initialized_{false};
  std::size_t goal_idx_{0};
  std::size_t start_idx_{0};  // current start, used by calcKey during search
  std::size_t last_start_idx_{0};
  CostT km_{0};

  std::vector<CostT> g_;
  std::vector<CostT> rhs_;
  std::vector<std::uint8_t> in_open_;
  std::vector<Key> open_key_;
  std::vector<OpenEntry> heap_;

  std::size_t nodes_expanded_{0};

  [[nodiscard]] CostT heuristic(std::size_t a_idx, std::size_t b_idx) const noexcept;
  [[nodiscard]] Key calcKey(std::size_t idx) const noexcept;
  [[nodiscard]] CostT edgeCost(std::size_t from_idx, std::size_t to_idx) const noexcept;
  [[nodiscard]] CostT minOverSuccessors(std::size_t idx) const noexcept;

  void pushOpen(std::size_t idx, Key key);
  void updateVertex(std::size_t idx);
  // Discards stale heap entries; returns false when the heap is empty.
  [[nodiscard]] bool peekTop(OpenEntry& out);
  void computeShortestPath();
  [[nodiscard]] std::optional<Path> extractPath() const;

  template <typename Fn>
  void forEachNeighbor(std::size_t idx, Fn&& fn) const;
};

}  // namespace planning
