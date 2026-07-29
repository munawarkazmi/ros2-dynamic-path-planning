// core/include/planning/astar.hpp
//
// A* on an 8-connected grid with an octile-distance heuristic over the
// exact integer cost metric from cost.hpp. Each call to plan() searches
// from scratch; internal buffers are reused across calls (epoch-stamped)
// so repeated planning does not pay allocation costs.
#pragma once

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "planning/cost.hpp"
#include "planning/grid.hpp"

namespace planning {

class AStarPlanner {
public:
  explicit AStarPlanner(bool allow_diagonal = true)
      : allow_diagonal_(allow_diagonal) {}

  // Plans start -> goal on the given grid. Returns std::nullopt when no
  // path exists or when either endpoint is not traversable.
  [[nodiscard]] std::optional<Path> plan(const Grid& grid,
                                         std::size_t start_x, std::size_t start_y,
                                         std::size_t goal_x, std::size_t goal_y);

  [[nodiscard]] std::size_t nodesExpanded() const noexcept { return nodes_expanded_; }

  // Cells expanded by the most recent plan() call (for visualization).
  [[nodiscard]] std::vector<Cell> expandedCells(const Grid& grid) const;

private:
  struct OpenEntry {
    CostT f;
    CostT g;
    std::uint32_t idx;
  };

  bool allow_diagonal_;
  std::size_t nodes_expanded_{0};

  // Epoch-stamped per-cell state; a cell's entry is valid only when its
  // stamp matches the current epoch, which makes per-plan reset O(1).
  std::vector<CostT> g_;
  std::vector<std::int64_t> parent_;
  std::vector<std::uint32_t> stamp_;
  std::vector<std::uint8_t> closed_;
  std::uint32_t epoch_{0};
};

}  // namespace planning
