// core/src/astar.cpp
#include "planning/astar.hpp"

#include <algorithm>
#include <queue>

namespace planning {

std::optional<Path> AStarPlanner::plan(const Grid& grid,
                                       std::size_t start_x, std::size_t start_y,
                                       std::size_t goal_x, std::size_t goal_y) {
  nodes_expanded_ = 0;

  if (!grid.inBounds(start_x, start_y) || !grid.inBounds(goal_x, goal_y) ||
      !grid.traversable(start_x, start_y) || !grid.traversable(goal_x, goal_y)) {
    return std::nullopt;
  }

  const std::size_t n = grid.size();
  if (g_.size() != n) {
    g_.assign(n, kInfCost);
    parent_.assign(n, -1);
    stamp_.assign(n, 0);
    closed_.assign(n, 0);
    epoch_ = 0;
  }
  ++epoch_;

  auto touch = [&](std::size_t idx) {
    if (stamp_[idx] != epoch_) {
      stamp_[idx] = epoch_;
      g_[idx] = kInfCost;
      parent_[idx] = -1;
      closed_[idx] = 0;
    }
  };

  auto cmp = [](const OpenEntry& a, const OpenEntry& b) {
    if (a.f != b.f) return a.f > b.f;
    return a.g < b.g;  // prefer deeper nodes on f-ties
  };
  std::priority_queue<OpenEntry, std::vector<OpenEntry>, decltype(cmp)> open(cmp);

  const std::size_t start_idx = grid.index(start_x, start_y);
  const std::size_t goal_idx = grid.index(goal_x, goal_y);

  touch(start_idx);
  g_[start_idx] = 0;
  open.push({octile(start_x, start_y, goal_x, goal_y), 0,
             static_cast<std::uint32_t>(start_idx)});

  const std::size_t width = grid.width();
  const std::size_t height = grid.height();

  while (!open.empty()) {
    const OpenEntry top = open.top();
    open.pop();

    const std::size_t idx = top.idx;
    touch(idx);
    if (closed_[idx]) continue;  // stale duplicate
    closed_[idx] = 1;
    ++nodes_expanded_;

    if (idx == goal_idx) {
      Path path;
      for (std::int64_t cur = static_cast<std::int64_t>(idx); cur >= 0;
           cur = parent_[static_cast<std::size_t>(cur)]) {
        const auto c = static_cast<std::size_t>(cur);
        path.emplace_back(c % width, c / width);
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    const std::size_t x = idx % width;
    const std::size_t y = idx / width;

    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) continue;
        const bool diagonal = (dx != 0 && dy != 0);
        if (!allow_diagonal_ && diagonal) continue;

        const auto nx = static_cast<std::size_t>(static_cast<std::int64_t>(x) + dx);
        const auto ny = static_cast<std::size_t>(static_cast<std::int64_t>(y) + dy);
        if (nx >= width || ny >= height) continue;  // unsigned wrap covers < 0
        if (!grid.traversable(nx, ny)) continue;

        const std::size_t nidx = grid.index(nx, ny);
        touch(nidx);
        if (closed_[nidx]) continue;

        const CostT tentative_g = g_[idx] + stepCost(grid.cost(nx, ny), diagonal);
        if (tentative_g < g_[nidx]) {
          g_[nidx] = tentative_g;
          parent_[nidx] = static_cast<std::int64_t>(idx);
          open.push({tentative_g + octile(nx, ny, goal_x, goal_y), tentative_g,
                     static_cast<std::uint32_t>(nidx)});
        }
      }
    }
  }

  return std::nullopt;
}

}  // namespace planning
