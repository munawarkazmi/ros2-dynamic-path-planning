// src/astar_planner.cpp
#include <astar_planner/astar_planner.hpp>
#include <algorithm>
#include <cmath>
#include <functional>
#include <queue>
#include <unordered_map>
#include <utility>

namespace astar_planner {

AStarPlanner::AStarPlanner(double heuristic_weight, bool allow_diagonal)
    : heuristic_weight_(heuristic_weight), allow_diagonal_(allow_diagonal) {}

constexpr double AStarPlanner::heuristic(std::size_t x1, std::size_t y1,
                                         std::size_t x2, std::size_t y2) noexcept {
  const double dx = std::abs(static_cast<double>(x1) - x2);
  const double dy = std::abs(static_cast<double>(y1) - y2);
  // Octile distance (best for 8-connected grid)
  return heuristic_weight_ * ((dx + dy) + (std::sqrt(2.0) - 2.0) * std::min(dx, dy));
}

constexpr std::array<std::pair<std::size_t, std::size_t>, 8>
AStarPlanner::getNeighbors(std::size_t x, std::size_t y) const noexcept {
  std::array<std::pair<std::size_t, std::size_t>, 8> neighbors{};
  std::size_t idx = 0;

  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      if (dx == 0 && dy == 0) continue;
      if (!allow_diagonal_ && dx != 0 && dy != 0) continue;
      neighbors[idx++] = {static_cast<std::size_t>(static_cast<std::ptrdiff_t>(x) + dx),
                          static_cast<std::size_t>(static_cast<std::ptrdiff_t>(y) + dy)};
    }
  }
  if (idx < 8) {
    // Pad remaining with invalid sentinel if needed (not used)
    for (; idx < 8; ++idx) neighbors[idx] = {0, 0};
  }
  return neighbors;
}

void AStarPlanner::reconstructPath(Node* goal_node,
    std::vector<std::pair<std::size_t, std::size_t>>& path) const {
  for (Node* node = goal_node; node != nullptr; node = node->parent) {
    path.emplace_back(node->x, node->y);
  }
  std::reverse(path.begin(), path.end());
}

std::optional<std::vector<std::pair<std::size_t, std::size_t>>>
AStarPlanner::plan(const nav2_costmap_2d::Costmap2D& costmap,
                   std::size_t start_x, std::size_t start_y,
                   std::size_t goal_x, std::size_t goal_y) {
  nodes_expanded_ = 0;

  const std::size_t width = costmap.getSizeInCellsX();
  const std::size_t height = costmap.getSizeInCellsY();

  // Simple hash: x + y * width (fast and sufficient)
  auto hash = [width](std::size_t x, std::size_t y) { return y * width + x; };

  std::unordered_map<std::size_t, Node> nodes;
  auto cmp = [](const Node* a, const Node* b) { return a->f() > b->f(); };
  std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open(cmp);

  auto& start_node = nodes[hash(start_x, start_y)];
  start_node.x = start_x;
  start_node.y = start_y;
  start_node.g = 0.0;
  start_node.h = heuristic(start_x, start_y, goal_x, goal_y);
  start_node.parent = nullptr;
  open.push(&start_node);

  while (!open.empty()) {
    Node* current = open.top();
    open.pop();
    ++nodes_expanded_;

    if (current->x == goal_x && current->y == goal_y) {
      std::vector<std::pair<std::size_t, std::size_t>> path;
      reconstructPath(current, path);
      return path;
    }

    const auto neighbors = getNeighbors(current->x, current->y);
    for (const auto& [nx, ny] : neighbors) {
      if (nx >= width || ny >= height) continue;

      const unsigned char cost = costmap.getCost(nx, ny);
      if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE || cost == nav2_costmap_2d::NO_INFORMATION) {
        continue;
      }

      const double move_cost = (nx != current->x && ny != current->y) ? std::sqrt(2.0) : 1.0;
      const double tentative_g = current->g + move_cost * (1.0 + cost / 254.0);  // slight penalty for higher cost cells

      const std::size_t key = hash(nx, ny);
      auto it = nodes.find(key);
      if (it == nodes.end()) {
        auto& node = nodes[key];
        node.x = nx;
        node.y = ny;
        node.g = tentative_g;
        node.h = heuristic(nx, ny, goal_x, goal_y);
        node.parent = current;
        open.push(&node);
      } else if (tentative_g < it->second.g) {
        it->second.g = tentative_g;
        it->second.parent = current;
        open.push(&it->second);  // Note: classic A* allows duplicates 
      }
    }
  }

  return std::nullopt;  // No path found
}

}  // namespace astar_planner
