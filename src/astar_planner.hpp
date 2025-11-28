// include/astar_planner/astar_planner.hpp
#pragma once

#include <vector>
#include <array>
#include <optional>
#include <limits>
#include <cstdint>  
#include <nav2_costmap_2d/costmap_2d.h>

namespace astar_planner {

struct Node {
  std::size_t x, y;
  double g{std::numeric_limits<double>::infinity()};
  double rhs{std::numeric_limits<double>::infinity()};  // for D* Lite compatibility
  double h{0.0};
  Node* parent{nullptr};

  [[nodiscard]] constexpr double f() const noexcept { return g + h; }
  [[nodiscard]] constexpr double key() const noexcept { return std::min(g, rhs) + h; }

  // Perfect for priority_queue with strict weak ordering
  auto operator<=>(const Node&) const noexcept = default;
};

class AStarPlanner {
public:
  explicit AStarPlanner(double heuristic_weight = 1.0, bool allow_diagonal = true)
      : heuristic_weight_(heuristic_weight), allow_diagonal_(allow_diagonal) {}

  [[nodiscard]] std::optional<std::vector<std::pair<std::size_t, std::size_t>>>
  plan(const nav2_costmap_2d::Costmap2D& costmap,
       std::size_t start_x, std::size_t start_y,
       std::size_t goal_x, std::size_t goal_y);

  [[nodiscard]] std::size_t nodesExpanded() const noexcept { return nodes_expanded_; }

private:
  const double heuristic_weight_;
  const bool allow_diagonal_;
  std::size_t nodes_expanded_{0};

  [[nodiscard]] static constexpr double heuristic(std::size_t x1, std::size_t y1,
                                                   std::size_t x2, std::size_t y2) noexcept {
    const double dx = std::abs(static_cast<double>(x1) - x2);
    const double dy = std::abs(static_cast<double>(y1) - y2);
    return (dx + dy) + (1.414213562 - 2) * std::min(dx, dy);  // octile distance
  }

  [[nodiscard]] constexpr std::array<std::pair<std::size_t, std::size_t>, 8>
  getNeighbors(std::size_t x, std::size_t y) const noexcept;

  void reconstructPath(Node* goal_node,
                       std::vector<std::pair<std::size_t, std::size_t>>& path) const;
};

}  // namespace astar_planner
