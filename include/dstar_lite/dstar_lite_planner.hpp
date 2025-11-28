// include/dstar_lite/dstar_lite_planner.hpp
#pragma once

#include <vector>
#include <array>
#include <optional>
#include <limits>
#include <unordered_map>
#include <queue>
#include <nav2_costmap_2d/costmap_2d.h>

namespace dstar_lite {

struct Node {
  std::size_t x, y;
  double g{std::numeric_limits<double>::infinity()};
  double rhs{std::numeric_limits<double>::infinity()};
  double h{0.0};
  Node* parent{nullptr};

  [[nodiscard]] constexpr std::pair<double, double> key(double km) const noexcept {
    const double min_grhs = std::min(g, rhs);
    return {min_grhs + h + km, min_grhs};
  }

  auto operator<=>(const Node&) const noexcept = default;
};

class DStarLitePlanner {
public:
  explicit DStarLitePlanner(double heuristic_weight = 1.0, bool allow_diagonal = true);

  [[nodiscard]] std::optional<std::vector<std::pair<std::size_t, std::size_t>>>
  plan(const nav2_costmap_2d::Costmap2D& costmap,
       std::size_t start_x, std::size_t start_y,
       std::size_t goal_x, std::size_t goal_y);

  // Call this when costmap changes (e.g. new obstacle detected)
  void updateCostmap(const nav2_costmap_2d::Costmap2D& costmap);

  [[nodiscard]] std::size_t nodesExpanded() const noexcept { return nodes_expanded_; }
  [[nodiscard]] std::size_t replansTriggered() const noexcept { return replan_count_; }

private:
  const double heuristic_weight_;
  const bool allow_diagonal_;
  double km_{0.0};  // key modifier for D* Lite

  std::size_t nodes_expanded_{0};
  std::size_t replan_count_{0};

  struct KeyCompare {
    double km;
    bool operator()(const Node* a, const Node* b) const noexcept {
      const auto [k1a, k1b] = a->key(km);
      const auto [k2a, k2b] = b->key(km);
      return (k1a > k2a) || (k1a == k2a && k1b > k2b);
    }
  };

  using PriorityQueue = std::priority_queue<Node*, std::vector<Node*>, KeyCompare>;

  std::unordered_map<std::size_t, Node> node_map_;  // key = x + y * width
  PriorityQueue open_;

  [[nodiscard]] static constexpr double heuristic(std::size_t x1, std::size_t y1,
                                                   std::size_t x2, std::size_t y2) noexcept {
    const double dx = std::abs(static_cast<double>(x1) - x2);
    const double dy = std::abs(static_cast<double>(y1) - y2);
    return heuristic_weight_ * ((dx + dy) + (1.414213562 - 2.0) * std::min(dx, dy));
  }

  [[nodiscard]] std::array<std::pair<std::size_t, std::size_t>, 8>
  getNeighbors(std::size_t x, std::size_t y, std::size_t width, std::size_t height) const noexcept;

  void updateVertex(Node* u, const nav2_costmap_2d::Costmap2D& costmap,
                    std::size_t goal_x, std::size_t goal_y);
  void computeShortestPath(const nav2_costmap_2d::Costmap2D& costmap,
                           std::size_t goal_x, std::size_t goal_y);

  [[nodiscard]] Node* getNode(std::size_t x, std::size_t y, std::size_t width);
  [[nodiscard]] double costBetween(const Node* a, const Node* b) const noexcept;
};

}  // namespace dstar_lite
