// src/dstar_lite_planner.cpp
#include <dstar_lite/dstar_lite_planner.hpp>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>

namespace dstar_lite {

DStarLitePlanner::DStarLitePlanner(double heuristic_weight, bool allow_diagonal)
    : heuristic_weight_(heuristic_weight), allow_diagonal_(allow_diagonal) {
  open_ = PriorityQueue(KeyCompare{0.0});
}

constexpr double DStarLitePlanner::heuristic(std::size_t x1, std::size_t y1,
                                             std::size_t x2, std::size_t y2) noexcept {
  const double dx = std::abs(static_cast<double>(x1) - x2);
  const double dy = std::abs(static_cast<double>(y1) - y2);
  return heuristic_weight_ * ((dx + dy) + (std::sqrt(2.0) - 2.0) * std::min(dx, dy));
}

std::array<std::pair<std::size_t, std::size_t>, 8>
DStarLitePlanner::getNeighbors(std::size_t x, std::size_t y,
                               std::size_t width, std::size_t height) const noexcept {
  std::array<std::pair<std::size_t, std::size_t>, 8> neighbors{};
  std::size_t idx = 0;

  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      if (dx == 0 && dy == 0) continue;
      if (!allow_diagonal_ && dx != 0 && dy != 0) continue;

      const std::size_t nx = x + dx;
      const std::size_t ny = y + dy;
      if (nx < width && ny < height) {
        neighbors[idx++] = {nx, ny};
      }
    }
  }
  neighbors[idx] = {width, height};  // sentinel
  return neighbors;
}

Node* DStarLitePlanner::getNode(std::size_t x, std::size_t y, std::size_t width) {
  const std::size_t key = y * width + x;
  return &node_map_[key];
}

double DStarLitePlanner::costBetween(const Node* a, const Node* b) const noexcept {
  const double dx = std::abs(static_cast<double>(a->x) - b->x);
  const double dy = std::abs(static_cast<double>(a->y) - b->y);
  return (dx + dy > 1.5) ? std::sqrt(2.0) : 1.0;  // diagonal vs cardinal
}

void DStarLitePlanner::updateVertex(Node* u, const nav2_costmap_2d::Costmap2D& costmap,
                                    std::size_t goal_x, std::size_t goal_y) {
  if (u->g != u->rhs) {
    open_.push(u);
  }

  const std::size_t width = costmap.getSizeInCellsX();
  const auto neighbors = getNeighbors(u->x, u->y, width, costmap.getSizeInCellsY());

  for (const auto& [nx, ny] : neighbors) {
    if (nx >= width) break;  // sentinel reached

    const unsigned char cost = costmap.getCost(nx, ny);
    if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE || cost == nav2_costmap_2d::NO_INFORMATION) {
      continue;
    }

    Node* succ = getNode(nx, ny, width);
    const double c = costBetween(u, succ) * (1.0 + cost / 254.0);

    if (u->rhs + c < succ->rhs) {
      succ->rhs = u->rhs + c;
      succ->parent = u;
      updateVertex(succ, costmap, goal_x, goal_y);
    }
  }
}

void DStarLitePlanner::computeShortestPath(const nav2_costmap_2d::Costmap2D& costmap,
                                           std::size_t goal_x, std::size_t goal_y) {
  const std::size_t width = costmap.getSizeInCellsX();

  while (!open_.empty()) {
    Node* u = open_.top();
    open_.pop();
    ++nodes_expanded_;

    const double old_rhs = u->rhs;

    if (u->g > u->rhs) {
      u->g = u->rhs;
    } else {
      u->g = std::numeric_limits<double>::infinity();
      updateVertex(u, costmap, goal_x, goal_y);
    }

    if (old_rhs != u->rhs) {
      updateVertex(u, costmap, goal_x, goal_y);
    }

    // Early exit: goal is locally consistent and has best key
    Node* goal_node = getNode(goal_x, goal_y, width);
    if (goal_node->g == goal_node->rhs &&
        (!open_.empty() ? open_.top()->key(km_).first >= goal_node->key(km_).first
                        : true)) {
      break;
    }
  }
}

std::optional<std::vector<std::pair<std::size_t, std::size_t>>>
DStarLitePlanner::plan(const nav2_costmap_2d::Costmap2D& costmap,
                       std::size_t start_x, std::size_t start_y,
                       std::size_t goal_x, std::size_t goal_y) {
  const std::size_t width = costmap.getSizeInCellsX();
  node_map_.clear();
  while (!open_.empty()) open_.pop();

  km_ = 0.0;
  nodes_expanded_ = 0;
  ++replan_count_;

  Node* start = getNode(start_x, start_y, width);
  Node* goal = getNode(goal_x, goal_y, width);

  start->rhs = 0.0;
  start->g = std::numeric_limits<double>::infinity();
  goal->h = 0.0;

  open_.push(start);
  open_.key_compare().km = km_;

  computeShortestPath(costmap, goal_x, goal_y);

  if (goal->g == std::numeric_limits<double>::infinity()) {
    return std::nullopt;
  }

  std::vector<std::pair<std::size_t, std::size_t>> path;
  for (Node* n = goal; n != nullptr; n = n->parent) {
    path.emplace_back(n->x, n->y);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

void DStarLitePlanner::updateCostmap(const nav2_costmap_2d::Costmap2D& costmap) {
  const std::size_t width = costmap.getSizeInCellsX();
  km_ += heuristic(0, 0, width / 2, costmap.getSizeInCellsY() / 2);  // simple km update

  open_.key_compare().km = km_;
  ++replan_count_;
}

}  // namespace dstar_lite
