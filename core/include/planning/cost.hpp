// core/include/planning/cost.hpp
//
// Exact integer cost algebra shared by the planners.
//
// D* Lite's correctness depends on exact key comparisons: priorities of
// different vertices are often mathematically equal (sums over the same
// multiset of step costs), and with floating point those ties land ulps
// apart, which can bury a vertex that must be expanded behind one that
// must not. Integer costs make every comparison exact.
//
// A straight step costs 70 and a diagonal 99 (99/70 ~ 1.4143, the
// classic rational approximation of sqrt(2)), scaled by the destination
// cell's costmap value: cost(step) = base * (254 + cell). The octile
// heuristic uses the same constants with the minimum multiplier 254, so
// it is admissible and consistent under this metric.
#pragma once

#include <cstdint>
#include <cstdlib>
#include <limits>

#include "planning/grid.hpp"

namespace planning {

using CostT = std::int64_t;

inline constexpr CostT kStraight = 70;
inline constexpr CostT kDiagonal = 99;
inline constexpr CostT kInfCost = std::numeric_limits<CostT>::max();

// One traversable cell of base cost 0 entered by a straight step.
inline constexpr CostT kUnitCost = kStraight * 254;

inline constexpr CostT addSat(CostT a, CostT b) noexcept {
  return (a == kInfCost || b == kInfCost) ? kInfCost : a + b;
}

// Cost of stepping onto a traversable cell with costmap value `cell`.
inline constexpr CostT stepCost(std::uint8_t cell, bool diagonal) noexcept {
  return (diagonal ? kDiagonal : kStraight) * (254 + static_cast<CostT>(cell));
}

// Octile distance under the same metric, assuming free cells.
inline CostT octile(std::size_t x1, std::size_t y1,
                    std::size_t x2, std::size_t y2) noexcept {
  const CostT dx = std::abs(static_cast<CostT>(x1) - static_cast<CostT>(x2));
  const CostT dy = std::abs(static_cast<CostT>(y1) - static_cast<CostT>(y2));
  const CostT m = dx < dy ? dx : dy;
  return 254 * (kStraight * (dx + dy) + (kDiagonal - 2 * kStraight) * m);
}

// For reporting: cost in "free straight cells" units.
inline constexpr double costToCells(CostT c) noexcept {
  return static_cast<double>(c) / static_cast<double>(kUnitCost);
}

}  // namespace planning
