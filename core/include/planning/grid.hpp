// core/include/planning/grid.hpp
//
// Minimal occupancy grid shared by the planners. Cost semantics follow
// nav2_costmap_2d: 0 = free, 1..252 = increasing traversal penalty,
// 253 = inscribed, 254 = lethal, 255 = no information. A cell is
// traversable iff cost <= 253.
#pragma once

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace planning {

using Cell = std::pair<std::size_t, std::size_t>;
using Path = std::vector<Cell>;

inline constexpr std::uint8_t kInscribed = 253;
inline constexpr std::uint8_t kLethal = 254;
inline constexpr std::uint8_t kNoInformation = 255;

class Grid {
public:
  Grid(std::size_t width, std::size_t height, std::uint8_t fill = 0)
      : width_(width), height_(height), cells_(width * height, fill) {}

  [[nodiscard]] std::size_t width() const noexcept { return width_; }
  [[nodiscard]] std::size_t height() const noexcept { return height_; }
  [[nodiscard]] std::size_t size() const noexcept { return cells_.size(); }

  [[nodiscard]] std::size_t index(std::size_t x, std::size_t y) const noexcept {
    return y * width_ + x;
  }

  [[nodiscard]] bool inBounds(std::size_t x, std::size_t y) const noexcept {
    return x < width_ && y < height_;
  }

  [[nodiscard]] std::uint8_t cost(std::size_t x, std::size_t y) const noexcept {
    return cells_[index(x, y)];
  }

  void setCost(std::size_t x, std::size_t y, std::uint8_t cost) noexcept {
    cells_[index(x, y)] = cost;
  }

  [[nodiscard]] bool traversable(std::size_t x, std::size_t y) const noexcept {
    return cells_[index(x, y)] < kLethal;
  }

  [[nodiscard]] const std::vector<std::uint8_t>& data() const noexcept { return cells_; }
  [[nodiscard]] std::vector<std::uint8_t>& data() noexcept { return cells_; }

private:
  std::size_t width_;
  std::size_t height_;
  std::vector<std::uint8_t> cells_;
};

}  // namespace planning
