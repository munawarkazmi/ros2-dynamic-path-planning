// core/src/dstar_lite.cpp
#include "planning/dstar_lite.hpp"

#include <algorithm>

namespace planning {

namespace {

struct HeapCmp {
  // std::push_heap/pop_heap build a max-heap; invert to get a min-heap.
  bool operator()(const auto& a, const auto& b) const noexcept { return b.key < a.key; }
};

}  // namespace

DStarLitePlanner::DStarLitePlanner(const Grid& grid, bool allow_diagonal)
    : grid_(grid), allow_diagonal_(allow_diagonal) {
  const std::size_t n = grid_.size();
  g_.assign(n, kInfCost);
  rhs_.assign(n, kInfCost);
  in_open_.assign(n, 0);
  open_key_.assign(n, Key{0, 0});
}

CostT DStarLitePlanner::heuristic(std::size_t a_idx, std::size_t b_idx) const noexcept {
  const std::size_t w = grid_.width();
  if (!allow_diagonal_) {
    const CostT dx = std::abs(static_cast<CostT>(a_idx % w) - static_cast<CostT>(b_idx % w));
    const CostT dy = std::abs(static_cast<CostT>(a_idx / w) - static_cast<CostT>(b_idx / w));
    return 254 * kStraight * (dx + dy);
  }
  return octile(a_idx % w, a_idx / w, b_idx % w, b_idx / w);
}

DStarLitePlanner::Key DStarLitePlanner::calcKey(std::size_t idx) const noexcept {
  const CostT m = std::min(g_[idx], rhs_[idx]);
  return {addSat(m, heuristic(start_idx_, idx) + km_), m};
}

CostT DStarLitePlanner::edgeCost(std::size_t from_idx, std::size_t to_idx) const noexcept {
  const std::size_t w = grid_.width();
  const std::size_t to_x = to_idx % w;
  const std::size_t to_y = to_idx / w;
  if (!grid_.traversable(to_x, to_y)) return kInfCost;
  const bool diagonal = (from_idx % w != to_x) && (from_idx / w != to_y);
  return stepCost(grid_.cost(to_x, to_y), diagonal);
}

template <typename Fn>
void DStarLitePlanner::forEachNeighbor(std::size_t idx, Fn&& fn) const {
  const std::size_t w = grid_.width();
  const std::size_t h = grid_.height();
  const std::size_t x = idx % w;
  const std::size_t y = idx / w;

  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      if (dx == 0 && dy == 0) continue;
      if (!allow_diagonal_ && dx != 0 && dy != 0) continue;
      const auto nx = static_cast<std::size_t>(static_cast<std::int64_t>(x) + dx);
      const auto ny = static_cast<std::size_t>(static_cast<std::int64_t>(y) + dy);
      if (nx >= w || ny >= h) continue;  // unsigned wrap covers < 0
      fn(grid_.index(nx, ny));
    }
  }
}

CostT DStarLitePlanner::minOverSuccessors(std::size_t idx) const noexcept {
  CostT best = kInfCost;
  forEachNeighbor(idx, [&](std::size_t s) {
    const CostT c = addSat(edgeCost(idx, s), g_[s]);
    if (c < best) best = c;
  });
  return best;
}

void DStarLitePlanner::pushOpen(std::size_t idx, Key key) {
  open_key_[idx] = key;
  in_open_[idx] = 1;
  heap_.push_back({key, static_cast<std::uint32_t>(idx)});
  std::push_heap(heap_.begin(), heap_.end(), HeapCmp{});
}

void DStarLitePlanner::updateVertex(std::size_t idx) {
  if (g_[idx] != rhs_[idx]) {
    pushOpen(idx, calcKey(idx));
  } else {
    in_open_[idx] = 0;  // logical removal; stale heap entries are skipped on pop
  }
}

bool DStarLitePlanner::peekTop(OpenEntry& out) {
  while (!heap_.empty()) {
    const OpenEntry& top = heap_.front();
    if (in_open_[top.idx] && top.key == open_key_[top.idx]) {
      out = top;
      return true;
    }
    std::pop_heap(heap_.begin(), heap_.end(), HeapCmp{});
    heap_.pop_back();
  }
  return false;
}

void DStarLitePlanner::computeShortestPath() {
  while (true) {
    OpenEntry top{};
    const bool has_top = peekTop(top);
    const bool start_consistent = (g_[start_idx_] == rhs_[start_idx_]);
    if (start_consistent && (!has_top || !(top.key < calcKey(start_idx_)))) break;
    if (!has_top) break;  // start inconsistent but nothing left to expand: no path

    std::pop_heap(heap_.begin(), heap_.end(), HeapCmp{});
    heap_.pop_back();

    const std::size_t u = top.idx;
    const Key k_old = top.key;
    const Key k_new = calcKey(u);

    if (k_old < k_new) {  // key outdated by km growth: reorder, don't expand
      pushOpen(u, k_new);
      continue;
    }

    in_open_[u] = 0;
    ++nodes_expanded_;

    if (g_[u] > rhs_[u]) {
      // Overconsistent: g takes the improved value and propagates to
      // predecessors.
      g_[u] = rhs_[u];
      forEachNeighbor(u, [&](std::size_t p) {
        if (p != goal_idx_) {
          const CostT cand = addSat(edgeCost(p, u), g_[u]);
          if (cand < rhs_[p]) rhs_[p] = cand;
        }
        updateVertex(p);
      });
    } else {
      // Underconsistent: invalidate g and recompute every vertex whose
      // rhs relied on the old value. The equality test is exact because
      // both sides are integer sums of the same terms.
      const CostT g_old = g_[u];
      g_[u] = kInfCost;
      auto relax = [&](std::size_t p) {
        if (p != goal_idx_ && rhs_[p] == addSat(edgeCost(p, u), g_old)) {
          rhs_[p] = minOverSuccessors(p);
        }
        updateVertex(p);
      };
      forEachNeighbor(u, [&](std::size_t p) { relax(p); });
      relax(u);
    }
  }
}

void DStarLitePlanner::setGoal(std::size_t goal_x, std::size_t goal_y) {
  goal_idx_ = grid_.index(goal_x, goal_y);
  has_goal_ = true;
  initialized_ = false;  // forces full re-initialization in the next plan()
}

std::optional<Path> DStarLitePlanner::plan(std::size_t start_x, std::size_t start_y) {
  nodes_expanded_ = 0;

  if (!has_goal_ || !grid_.inBounds(start_x, start_y) ||
      !grid_.traversable(start_x, start_y)) {
    return std::nullopt;
  }

  const std::size_t start = grid_.index(start_x, start_y);

  if (!initialized_) {
    // First plan for this goal: initialize per Koenig & Likhachev.
    std::fill(g_.begin(), g_.end(), kInfCost);
    std::fill(rhs_.begin(), rhs_.end(), kInfCost);
    std::fill(in_open_.begin(), in_open_.end(), 0);
    heap_.clear();
    km_ = 0;
    rhs_[goal_idx_] = 0;
    start_idx_ = start;
    last_start_idx_ = start;
    initialized_ = true;
    pushOpen(goal_idx_, calcKey(goal_idx_));
  } else if (start != last_start_idx_) {
    // Start moved: grow km so keys computed for the old start remain
    // valid lower bounds under the new one.
    km_ += heuristic(last_start_idx_, start);
    last_start_idx_ = start;
    start_idx_ = start;
  } else {
    start_idx_ = start;
  }

  computeShortestPath();

  if (rhs_[start_idx_] == kInfCost) return std::nullopt;
  return extractPath();
}

void DStarLitePlanner::updateCell(std::size_t x, std::size_t y, std::uint8_t new_cost) {
  if (!grid_.inBounds(x, y) || grid_.cost(x, y) == new_cost) return;
  grid_.setCost(x, y, new_cost);

  if (!initialized_) return;  // next plan() initializes fresh anyway

  // Cost changes at cell w alter the edges into w, so w's neighbors'
  // rhs values may change; w's own vertex may (dis)appear if the
  // change crosses the lethal threshold. Recomputing rhs from its
  // definition covers every case.
  const std::size_t w_idx = grid_.index(x, y);
  auto refresh = [&](std::size_t v) {
    if (v != goal_idx_) {
      const std::size_t gw = grid_.width();
      rhs_[v] = grid_.traversable(v % gw, v / gw) ? minOverSuccessors(v) : kInfCost;
    }
    updateVertex(v);
  };
  refresh(w_idx);
  forEachNeighbor(w_idx, [&](std::size_t v) { refresh(v); });
}

std::optional<Path> DStarLitePlanner::extractPath() const {
  Path path;
  std::size_t cur = start_idx_;
  const std::size_t w = grid_.width();
  path.emplace_back(cur % w, cur / w);

  // Follow the steepest descent of c(s, s') + g(s'); bounded by the
  // grid size to guard against a corrupted gradient.
  for (std::size_t steps = 0; cur != goal_idx_; ++steps) {
    if (steps > grid_.size()) return std::nullopt;
    CostT best = kInfCost;
    std::size_t next = cur;
    forEachNeighbor(cur, [&](std::size_t s) {
      const CostT c = addSat(edgeCost(cur, s), g_[s]);
      if (c < best) {
        best = c;
        next = s;
      }
    });
    if (best == kInfCost) return std::nullopt;
    cur = next;
    path.emplace_back(cur % w, cur / w);
  }
  return path;
}

}  // namespace planning
