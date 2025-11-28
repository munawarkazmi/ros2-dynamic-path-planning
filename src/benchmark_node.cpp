// src/benchmark_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <fstream>
#include <random>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <astar_planner/astar_planner.hpp>
#include <dstar_lite/dstar_lite_planner.hpp>

using namespace std::chrono_literals;

class BenchmarkNode : public rclcpp::Node {
public:
  BenchmarkNode() : Node("path_planning_benchmark") {
    // Parameters
    this->declare_parameter("trials", 100);
    this->declare_parameter("seed", 42);
    this->declare_parameter("dynamic_obstacles_per_trial", 8);
    this->declare_parameter("output_csv", "reports/results/travel_time_comparison.csv");

    trials_ = this->get_parameter("trials").as_int();
    seed_ = this->get_parameter("seed").as_int();
    dynamic_obstacles_ = this->get_parameter("dynamic_obstacles_per_trial").as_int();
    csv_path_ = this->get_parameter("output_csv").as_string();

    // Seed RNG for full reproducibility
    gen_.seed(static_cast<unsigned int>(seed_));

    // Load costmap
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
    costmap_ros_->on_configure(this->get_node_base_interface());

    RCLCPP_INFO(this->get_logger(), "Starting benchmark with %ld trials", trials_);
    RCLCPP_INFO(this->get_logger(), "RNG seed: %d → 100%% reproducible results", seed_);
    RCLCPP_INFO(this->get_logger(), "Dynamic obstacles per trial: %d", dynamic_obstacles_);

    runBenchmark();

    RCLCPP_INFO(this->get_logger(), "Benchmark complete! Results saved to %s", csv_path_.c_str());
    rclcpp::shutdown();
  }

private:
  void runBenchmark() {
    std::ofstream csv(csv_path_, std::ios::out);
    csv << "Trial,AStar_Time_s,AStar_Nodes,DStar_Time_s,DStar_Nodes,DStar_Replans,Improvement_%\n";

    astar_planner::AStarPlanner astar(1.0, true);
    dstar_lite::DStarLitePlanner dstar(1.0, true);

    double total_astar = 0.0, total_dstar = 0.0;

    for (int i = 0; i < trials_; ++i) {
      auto start = getRandomStart();
      auto goal = getRandomGoal(start);

      // Reset dynamic changes
      costmap_ros_->resetLayers();

      // A* (static environment)
      auto t1 = std::chrono::high_resolution_clock::now();
      auto path_astar = astar.plan(*costmap_ros_->getCostmap(), start.first, start.second, goal.first, goal.second);
      auto t2 = std::chrono::high_resolution_clock::now();
      double time_astar = std::chrono::duration<double>(t2 - t1).count();

      // D* Lite with dynamic obstacles
      auto t3 = std::chrono::high_resolution_clock::now();
      auto path_dstar = dstar.plan(*costmap_ros_->getCostmap(), start.first, start.second, goal.first, goal.second);

      injectDynamicObstacles(dynamic_obstacles_);
      dstar.updateCostmap(*costmap_ros_->getCostmap());
      path_dstar = dstar.plan(*costmap_ros_->getCostmap(), start.first, start.second, goal.first, goal.second);

      auto t4 = std::chrono::high_resolution_clock::now();
      double time_dstar = std::chrono::duration<double>(t4 - t3).count();

      double improvement = (time_astar > 0.0) ? (time_astar - time_dstar) / time_astar * 100.0 : 0.0;

      csv << i << ","
          << std::fixed << std::setprecision(6) << time_astar << ","
          << astar.nodesExpanded() << ","
          << time_dstar << ","
          << dstar.nodesExpanded() << ","
          << dstar.replansTriggered() << ","
          << std::setprecision(2) << improvement << "\n";

      total_astar += time_astar;
      total_dstar += time_dstar;

      // Live, responsive terminal output
      std::cout << "\r"
                << std::left << std::setw(6) << (i + 1)
                << "A*: " << std::right << std::fixed << std::setprecision(4) << std::setw(8) << time_astar << "s  "
                << "D* Lite: " << std::setw(8) << time_dstar << "s  "
                << "↑ " << std::setw(6) << std::setprecision(1) << improvement << "%  "
                << "[Replans: " << std::setw(3) << dstar.replansTriggered() << "]" << std::flush;
    }

    double avg_improvement = (total_astar - total_dstar) / total_astar * 100.0;

    std::cout << "\n\nFINAL RESULT: D* Lite is "
              << std::fixed << std::setprecision(2) << avg_improvement
              << "% faster on average!\n" << std::endl;

    csv.close();
  }

  std::pair<std::size_t, std::size_t> getRandomStart() {
    return {dist_x_(gen_), dist_y_(gen_)};
  }

  std::pair<std::size_t, std::size_t> getRandomGoal(std::pair<std::size_t, std::size_t> start) {
    std::pair<std::size_t, std::size_t> goal;
    do {
      goal = {dist_x_(gen_), dist_y_(gen_)};
    } while (std::abs(static_cast<int>(goal.first) - static_cast<int>(start.first)) +
                 std::abs(static_cast<int>(goal.second) - static_cast<int>(start.second)) < 300);
    return goal;
  }

  void injectDynamicObstacles(int count) {
    auto* map = costmap_ros_->getCostmap();
    std::uniform_int_distribution<std::size_t> dist(50, map->getSizeInCellsX() - 50);

    for (int i = 0; i < count; ++i) {
      std::size_t cx = dist(gen_), cy = dist(gen_);
      for (int dx = -15; dx <= 15; ++dx) {
        for (int dy = -15; dy <= 15; ++dy) {
          std::size_t x = cx + dx, y = cy + dy;
          if (x < map->getSizeInCellsX() && y < map->getSizeInCellsY()) {
            if (dx*dx + dy*dy < 225) {  // ~15 cell radius
              map->setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
            }
          }
        }
      }
    }
  }

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  int trials_{100};
  int seed_{42};
  int dynamic_obstacles_{8};
  std::string csv_path_;

  std::random_device rd_;
  std::mt19937 gen_{rd_()};
  std::uniform_int_distribution<std::size_t> dist_x_{50, 750};
  std::uniform_int_distribution<std::size_t> dist_y_{50, 550};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BenchmarkNode>());
  return 0;
}
